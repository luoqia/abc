#include "partNtk.h"
#include <sys/wait.h>
#include <sys/resource.h>
#include <unistd.h>
#include <sys/types.h>
#include <unordered_map>
#include <fcntl.h>
#include <cstdio>
#include <sys/stat.h>
#include <linux/limits.h> // PATH_MAX
#include <libgen.h>		  // dirname

using Clock = std::chrono::high_resolution_clock;

namespace ymc
{
	// Task 16 Stage 3 behavior-neutral telemetry (child rows). Gated by
	// PIF_TELEMETRY_DIR; no file is opened when the variable is unset.
	namespace
	{
		const char *pifChildTelemetryDir()
		{
			static const char *d = getenv("PIF_TELEMETRY_DIR");
			return (d && *d) ? d : nullptr;
		}
		void pifChildTelemetryRow(const char *name, const char *header, const char *row)
		{
			const char *d = pifChildTelemetryDir();
			if (!d)
				return;
			static char path[PATH_MAX];
			snprintf(path, sizeof(path), "%s/%s", d, name);
			FILE *f = fopen(path, "a");
			if (!f)
				return;
			if (fseek(f, 0, SEEK_END) == 0 && ftell(f) == 0)
				fprintf(f, "%s\n", header);
			fprintf(f, "%s\n", row);
			fclose(f);
		}
	}

	// ---------------------------------------------------------------------------
	// Path-independent result-type contract helpers.
	// The merge path must never be chosen from the -S value (a file path in the
	// project flow) or from any directory/filename substring. The primary
	// classifier is the actual child network; the script CONTENT (read from the
	// referenced file when readable) is used only for the fallback intent.
	// ---------------------------------------------------------------------------

	// Primary classifier: 1 if the loaded child network is a mapped result.
	// A network is mapped when it carries Mio mapping data (ABC_FUNC_MAP) or
	// when it contains LUT-width gates (nodes with more than 3 fanins, i.e.
	// K4/K6 LUT truth tables read back from BLIF). AIG/SOP results from
	// AIG-mode scripts have only 1-3-input gates and take the AIG form.
	// This is a property of the actual child network, never of the -S value
	// or any path string.
	static bool IsMappedLogic(Abc_Ntk_t *pLogic)
	{
		if (!pLogic)
			return false;
		if (Abc_NtkHasMapping(pLogic))
			return true;
		Abc_Obj_t *pObj;
		int i;
		Abc_NtkForEachNode(pLogic, pObj, i)
			if (Abc_ObjFaninNum(pObj) > 3)
				return true;
		return false;
	}

	// Read the -S value as content: file content when the value names a
	// readable file, otherwise the inline value itself. Never the path string.
	static std::string OptScriptContent(const std::string &value)
	{
		FILE *f = fopen(value.c_str(), "r");
		if (!f)
			return value;
		std::string content;
		char buf[4096];
		size_t n;
		while ((n = fread(buf, 1, sizeof(buf), f)) > 0)
			content.append(buf, n);
		fclose(f);
		return content;
	}

	// Content-based mapping intent: a 'map' or 'if' command token in the script
	// content decides mapped intent; a dch-only script decides AIG intent.
	static bool ScriptHasMappingCommand(const std::string &content)
	{
		size_t pos = 0;
		while (pos <= content.size())
		{
			size_t semi = content.find(';', pos);
			std::string cmd = content.substr(pos, semi == std::string::npos ? std::string::npos : semi - pos);
			size_t b = cmd.find_first_not_of(" \t\r\n");
			if (b != std::string::npos)
			{
				size_t e = cmd.find_first_of(" \t", b);
				std::string tok = cmd.substr(b, e == std::string::npos ? std::string::npos : e - b);
				if (tok == "map" || tok == "if")
					return true;
			}
			if (semi == std::string::npos)
				break;
			pos = semi + 1;
		}
		return false;
	}

	// Flow mapping intent: explicit -m fpga/asic, else the read script content.
	static bool FlowIntendsMapped(const std::string &mapType, const std::string &optScript)
	{
		if (mapType == "fpga" || mapType == "asic")
			return true;
		return ScriptHasMappingCommand(OptScriptContent(optScript));
	}

	// Normalize a stored child to the AIG form: a STRASH network that the AIG
	// merge (Abc_NtkMerge) can process. Takes ownership of pNtk on success.
	static Abc_Ntk_t *ToAigForm(Abc_Ntk_t *pNtk)
	{
		if (!pNtk)
			return nullptr;
		if (Abc_NtkIsStrash(pNtk))
			return pNtk;
		Abc_Ntk_t *pStrash = Abc_NtkStrash(pNtk, 0, 1, 0);
		if (pStrash)
		{
			Abc_NtkDelete(pNtk);
			return pStrash;
		}
		return pNtk;
	}

	// Compact SHA-256 (FIPS 180-4) for effective child-script identification.
	static inline uint32_t Rotr(uint32_t x, int n) { return (x >> n) | (x << (32 - n)); }
	static std::string Sha256Hex(const std::string &data)
	{
		static const uint32_t K[64] = {
			0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
			0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
			0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
			0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
			0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
			0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
			0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
			0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2};
		uint32_t h[8] = {0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19};
		uint64_t len = data.size();
		std::string msg = data;
		msg += (char)0x80;
		while (msg.size() % 64 != 56)
			msg += (char)0;
		for (int i = 7; i >= 0; i--)
			msg += (char)((len * 8) >> (i * 8));
		for (size_t off = 0; off < msg.size(); off += 64)
		{
			uint32_t w[64];
			for (int i = 0; i < 16; i++)
				w[i] = ((uint32_t)(unsigned char)msg[off+i*4] << 24) |
					   ((uint32_t)(unsigned char)msg[off+i*4+1] << 16) |
					   ((uint32_t)(unsigned char)msg[off+i*4+2] << 8) |
					   (uint32_t)(unsigned char)msg[off+i*4+3];
			for (int i = 16; i < 64; i++)
			{
				uint32_t s0 = Rotr(w[i-15], 7) ^ Rotr(w[i-15], 18) ^ (w[i-15] >> 3);
				uint32_t s1 = Rotr(w[i-2], 17) ^ Rotr(w[i-2], 19) ^ (w[i-2] >> 10);
				w[i] = w[i-16] + s0 + w[i-7] + s1;
			}
			uint32_t a=h[0],b=h[1],c=h[2],d=h[3],e=h[4],f=h[5],g=h[6],hh=h[7];
			for (int i = 0; i < 64; i++)
			{
				uint32_t S1 = Rotr(e,6) ^ Rotr(e,11) ^ Rotr(e,25);
				uint32_t ch = (e & f) ^ (~e & g);
				uint32_t t1 = hh + S1 + ch + K[i] + w[i];
				uint32_t S0 = Rotr(a,2) ^ Rotr(a,13) ^ Rotr(a,22);
				uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
				uint32_t t2 = S0 + maj;
				hh=g; g=f; f=e; e=d+t1; d=c; c=b; b=a; a=t1+t2;
			}
			h[0]+=a; h[1]+=b; h[2]+=c; h[3]+=d; h[4]+=e; h[5]+=f; h[6]+=g; h[7]+=hh;
		}
		char hex[65];
		for (int i = 0; i < 8; i++)
			snprintf(hex + i*8, 9, "%08x", h[i]);
		return std::string(hex, 64);
	}

	PartNtk::~PartNtk()
	{
		// m_pMappedNtk 已返回给 ABC 框架, 由 ABC 管理生命周期
		// 不要在这里 delete 它

		// Abc_NtkMergeMapped 是深拷贝, 所以 delete 子网是安全的
		for (auto pNtk : m_vSubNtks)
		{
			if (pNtk)
			{
				Abc_NtkCleanMarkA(pNtk);
				Abc_NtkDelete(pNtk);
			}
		}
		for (auto pNtk : m_vSubNtksMapped)
		{
			if (pNtk)
			{
				Abc_NtkCleanMarkA(pNtk);
				Abc_NtkDelete(pNtk);
			}
		}
		for (auto pNtk : m_vSubNtksOptimized)
		{
			if (pNtk)
			{
				Abc_NtkCleanMarkA(pNtk);
				Abc_NtkDelete(pNtk);
			}
		}
	}

	void PartNtk::init()
	{
		// === 推断 abc 可执行文件路径 ===
		if (const char *env = getenv("PIF_ABC_BIN"))
			m_abcBin = env;
		else
		{
			char selfPath[PATH_MAX];
			ssize_t len = readlink("/proc/self/exe", selfPath, sizeof(selfPath) - 1);
			if (len > 0)
			{
				selfPath[len] = '\0';
				m_abcBin = selfPath;
			}
			else
			{
				m_abcBin = "abc"; // fallback: 依赖 PATH
			}
		}

		// === 推断 abc.rc 路径 ===
		if (const char *env = getenv("PIF_ABC_RC"))
			m_abcRc = env;
		else
		{
			// abc.rc 通常在可执行文件同目录或上级目录
			char binCopy[PATH_MAX];
			strncpy(binCopy, m_abcBin.c_str(), sizeof(binCopy) - 1);
			binCopy[sizeof(binCopy) - 1] = '\0';
			std::string binDir = dirname(binCopy);

			// 优先检查同目录
			std::string rcCandidate = binDir + "/abc.rc";
			if (access(rcCandidate.c_str(), R_OK) == 0)
				m_abcRc = rcCandidate;
			else
			{
				// 尝试上级目录
				strncpy(binCopy, binDir.c_str(), sizeof(binCopy) - 1);
				rcCandidate = std::string(dirname(binCopy)) + "/abc.rc";
				if (access(rcCandidate.c_str(), R_OK) == 0)
					m_abcRc = rcCandidate;
				else
					m_abcRc = "abc.rc"; // fallback
			}
		}

		// === 默认优化脚本 ===
		if (m_optScript.empty())
		{
			if (m_mapType == "asic")
				m_optScript = "strash; dc2; fraig; resyn2; map";
			else
				m_optScript = "strash; dc2; fraig; resyn2; if -K 6 -C 8";
		}

		// === -S script resolution: file mode vs inline mode ===
		// The project passes the per-child script through a file path. When
		// the value names a readable regular file, load its exact content as
		// the child command text; otherwise the value is inline command text
		// embedded verbatim. File-mode failures (unreadable or empty) are
		// rejected through the existing pipeline-failure contract.
		if (!m_optScript.empty())
		{
			struct stat st;
			if (stat(m_optScript.c_str(), &st) == 0 && S_ISREG(st.st_mode))
			{
				FILE *scriptFile = fopen(m_optScript.c_str(), "r");
				if (!scriptFile)
				{
					ylog("[Error] -S file mode: cannot open %s\n", m_optScript.c_str());
					m_fPipelineFailed = true;
				}
				else
				{
					std::string scriptText;
					char buffer[4096];
					size_t n;
					while ((n = fread(buffer, 1, sizeof(buffer), scriptFile)) > 0)
						scriptText.append(buffer, n);
					fclose(scriptFile);
					if (scriptText.empty())
					{
						ylog("[Error] -S file mode: script file %s is empty\n", m_optScript.c_str());
						m_fPipelineFailed = true;
					}
					else
					{
						ylog("[PIF-SCRIPT] -S file mode: %s\n", m_optScript.c_str());
						m_optScript = scriptText;
						ylog("[PIF-SCRIPT] effective child script sha256: %s\n", Sha256Hex(m_optScript).c_str());
					}
				}
			}
			else
			{
				ylog("[PIF-SCRIPT] -S inline mode\n");
				ylog("[PIF-SCRIPT] effective child script sha256: %s\n", Sha256Hex(m_optScript).c_str());
			}
		}

		// === ASIC 映射校验 ===
		if (m_mapType == "asic" && m_libPath.empty())
		{
			// 尝试从 ABC 全局框架获取已加载的库路径
			Abc_Frame_t *pAbc = Abc_FrameGetGlobalFrame();
			if (pAbc && Abc_FrameReadLibGen())
			{
				ylog("ASIC mode: using already loaded genlib from ABC framework.\n");
			}
			else
			{
				ylog("[Error] ASIC mapping requires a standard cell library.\n");
				ylog("        Use -L to specify library path (.lib or .genlib), or load it before calling pif.\n");
			}
		}

		if (m_tmpDir != "/dev/shm")
			mkdir(m_tmpDir.c_str(), 0755);

		ylog("ABC binary: %s\n", m_abcBin.c_str());
		ylog("ABC RC:     %s\n", m_abcRc.c_str());
		ylog("Opt script: %s\n", m_optScript.c_str());
		ylog("Map type:   %s\n", m_mapType.c_str());
		if (m_nMaxConcurrent > 0)
			ylog("Concurrency cap: %d\n", m_nMaxConcurrent);
		else
			ylog("Concurrency cap: default (hw/2)\n");
		ylog("Task tmp dir:    %s\n", m_tmpDir.c_str());
		ylog("Strict mode:     %s\n", m_fStrict ? "on" : "off");
		if (!m_libPath.empty())
			ylog("Lib path:   %s\n", m_libPath.c_str());

		if (m_nParts > 0)
		{
			m_vSubNtks.resize(m_nParts);
			m_vSubNtksMapped.resize(m_nParts);
		}
	}

	void PartNtk::runPipeline()
	{
		auto start_flow = Clock::now();

		// Phase 1: Partitioning
		partOriginNtk();
		tagSubNtkPorts();

		// Phase 2: Parallel Optimization + Mapping (Process-based)
		optimizeSubNtks();

		if (m_fPipelineFailed)
		{
			m_pMappedNtk = NULL;
			auto end_flow = Clock::now();
			m_stats.timeTotal = std::chrono::duration<double>(end_flow - start_flow).count();
			printTimeStats();
			return;
		}

		// Phase 3: Align & Merge
		alignInterfaces();

		if (!m_vSubNtksOptimized.empty())
			mergeAndOutput();
		else
			ylog("[Warn] Optimization results are empty, skipping Output.\n");

		auto end_flow = Clock::now();
		m_stats.timeTotal = std::chrono::duration<double>(end_flow - start_flow).count();

		printTimeStats();
	}

	void PartNtk::partOriginNtk()
	{
		auto start = Clock::now();

		ylog("Starting partitioning routine. User requested nParts = %d (0 means adaptive)\n", m_nParts);
		MetisGraph graph(m_pOriginNtk, 0);
		graph.set_sCluster(m_sCluster);

		MetisAig aig;
		aig.bindGraph(&graph);

		if (m_useMffc)
		{
			aig.parseAigMffc(m_nParts);
			m_nParts = aig.partitionAigMffc();
		}
		else
		{
			aig.parseAig(m_nParts);
			m_nParts = aig.partitionAig();
		}

		ylog(" -> Adaptive nParts = %d\n", m_nParts);
		init();
		graph.createSubNtksFromPartition(m_vSubNtks);

		// Task 17 Stage 2: emitted child PI/PO/interface census
		// (behavior-neutral telemetry; gated by PIF_TELEMETRY_DIR).
		if (pifTelemetryDir())
		{
			Abc_Obj_t *pObj;
			int k;
			for (size_t ci = 0; ci < m_vSubNtks.size(); ci++)
			{
				Abc_Ntk_t *pNtk = m_vSubNtks[ci];
				int nPi = 0, nPo = 0, nIfPi = 0, nIfPo = 0;
				Abc_NtkForEachPi(pNtk, pObj, k)
				{
					nPi++;
					if (pObj->fMarkA)
						nIfPi++;
				}
				Abc_NtkForEachPo(pNtk, pObj, k)
				{
					nPo++;
					if (pObj->fMarkA)
						nIfPo++;
				}
				char buf[256];
				snprintf(buf, sizeof(buf), "%zu\t%d\t%d\t%d\t%d\t%d",
						 ci, Abc_NtkNodeNum(pNtk), nPi, nPo, nIfPi, nIfPo);
				pifTelemetryRow("pif_subntk.tsv",
								"childIdx\tnInternalNodes\tnPi\tnPo\tnInterfacePi\tnInterfacePo",
								buf);
			}
		}

		// Telemetry: per-child predicted workload in child index order.
		m_vSubNtkPredWorkload = aig.getPartitionWorkloads();
		if (m_vSubNtkPredWorkload.size() != m_vSubNtks.size())
			m_vSubNtkPredWorkload.assign(m_vSubNtks.size(), -1);

		auto end = Clock::now();
		m_stats.timePartition = std::chrono::duration<double>(end - start).count();
	}

	void PartNtk::tagSubNtkPorts()
	{
		auto start = Clock::now();

		ylog("Tagging pseudo-ports with anchors for persistence...\n");

		int i, k;
		Abc_Ntk_t *pNtk;
		Abc_Obj_t *pObj;

		// 遍历所有子网
		for (i = 0; i < m_vSubNtks.size(); i++)
		{
			pNtk = m_vSubNtks[i];

			// ============================================================
			// 1. 重置命名管理器 (保持之前的修复逻辑)
			// ============================================================
			std::vector<std::pair<int, std::string>> realPortNames;
			Abc_NtkForEachPi(pNtk, pObj, k)
			{
				if (pObj->fMarkA == 0)
				{
					char *name = Abc_ObjName(pObj);
					if (name)
						realPortNames.push_back({Abc_ObjId(pObj), std::string(name)});
				}
			}
			Abc_NtkForEachPo(pNtk, pObj, k)
			{
				if (pObj->fMarkA == 0)
				{
					char *name = Abc_ObjName(pObj);
					if (name)
						realPortNames.push_back({Abc_ObjId(pObj), std::string(name)});
				}
			}

			if (pNtk->pManName)
			{
				Nm_ManFree(pNtk->pManName);
				pNtk->pManName = NULL;
			}
			pNtk->pManName = Nm_ManCreate(Abc_NtkObjNumMax(pNtk));

			for (auto &pair : realPortNames)
			{
				Abc_Obj_t *pNode = Abc_NtkObj(pNtk, pair.first);
				Abc_ObjAssignName(pNode, (char *)pair.second.c_str(), NULL);
			}

			// ============================================================
			// 2. 给 Pseudo-PO (驱动源) 命名
			// ============================================================
			Abc_NtkForEachPo(pNtk, pObj, k)
			{
				if (pObj->fMarkA == 1)
				{
					char buf[100];
					// 格式: lc_<SubId>_<ObjId>
					snprintf(buf, sizeof(buf), "lc_%d_%d", i, Abc_ObjId(pObj));
					Abc_ObjAssignName(pObj, buf, NULL);
				}
			}
		}

		// ============================================================
		// 3. 给 Pseudo-PI 命名并去重 (关键修复步骤)
		// ============================================================
		// 注意：这里需要分开遍历，因为我们要跨子网去查找 PO 的名字，
		// 必须等所有子网的 PO 都命名完了再处理 PI。

		for (i = 0; i < m_vSubNtks.size(); i++)
		{
			pNtk = m_vSubNtks[i];

			// Map: Source_PO_Ptr -> First_PPI_in_this_SubNtk
			// 用于检测同一个子网内是否有多个 PI 指向同一个源
			std::map<Abc_Obj_t *, Abc_Obj_t *> sourceToPiMap;
			std::vector<Abc_Obj_t *> duplicatePis; // 待删除列表

			Abc_NtkForEachPi(pNtk, pObj, k)
			{
				if (pObj->fMarkA == 1)
				{
					Abc_Obj_t *pSourcePo = (Abc_Obj_t *)pObj->pData;
					if (!pSourcePo)
					{
						// 容错: 如果没有源指针，忽略
						continue;
					}

					// 检查本子网是否已经有一个 PI 连到这个源了
					if (sourceToPiMap.find(pSourcePo) == sourceToPiMap.end())
					{
						// === Case A: 第一次遇到连接该源的 PI ===

						// 1. 获取源 PO 的名字
						char *pName = Abc_ObjName(pSourcePo);

						// 2. 命名当前 PI
						Abc_ObjAssignName(pObj, pName, NULL);

						// 3. 记录下来
						sourceToPiMap[pSourcePo] = pObj;
					}
					else
					{
						// === Case B: 发现重复！(Repeated CI names 的根源) ===
						// 意味着 pObj 和 existingPi 其实是同一根信号线

						Abc_Obj_t *existingPi = sourceToPiMap[pSourcePo];

						// 1. 将 pObj 的所有扇出转移给 existingPi
						Abc_ObjTransferFanout(pObj, existingPi);

						// 2. 标记 pObj 待删除 (不能在遍历时直接删)
						duplicatePis.push_back(pObj);
					}
				}
			}

			// 执行删除操作
			for (Abc_Obj_t *pDup : duplicatePis)
			{
				Abc_NtkDeleteObj(pDup);
			}
		}

		auto end = Clock::now();
		m_stats.timeTagPorts = std::chrono::duration<double>(end - start).count();
	}

	void PartNtk::optimizeSubNtks()
	{
		if (m_vSubNtks.empty())
			return;

		// Resolve library read command based on file extension
		const char *readLibCmd = "read_genlib";
		if (!m_libPath.empty())
		{
			size_t len = m_libPath.size();
			if (len >= 4 && m_libPath.compare(len - 4, 4, ".lib") == 0)
				readLibCmd = "read_lib";
		}

		if (!m_libPath.empty() && !Abc_FrameReadLibGen())
		{
			ylog("Loading library for parent process: %s\n", m_libPath.c_str());
			char cmdBuf[512];
			snprintf(cmdBuf, sizeof(cmdBuf), "%s %s", readLibCmd, m_libPath.c_str());
			Cmd_CommandExecute(Abc_FrameGetGlobalFrame(), cmdBuf);
		}

		ylog("Optimizing sub-networks (Index-based Restoration)...\n");
		auto start_total = Clock::now();

		size_t nTasks = m_vSubNtks.size();
		m_vSubNtksOptimized.clear();
		m_vSubNtksOptimized.resize(nTasks, nullptr);
		m_stats.timeSubNtksOpt.resize(nTasks);

		int max_concurrent = m_nMaxConcurrent > 0 ? m_nMaxConcurrent
												  : (std::thread::hardware_concurrency() / 2);
		if (max_concurrent < 1)
			max_concurrent = 1;

		int running_procs = 0;
		pid_t my_pid = getpid();
		std::unordered_map<pid_t, int> pid_to_idx;
		std::unordered_map<pid_t, long> pid_to_parent_rss;
		std::unordered_map<int, std::chrono::time_point<Clock>> task_start_times;

		auto reap_process = [&](pid_t pid, int status, const struct rusage &ru)
		{
			if (pid_to_idx.count(pid))
			{
				int idx = pid_to_idx[pid];
				auto end = Clock::now();
				m_stats.timeSubNtksOpt[idx] = std::chrono::duration<double>(end - task_start_times[idx]).count();

				bool fChildOk = WIFEXITED(status) && WEXITSTATUS(status) == 0;
				int nExitStatus = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
				int nSignal = WIFSIGNALED(status) ? WTERMSIG(status) : 0;
				bool fReadOk = false;
				if (fChildOk)
				{
					char file_out[256];
					snprintf(file_out, sizeof(file_out), "%s/pif_sub_%d_pid%d_opt.blif", m_tmpDir.c_str(), idx, my_pid);

					Abc_Ntk_t *pNetlist = Io_ReadBlif(file_out, 1);
					fReadOk = (pNetlist != NULL);
					if (pNetlist)
					{
						// 分类依据是实际子网网络, 与 -S 值/路径/文件名无关
						bool isMappedResult = false;
						Abc_Ntk_t *pLogic = Abc_NtkToLogic(pNetlist);
						isMappedResult = IsMappedLogic(pLogic);
						Abc_NtkDelete(pNetlist);

						if (pLogic)
						{
							if (isMappedResult)
							{
								// 映射后的网络：保持 logic 形式，不做 strash
								m_vSubNtksOptimized[idx] = pLogic;
							}
							else
							{
								// 纯优化后的网络：转为 AIG
								m_vSubNtksOptimized[idx] = Abc_NtkStrash(pLogic, 0, 1, 0);
								Abc_NtkDelete(pLogic);
							}
						}
					}
					// 清理临时文件
					remove(file_out);
					char file_in[256];
					snprintf(file_in, sizeof(file_in), "%s/pif_sub_%d_pid%d.blif", m_tmpDir.c_str(), idx, my_pid);
					remove(file_in);
				}
				double dElapsed = m_stats.timeSubNtksOpt[idx];
				int nNodes = Abc_NtkNodeNum(m_vSubNtks[idx]);
				m_dChildElapsedSum += dElapsed;
				if (dElapsed > m_dChildElapsedMax)
					m_dChildElapsedMax = dElapsed;

				// Telemetry: per-child row (elapsed, CPU, RSS, outcome,
				// before/after nodes and levels, predicted workload).
				{
					double cpuSec = (double)ru.ru_utime.tv_sec + (double)ru.ru_utime.tv_usec / 1e6 +
									(double)ru.ru_stime.tv_sec + (double)ru.ru_stime.tv_usec / 1e6;
					Abc_Ntk_t *pRes = m_vSubNtksOptimized[idx];
					const char *resType = "failure";
					if (pRes)
						resType = Abc_NtkIsStrash(pRes) ? "aig" : "mapped";
					if (pRes && !fChildOk)
						resType = "fallback";
					int64_t predWl = (idx < (int)m_vSubNtkPredWorkload.size()) ? m_vSubNtkPredWorkload[idx] : -1;
					auto itRss = pid_to_parent_rss.find(pid);
					long parentRssKB = (itRss != pid_to_parent_rss.end()) ? itRss->second : -1;
					char buf[512];
					snprintf(buf, sizeof(buf),
							 "%d\t%lld\t%d\t%.6f\t%.6f\t%ld\t%ld\t%d\t%d\t%d\t%d\t%s\t%d\t%d\t%d",
							 idx, (long long)predWl, nNodes, dElapsed, cpuSec,
							 ru.ru_maxrss, parentRssKB, nExitStatus, nSignal, (int)fReadOk,
							 (int)(!m_vSubNtksOptimized[idx]), resType,
							 pRes ? Abc_NtkNodeNum(pRes) : 0,
							 pRes ? Abc_NtkLevel(pRes) : 0,
							 pRes ? Abc_NtkGetChoiceNum(pRes) : 0);
					pifChildTelemetryRow("pif_child.tsv",
										 "idx\tpredWorkload\tnNodesIn\telapsedS\tcpuS\trssKB\tparentRssKB\texit\tsig\tread\tfailed\tresultType\tnNodesOut\tlevOut\tchoicesOut",
										 buf);
				}

				if (!m_vSubNtksOptimized[idx])
				{
					if (m_fStrict)
					{
						m_nChildFailure++;
						m_fPipelineFailed = true;
						ylog("[PIF-CHILD] idx=%d nodes=%d pid=%d start=%.4f end=%.4f elapsed=%.4f exit=%d sig=%d read=%d fallback=%d reason=strict_child_error\n",
							 idx, nNodes, pid,
							 std::chrono::duration<double>(task_start_times[idx] - start_total).count(),
							 std::chrono::duration<double>(end - start_total).count(),
							 dElapsed, nExitStatus, nSignal, (int)fReadOk, (int)false);
						ylog("[PIF-STRICT] sub %d has no valid optimized result; failing pif per -e.\n", idx);
						pid_to_idx.erase(pid);
						return;
					}
					ylog("[PIF-CHILD] idx=%d nodes=%d pid=%d start=%.4f end=%.4f elapsed=%.4f exit=%d sig=%d read=%d fallback=%d reason=child_failed\n",
						 idx, nNodes, pid,
						 std::chrono::duration<double>(task_start_times[idx] - start_total).count(),
						 std::chrono::duration<double>(end - start_total).count(),
						 dElapsed, nExitStatus, nSignal, (int)fReadOk, (int)true);
					printf("[Warn] Optimization failed/aborted for sub %d, using original.\n", idx);
					Abc_Ntk_t *pDup = Abc_NtkDup(m_vSubNtks[idx]);

					if (Abc_NtkIsStrash(pDup))
					{
						Abc_Ntk_t *pLogic = Abc_NtkToLogic(pDup);
						Abc_NtkDelete(pDup);
						pDup = pLogic;
					}

					// 映射模式下: 在父进程中执行映射, 保证产出 ABC_FUNC_MAP
					// 意图判断: 显式 -m, 否则读取脚本内容(文件或内联)判断
					bool isMappedFallback = FlowIntendsMapped(m_mapType, m_optScript);

					if (isMappedFallback && pDup && !Abc_NtkHasMapping(pDup))
					{
						// 需要做映射, 通过 fork+exec 避免破坏父进程的 ABC 框架
						fflush(stdout);
						int orig_stdout_fb = dup(STDOUT_FILENO);
						int dev_null_fb = open("/dev/null", O_WRONLY);
						if (dev_null_fb != -1)
							dup2(dev_null_fb, STDOUT_FILENO);

						Abc_Ntk_t *pNetlist = Abc_NtkToNetlist(Abc_NtkDup(pDup));
						if (pNetlist)
						{
							char tmpIn[256], tmpOut[256];
							snprintf(tmpIn, sizeof(tmpIn), "%s/pif_fb_%d_pid%d.blif", m_tmpDir.c_str(), idx, getpid());
							snprintf(tmpOut, sizeof(tmpOut), "%s/pif_fb_%d_pid%d_opt.blif", m_tmpDir.c_str(), idx, getpid());

							Io_WriteBlif(pNetlist, tmpIn, 1, 0, 0);
							Abc_NtkDelete(pNetlist);

							fflush(stdout);
							if (orig_stdout_fb != -1)
							{
								dup2(orig_stdout_fb, STDOUT_FILENO);
								close(orig_stdout_fb);
							}
							if (dev_null_fb != -1)
								close(dev_null_fb);

							char cmd[2048];
							if (!m_libPath.empty())
							{
								if (m_mapType == "asic" || ScriptHasMappingCommand(OptScriptContent(m_optScript)))
									snprintf(cmd, sizeof(cmd),
											 "source %s; %s %s; read_blif %s; strash; map; write_blif %s",
											 m_abcRc.c_str(), readLibCmd, m_libPath.c_str(), tmpIn, tmpOut);
								else
									snprintf(cmd, sizeof(cmd),
											 "source %s; %s %s; read_blif %s; strash; if -K 6 -C 8; write_blif %s",
											 m_abcRc.c_str(), readLibCmd, m_libPath.c_str(), tmpIn, tmpOut);
							}
							else
							{
								if (m_mapType == "asic" || ScriptHasMappingCommand(OptScriptContent(m_optScript)))
									snprintf(cmd, sizeof(cmd),
											 "source %s; read_blif %s; strash; map; write_blif %s",
											 m_abcRc.c_str(), tmpIn, tmpOut);
								else
									snprintf(cmd, sizeof(cmd),
											 "source %s; read_blif %s; strash; if -K 6 -C 8; write_blif %s",
											 m_abcRc.c_str(), tmpIn, tmpOut);
							}

							pid_t cpid = fork();
							if (cpid == 0)
							{
								int devNull = open("/dev/null", O_WRONLY);
								dup2(devNull, 1);
								dup2(devNull, 2);
								close(devNull);
								execlp(m_abcBin.c_str(), "abc", "-c", cmd, nullptr);
								_exit(1);
							}
							else if (cpid > 0)
							{
								int st;
								waitpid(cpid, &st, 0);
								if (WIFEXITED(st) && WEXITSTATUS(st) == 0)
								{
									Abc_Ntk_t *pRB = Io_ReadBlif(tmpOut, 1);
									if (pRB)
									{
										Abc_Ntk_t *pL = Abc_NtkToLogic(pRB);
										Abc_NtkDelete(pRB);
										if (pL && Abc_NtkHasMapping(pL))
										{
											Abc_NtkDelete(pDup);
											m_vSubNtksOptimized[idx] = pL;
											pDup = NULL; // 已处理
										}
										else
										{
											if (pL)
												Abc_NtkDelete(pL);
										}
									}
								}
							}
							remove(tmpIn);
							remove(tmpOut);
						}
						// 如果上面没成功, pDup 仍然有效
						if (!m_vSubNtksOptimized[idx])
						{
							// 非映射意图: 存 STRASH, 保证 AIG 合并安全
							m_vSubNtksOptimized[idx] = isMappedFallback ? pDup : ToAigForm(pDup);
						}
					}
					else
					{
						m_vSubNtksOptimized[idx] = isMappedFallback ? pDup : ToAigForm(pDup);
					}
					m_nChildFallback++;
				}
				else
				{
					m_nChildOk++;
					ylog("[PIF-CHILD] idx=%d nodes=%d pid=%d start=%.4f end=%.4f elapsed=%.4f exit=%d sig=%d read=%d fallback=%d reason=ok\n",
						 idx, nNodes, pid,
						 std::chrono::duration<double>(task_start_times[idx] - start_total).count(),
						 std::chrono::duration<double>(end - start_total).count(),
						 dElapsed, nExitStatus, nSignal, (int)fReadOk, (int)false);
				}
				pid_to_idx.erase(pid);
			}
		};

		// 意图判断: 显式 -m, 否则读取脚本内容(文件或内联)判断
		bool isMappedFlow = FlowIntendsMapped(m_mapType, m_optScript);

		for (int i = 0; i < nTasks; ++i)
		{
			// === 空子网: 无论什么模式都直接跳过 ===
			if (Abc_NtkNodeNum(m_vSubNtks[i]) == 0)
			{
				// 空子网: 非映射意图存 STRASH, 映射意图存 logic
				Abc_Ntk_t *pDup = Abc_NtkDup(m_vSubNtks[i]);
				if (!isMappedFlow)
					m_vSubNtksOptimized[i] = ToAigForm(pDup);
				else if (Abc_NtkIsStrash(pDup))
				{
					Abc_Ntk_t *pLogic = Abc_NtkToLogic(pDup);
					Abc_NtkDelete(pDup);
					m_vSubNtksOptimized[i] = pLogic;
				}
				else
				{
					m_vSubNtksOptimized[i] = pDup;
				}
				m_stats.timeSubNtksOpt[i] = 0.0;
				continue;
			}

			// === 小子网 (节点数 < 64) ===
			if (Abc_NtkNodeNum(m_vSubNtks[i]) < 64)
			{
				if (!isMappedFlow)
				{
					// 非映射模式: 保持 STRASH, AIG 合并需要 STRASH 子网
					Abc_Ntk_t *pDup = Abc_NtkDup(m_vSubNtks[i]);
					m_vSubNtksOptimized[i] = ToAigForm(pDup);
				}
				else
				{
					// 映射模式: 在父进程内直接做映射 (避免 fork 子进程对极小网络失败)
					// 通过文件往返来避免破坏 ABC 框架中的当前网络
					Abc_Ntk_t *pDup = Abc_NtkDup(m_vSubNtks[i]);
					if (Abc_NtkIsStrash(pDup))
					{
						Abc_Ntk_t *pLogic = Abc_NtkToLogic(pDup);
						Abc_NtkDelete(pDup);
						pDup = pLogic;
					}

					char tmpIn[256], tmpOut[256];
					snprintf(tmpIn, sizeof(tmpIn), "%s/pif_small_%d_pid%d.blif", m_tmpDir.c_str(), i, getpid());
					snprintf(tmpOut, sizeof(tmpOut), "%s/pif_small_%d_pid%d_opt.blif", m_tmpDir.c_str(), i, getpid());

					// 写出临时 blif (重定向 stdout 抑制 ABC 日志)
					fflush(stdout);
					int orig_stdout_s = dup(STDOUT_FILENO);
					int dev_null_s = open("/dev/null", O_WRONLY);
					if (dev_null_s != -1)
						dup2(dev_null_s, STDOUT_FILENO);

					Abc_Ntk_t *pNetlist = Abc_NtkToNetlist(pDup);
					if (pNetlist)
					{
						Io_WriteBlif(pNetlist, tmpIn, 1, 0, 0);
						Abc_NtkDelete(pNetlist);
					}

					fflush(stdout);
					if (orig_stdout_s != -1)
					{
						dup2(orig_stdout_s, STDOUT_FILENO);
						close(orig_stdout_s);
					}
					if (dev_null_s != -1)
						close(dev_null_s);
					Abc_NtkDelete(pDup);

					// 构造命令: 读取, 映射, 写出
					char cmd[2048];
					if (!m_libPath.empty())
					{
						if (m_mapType == "asic" || ScriptHasMappingCommand(OptScriptContent(m_optScript)))
							snprintf(cmd, sizeof(cmd),
									 "source %s; %s %s; read_blif %s; strash; map; write_blif %s",
									 m_abcRc.c_str(), readLibCmd, m_libPath.c_str(), tmpIn, tmpOut);
						else
							snprintf(cmd, sizeof(cmd),
									 "source %s; %s %s; read_blif %s; strash; if -K 6 -C 8; write_blif %s",
									 m_abcRc.c_str(), readLibCmd, m_libPath.c_str(), tmpIn, tmpOut);
					}
					else
					{
						if (m_mapType == "asic" || ScriptHasMappingCommand(OptScriptContent(m_optScript)))
							snprintf(cmd, sizeof(cmd),
									 "source %s; read_blif %s; strash; map; write_blif %s",
									 m_abcRc.c_str(), tmpIn, tmpOut);
						else
							snprintf(cmd, sizeof(cmd),
									 "source %s; read_blif %s; strash; if -K 6 -C 8; write_blif %s",
									 m_abcRc.c_str(), tmpIn, tmpOut);
					}

					// 用 fork+exec 执行, 避免污染父进程的 ABC 框架
					pid_t cpid = fork();
					if (cpid == 0)
					{
						int devNull = open("/dev/null", O_WRONLY);
						dup2(devNull, 1);
						dup2(devNull, 2);
						close(devNull);
						execlp(m_abcBin.c_str(), "abc", "-c", cmd, nullptr);
						_exit(1);
					}
					else if (cpid > 0)
					{
						int status;
						waitpid(cpid, &status, 0);

						if (WIFEXITED(status) && WEXITSTATUS(status) == 0)
						{
							Abc_Ntk_t *pReadBack = Io_ReadBlif(tmpOut, 1);
							if (pReadBack)
							{
								Abc_Ntk_t *pLogic = Abc_NtkToLogic(pReadBack);
								Abc_NtkDelete(pReadBack);
								if (pLogic && Abc_NtkHasMapping(pLogic))
								{
									m_vSubNtksOptimized[i] = pLogic;
								}
								else
								{
									if (pLogic)
										Abc_NtkDelete(pLogic);
									// [FIX] 映射失败: 尝试父进程重映射
									Abc_Ntk_t *pFB = Abc_NtkDup(m_vSubNtks[i]);
									if (Abc_NtkIsStrash(pFB))
									{
										Abc_Ntk_t *pL = Abc_NtkToLogic(pFB);
										Abc_NtkDelete(pFB);
										pFB = pL;
									}
									{
										char tmpIn2[256], tmpOut2[256];
										snprintf(tmpIn2, sizeof(tmpIn2), "%s/pif_small_fb_%d_pid%d.blif", m_tmpDir.c_str(), i, getpid());
										snprintf(tmpOut2, sizeof(tmpOut2), "%s/pif_small_fb_%d_pid%d_opt.blif", m_tmpDir.c_str(), i, getpid());
										fflush(stdout);
										int orig_stdout_s2 = dup(STDOUT_FILENO);
										int dev_null_s2 = open("/dev/null", O_WRONLY);
										if (dev_null_s2 != -1)
											dup2(dev_null_s2, STDOUT_FILENO);
										Abc_Ntk_t *pNetlist2 = Abc_NtkToNetlist(Abc_NtkDup(pFB));
										if (pNetlist2)
										{
											Io_WriteBlif(pNetlist2, tmpIn2, 1, 0, 0);
											Abc_NtkDelete(pNetlist2);
										}
										fflush(stdout);
										if (orig_stdout_s2 != -1)
										{
											dup2(orig_stdout_s2, STDOUT_FILENO);
											close(orig_stdout_s2);
										}
										if (dev_null_s2 != -1)
											close(dev_null_s2);
										char cmd2[2048];
										if (!m_libPath.empty())
										{
											if (m_mapType == "asic" || ScriptHasMappingCommand(OptScriptContent(m_optScript)))
												snprintf(cmd2, sizeof(cmd2),
														 "source %s; %s %s; read_blif %s; strash; map; write_blif %s",
														 m_abcRc.c_str(), readLibCmd, m_libPath.c_str(), tmpIn2, tmpOut2);
											else
												snprintf(cmd2, sizeof(cmd2),
														 "source %s; %s %s; read_blif %s; strash; if -K 6 -C 8; write_blif %s",
														 m_abcRc.c_str(), readLibCmd, m_libPath.c_str(), tmpIn2, tmpOut2);
										}
										else
										{
											if (m_mapType == "asic" || ScriptHasMappingCommand(OptScriptContent(m_optScript)))
												snprintf(cmd2, sizeof(cmd2),
														 "source %s; read_blif %s; strash; map; write_blif %s",
														 m_abcRc.c_str(), tmpIn2, tmpOut2);
											else
												snprintf(cmd2, sizeof(cmd2),
														 "source %s; read_blif %s; strash; if -K 6 -C 8; write_blif %s",
														 m_abcRc.c_str(), tmpIn2, tmpOut2);
										}
										pid_t cpid2 = fork();
										if (cpid2 == 0)
										{
											int devNull2 = open("/dev/null", O_WRONLY);
											dup2(devNull2, 1);
											dup2(devNull2, 2);
											close(devNull2);
											execlp(m_abcBin.c_str(), "abc", "-c", cmd2, nullptr);
											_exit(1);
										}
										else if (cpid2 > 0)
										{
											int st2;
											waitpid(cpid2, &st2, 0);
											if (WIFEXITED(st2) && WEXITSTATUS(st2) == 0)
											{
												Abc_Ntk_t *pRB2 = Io_ReadBlif(tmpOut2, 1);
												if (pRB2)
												{
													Abc_Ntk_t *pL2 = Abc_NtkToLogic(pRB2);
													Abc_NtkDelete(pRB2);
													if (pL2 && Abc_NtkHasMapping(pL2))
													{
														Abc_NtkDelete(pFB);
														m_vSubNtksOptimized[i] = pL2;
														pFB = NULL;
													}
													else if (pL2)
														Abc_NtkDelete(pL2);
												}
											}
										}
										remove(tmpIn2);
										remove(tmpOut2);
									}
									if (!m_vSubNtksOptimized[i])
									{
										if (m_fStrict)
										{
											m_nChildFailure++;
											m_fPipelineFailed = true;
											ylog("[PIF-STRICT] small sub %d mapping produced a non-mapped result; failing pif per -e.\n", i);
											if (pFB)
												Abc_NtkDelete(pFB);
										}
										else
										{
											m_vSubNtksOptimized[i] = pFB;
											ylog("[Warn] Small sub %d mapping produced non-mapped result\n", i);
										}
									}
								}
							}
						}
						else
						{
							Abc_Ntk_t *pFB = Abc_NtkDup(m_vSubNtks[i]);
							if (Abc_NtkIsStrash(pFB))
							{
								Abc_Ntk_t *pL = Abc_NtkToLogic(pFB);
								Abc_NtkDelete(pFB);
								pFB = pL;
							}
							{
								char tmpIn2[256], tmpOut2[256];
								snprintf(tmpIn2, sizeof(tmpIn2), "%s/pif_small_fb_%d_pid%d.blif", m_tmpDir.c_str(), i, getpid());
								snprintf(tmpOut2, sizeof(tmpOut2), "%s/pif_small_fb_%d_pid%d_opt.blif", m_tmpDir.c_str(), i, getpid());
								fflush(stdout);
								int orig_stdout_s2 = dup(STDOUT_FILENO);
								int dev_null_s2 = open("/dev/null", O_WRONLY);
								if (dev_null_s2 != -1)
									dup2(dev_null_s2, STDOUT_FILENO);
								Abc_Ntk_t *pNetlist2 = Abc_NtkToNetlist(Abc_NtkDup(pFB));
								if (pNetlist2)
								{
									Io_WriteBlif(pNetlist2, tmpIn2, 1, 0, 0);
									Abc_NtkDelete(pNetlist2);
								}
								fflush(stdout);
								if (orig_stdout_s2 != -1)
								{
									dup2(orig_stdout_s2, STDOUT_FILENO);
									close(orig_stdout_s2);
								}
								if (dev_null_s2 != -1)
									close(dev_null_s2);
								char cmd2[2048];
								if (!m_libPath.empty())
								{
									if (m_mapType == "asic" || ScriptHasMappingCommand(OptScriptContent(m_optScript)))
										snprintf(cmd2, sizeof(cmd2),
												 "source %s; %s %s; read_blif %s; strash; map; write_blif %s",
												 m_abcRc.c_str(), readLibCmd, m_libPath.c_str(), tmpIn2, tmpOut2);
									else
										snprintf(cmd2, sizeof(cmd2),
												 "source %s; %s %s; read_blif %s; strash; if -K 6 -C 8; write_blif %s",
												 m_abcRc.c_str(), readLibCmd, m_libPath.c_str(), tmpIn2, tmpOut2);
								}
								else
								{
									if (m_mapType == "asic" || ScriptHasMappingCommand(OptScriptContent(m_optScript)))
										snprintf(cmd2, sizeof(cmd2),
												 "source %s; read_blif %s; strash; map; write_blif %s",
												 m_abcRc.c_str(), tmpIn2, tmpOut2);
									else
										snprintf(cmd2, sizeof(cmd2),
												 "source %s; read_blif %s; strash; if -K 6 -C 8; write_blif %s",
												 m_abcRc.c_str(), tmpIn2, tmpOut2);
								}
								pid_t cpid2 = fork();
								if (cpid2 == 0)
								{
									int devNull2 = open("/dev/null", O_WRONLY);
									dup2(devNull2, 1);
									dup2(devNull2, 2);
									close(devNull2);
									execlp(m_abcBin.c_str(), "abc", "-c", cmd2, nullptr);
									_exit(1);
								}
								else if (cpid2 > 0)
								{
									int st2;
									waitpid(cpid2, &st2, 0);
									if (WIFEXITED(st2) && WEXITSTATUS(st2) == 0)
									{
										Abc_Ntk_t *pRB2 = Io_ReadBlif(tmpOut2, 1);
										if (pRB2)
										{
											Abc_Ntk_t *pL2 = Abc_NtkToLogic(pRB2);
											Abc_NtkDelete(pRB2);
											if (pL2 && Abc_NtkHasMapping(pL2))
											{
												Abc_NtkDelete(pFB);
												m_vSubNtksOptimized[i] = pL2;
												pFB = NULL;
											}
											else if (pL2)
												Abc_NtkDelete(pL2);
										}
									}
								}
								remove(tmpIn2);
								remove(tmpOut2);
							}
							if (!m_vSubNtksOptimized[i])
							{
								if (m_fStrict)
								{
									m_nChildFailure++;
									m_fPipelineFailed = true;
									ylog("[PIF-STRICT] small sub %d fork mapping failed; failing pif per -e.\n", i);
									if (pFB)
										Abc_NtkDelete(pFB);
								}
								else
								{
									m_vSubNtksOptimized[i] = pFB;
									ylog("[Warn] Small sub %d fork mapping failed\n", i);
								}
							}
						}
					}
					remove(tmpIn);
					remove(tmpOut);
				}
				m_stats.timeSubNtksOpt[i] = 0.0;
				continue;
			}
			// 大子网: 走 fork 子进程
			if (m_fPipelineFailed)
				break;

			while (running_procs >= max_concurrent)
			{
				int status;
				struct rusage ru;
				pid_t pid = wait4(-1, &status, 0, &ru);
				if (pid > 0)
				{
					reap_process(pid, status, ru);
					running_procs--;
				}
			}

			task_start_times[i] = Clock::now();
			char file_in[256], file_out[256];
			snprintf(file_in, sizeof(file_in), "%s/pif_sub_%d_pid%d.blif", m_tmpDir.c_str(), i, my_pid);
			snprintf(file_out, sizeof(file_out), "%s/pif_sub_%d_pid%d_opt.blif", m_tmpDir.c_str(), i, my_pid);
			remove(file_in);

			// 重定向 stdout 到 /dev/null，抑制子进程产生的 ABC 日志
			fflush(stdout);
			int original_stdout = dup(STDOUT_FILENO);
			int dev_null = open("/dev/null", O_WRONLY);
			if (dev_null != -1)
				dup2(dev_null, STDOUT_FILENO);

			Abc_Ntk_t *pNtkNetlist = Abc_NtkToNetlist(m_vSubNtks[i]);
			if (pNtkNetlist)
			{
				if (!Abc_NtkIsNetlist(pNtkNetlist))
				{
					Abc_Ntk_t *tmp = Abc_NtkToNetlist(pNtkNetlist);
					Abc_NtkDelete(pNtkNetlist);
					pNtkNetlist = tmp;
				}
				Io_WriteBlif(pNtkNetlist, file_in, 1, 0, 0);
				Abc_NtkDelete(pNtkNetlist);
			}

			// 恢复 stdout
			fflush(stdout);
			if (original_stdout != -1)
			{
				dup2(original_stdout, STDOUT_FILENO);
				close(original_stdout);
			}
			if (dev_null != -1)
				close(dev_null);

			struct rusage rufork;
			getrusage(RUSAGE_SELF, &rufork);
			pid_t pid = fork();
			if (pid == 0)
			{
				int devNull = open("/dev/null", O_WRONLY);
				dup2(devNull, 1);
				dup2(devNull, 2);
				close(devNull);
				char cmd[2048];

				if (!m_libPath.empty())
				{
					// ASIC 模式：子进程需要先加载标准单元库
					snprintf(cmd, sizeof(cmd),
							 "source %s; %s %s; read_blif %s; %s; write_blif %s",
							 m_abcRc.c_str(), readLibCmd, m_libPath.c_str(), file_in,
							 m_optScript.c_str(), file_out);
				}
				else
				{
					snprintf(cmd, sizeof(cmd),
							 "source %s; read_blif %s; %s; write_blif %s",
							 m_abcRc.c_str(), file_in, m_optScript.c_str(), file_out);
				}

				execlp(m_abcBin.c_str(), "abc", "-c", cmd, nullptr);
				_exit(1);
			}
			else if (pid > 0)
			{
				pid_to_idx[pid] = i;
				pid_to_parent_rss[pid] = rufork.ru_maxrss;
				running_procs++;
			}
		}

		while (running_procs > 0)
		{
			int status;
			struct rusage ru;
			pid_t pid = wait4(-1, &status, 0, &ru);
			if (pid > 0)
			{
				reap_process(pid, status, ru);
				running_procs--;
			}
		}

		auto end_total = Clock::now();
		m_stats.timeOptTotal = std::chrono::duration<double>(end_total - start_total).count();
		if (m_fPipelineFailed)
			ylog("[PIF-STRICT] pif pipeline failed: ok=%d fallback=%d failure=%d\n",
				 m_nChildOk, m_nChildFallback, m_nChildFailure);
	}

	void PartNtk::alignInterfaces()
	{
		vector<Abc_Ntk_t *> &targetNtks = m_vSubNtksOptimized;
		if (targetNtks.empty())
			return;

		auto start = Clock::now();

		ylog("Aligning interfaces via anchor names...\n");
		std::unordered_map<std::string, Abc_Obj_t *> globalAnchorMap;
		int linkCount = 0;

		// Pass 1: 注册锚点 (Pseudo-PO)
		for (auto pNtk : targetNtks)
		{
			if (!pNtk)
				continue;
			Abc_Obj_t *pObj;
			int k;
			Abc_NtkForEachPo(pNtk, pObj, k)
			{
				std::string name = Abc_ObjName(pObj);
				if (name.find("lc_") == 0)
				{
					pObj->fMarkA = 1;
					globalAnchorMap[name] = pObj;
				}
				else
				{
					pObj->fMarkA = 0;
				}
			}
		}

		// Pass 2: 连接负载 (Pseudo-PI)
		for (auto pNtk : targetNtks)
		{
			if (!pNtk)
				continue;
			Abc_Obj_t *pObj;
			int k;
			Abc_NtkForEachPi(pNtk, pObj, k)
			{
				std::string name = Abc_ObjName(pObj);
				if (name.find("lc_") == 0)
				{
					pObj->fMarkA = 1;
					if (globalAnchorMap.count(name))
					{
						pObj->pData = globalAnchorMap[name];
						linkCount++;
					}
					else
					{
						ylog("[Error] Dangling Cut PI: %s\n", name.c_str());
					}
				}
				else
				{
					pObj->fMarkA = 0;
				}
			}
		}
		ylog("  -> Restored %d cross-boundary connections.\n", linkCount);

		auto end = Clock::now();
		m_stats.timeAlign = std::chrono::duration<double>(end - start).count();
	}

	void PartNtk::mergeAndOutput()
	{
		// 合并分派依据实际子网结果类型, 与 -S 值/路径无关
		vector<Abc_Ntk_t *> &targetNtks = m_vSubNtksOptimized;

		if (targetNtks.empty())
			return;

		// 实际子网形式决定合并函数: 全部 STRASH 走 AIG 合并;
		// 含 logic(映射/SOP) 子网走 mapped 合并 (mapped 合并能处理
		// Mio 与 SOP 子网, 且锚点指向子网对象, 不能在合并前释放/替换)。
		// 混合类型仅可能出现在映射意图的回退子网, 默认策略用 mapped 合并,
		// strict 模式失败。
		bool fAnyLogic = false, fAnyStrash = false;
		for (auto pNtk : targetNtks)
		{
			if (!pNtk)
				continue;
			if (Abc_NtkIsStrash(pNtk))
				fAnyStrash = true;
			else
				fAnyLogic = true;
		}
		if (fAnyLogic && fAnyStrash && m_fStrict)
		{
			ylog("[PIF-STRICT] mixed mapped/AIG child results; failing pif per -e.\n");
			m_nChildFailure++;
			m_fPipelineFailed = true;
			return;
		}
		if (fAnyLogic && fAnyStrash)
		{
			// 混合子网: mapped 合并无法安全处理 strash 子网 (节点 pData 为 NULL,
			// Abc_NtkDupObj 的 Hop_Transfer 会解引用无效指针)。
			// 默认策略: 全部转为 STRASH 走 AIG 合并 (函数保持, v3c 下游有 strash)。
			// 转换替换了锚点指向的子网对象, 必须重新对齐接口。
			ylog("[Warn] mixed mapped/AIG child results; converting all children to AIG for merge.\n");
			for (auto &pNtk : targetNtks)
			{
				if (pNtk && !Abc_NtkIsStrash(pNtk))
					pNtk = ToAigForm(pNtk);
			}
			alignInterfaces();
			fAnyLogic = false;
		}
		bool isMapped = fAnyLogic;

		ylog("Merging %s subnetworks (Structure-based)...\n", isMapped ? "mapped" : "optimized");

		Vec_Ptr_t *vSubNtksRef = Vec_PtrAlloc(m_vSubNtks.size());
		for (auto pNtk : m_vSubNtks)
			Vec_PtrPush(vSubNtksRef, pNtk);

		Vec_Ptr_t *vSubNtksMerge = Vec_PtrAlloc(targetNtks.size());
		for (auto pNtk : targetNtks)
			Vec_PtrPush(vSubNtksMerge, pNtk);

		Abc_Ntk_t *pMergedNtk = NULL;
		auto start = Clock::now();

		if (isMapped)
			pMergedNtk = Abc_NtkMergeMapped(m_pOriginNtk, vSubNtksRef, vSubNtksMerge);
		else
			pMergedNtk = Abc_NtkMerge(m_pOriginNtk, vSubNtksRef, vSubNtksMerge);

		Vec_PtrFree(vSubNtksRef);
		Vec_PtrFree(vSubNtksMerge);

		if (pMergedNtk == NULL)
		{
			ylog("[Error] Merge failed (Result is NULL).\n");
			return;
		}

		auto end = Clock::now();
		m_stats.timeMerge = std::chrono::duration<double>(end - start).count();

		// === BLIF round-trip: 让 pMergedNtk 拥有独立的 pManFunc ===
		// 用 BLIF 写+读代替 Abc_NtkDup，避免因个别节点 pData 无效而 SIGSEGV
		if (pMergedNtk && Abc_NtkIsLogic(pMergedNtk))
		{
			Abc_Ntk_t *pNetlist = Abc_NtkToNetlist(pMergedNtk);
			if (pNetlist)
			{
				char tmpFile[256];
				snprintf(tmpFile, sizeof(tmpFile), "%s/pif_merged_tmp.blif", m_dirName);
				fflush(stdout);
				int saved_out = dup(STDOUT_FILENO);
				int dev_null = open("/dev/null", O_WRONLY);
				if (dev_null != -1)
					dup2(dev_null, STDOUT_FILENO);
				Io_WriteBlif(pNetlist, tmpFile, 1, 0, 0);
				fflush(stdout);
				if (saved_out != -1)
				{
					dup2(saved_out, STDOUT_FILENO);
					close(saved_out);
				}
				if (dev_null != -1)
					close(dev_null);
				Abc_NtkDelete(pNetlist);
				Abc_Ntk_t *pClean = Io_ReadBlif(tmpFile, 1);
				remove(tmpFile);
				if (pClean)
				{
					if (Abc_NtkIsNetlist(pClean))
					{
						Abc_Ntk_t *pLogic = Abc_NtkToLogic(pClean);
						if (pLogic)
						{
							Abc_NtkDelete(pClean);
							pClean = pLogic;
						}
					}
					if (pClean)
					{
						Abc_NtkDelete(pMergedNtk);
						pMergedNtk = pClean;
					}
				}
				else
				{
					ylog("[Warn] Merged BLIF sanitize failed, keeping original.\n");
				}
			}
			else
			{
				ylog("[Warn] Abc_NtkToNetlist on merged network failed.\n");
			}
		}

		// === 设置名字 ===
		if (pMergedNtk)
		{
			if (pMergedNtk->pName)
				ABC_FREE(pMergedNtk->pName);
			if (m_pOriginNtk && Abc_NtkName(m_pOriginNtk))
				pMergedNtk->pName = Abc_UtilStrsav(Abc_NtkName(m_pOriginNtk));
			else
				pMergedNtk->pName = Abc_UtilStrsav("pif_result");
		}

		// === 写文件 ===
		char filename[256];
		// 写形式跟随合并后网络的实际类型: STRASH -> AIG 文件, logic -> 映射文件
		bool isMappedOut = Abc_NtkIsLogic(pMergedNtk);
		if (isMappedOut)
		{
			bool isAsic = (m_mapType == "asic");

			if (isAsic)
			{
				// ASIC 映射：写 Verilog
				snprintf(filename, sizeof(filename), "%s/merged_mapped.v", m_dirName);
				ylog("Writing merged Verilog: %s\n", filename);

				Abc_Ntk_t *pNetlist = Abc_NtkToNetlist(pMergedNtk);
				if (pNetlist)
				{
					if (m_pOriginNtk && Abc_NtkName(m_pOriginNtk))
						pNetlist->pName = Abc_UtilStrsav(Abc_NtkName(m_pOriginNtk));
					Io_WriteVerilog(pNetlist, filename, 0, 0);
					Abc_NtkDelete(pNetlist);
				}
				else
				{
					ylog("[Warn] Netlist conversion failed.\n");
				}
			}
			else
			{
				// FPGA(LUT) 映射：写 blif
				snprintf(filename, sizeof(filename), "%s/merged_mapped.blif", m_dirName);
				ylog("Writing merged mapped BLIF: %s\n", filename);

				Abc_Ntk_t *pNetlist = Abc_NtkToNetlist(pMergedNtk);
				if (pNetlist)
				{
					if (m_pOriginNtk && Abc_NtkName(m_pOriginNtk))
						pNetlist->pName = Abc_UtilStrsav(Abc_NtkName(m_pOriginNtk));
					Io_WriteBlif(pNetlist, filename, 1, 0, 0);
					Abc_NtkDelete(pNetlist);
				}
				else
				{
					ylog("[Warn] Netlist conversion failed.\n");
				}
			}
		}
		else
		{
			// 纯优化（无映射）：写 AIG
			snprintf(filename, sizeof(filename), "%s/merged.aig", m_dirName);
			ylog("Writing merged AIG: %s\n", filename);
			if (!Abc_NtkIsStrash(pMergedNtk))
			{
				Abc_Ntk_t *pTemp = Abc_NtkStrash(pMergedNtk, 0, 1, 0);
				Abc_NtkDelete(pMergedNtk);
				pMergedNtk = pTemp;
			}
			Io_WriteAiger(pMergedNtk, filename, 1, 0, 0);
		}

		m_pMappedNtk = pMergedNtk;
	}

	void PartNtk::normalizeSkeletonInterfaces()
	{
		if (!m_pOriginNtk)
			return;

		ylog("Assigning persistent names to skeleton interfaces...\n");

		if (m_pOriginNtk->pManName == NULL)
			m_pOriginNtk->pManName = Nm_ManCreate(Abc_NtkObjNumMax(m_pOriginNtk));

		Abc_Obj_t *pObj;
		int k;

		Abc_NtkForEachPi(m_pOriginNtk, pObj, k)
		{
			char *pName = Abc_ObjName(pObj);
			if (pName == NULL || pName[0] == 'n')
			{
				char buf[100];
				snprintf(buf, sizeof(buf), "sys_pi_%d", Abc_ObjId(pObj));
				Abc_ObjAssignName(pObj, buf, NULL);
			}
		}

		Abc_NtkForEachPo(m_pOriginNtk, pObj, k)
		{
			char *pName = Abc_ObjName(pObj);
			if (pName == NULL || pName[0] == 'n')
			{
				char buf[100];
				snprintf(buf, sizeof(buf), "sys_po_%d", Abc_ObjId(pObj));
				Abc_ObjAssignName(pObj, buf, NULL);
			}
		}
	}

	void PartNtk::printTimeStats()
	{
		double total = m_stats.timeTotal;
		if (total <= 1e-9)
			total = 1.0;

		auto fmt_pct = [total](double t)
		{
			return (t / total) * 100.0;
		};

		auto calc_balance = [](double avg, double max)
		{
			return (max > 1e-9) ? (avg / max) * 100.0 : 0.0;
		};

		printf("\n==========================================================\n");
		printf("               PIF Real-time Performance                   \n");
		printf("==========================================================\n");

		const char *fmt_head = "%-32s %8.4f s   [%5.1f%%]\n";
		const char *fmt_sub = "    - %-26s %8.4f s   [%5.1f%%]\n";

		// [1] Partitioning Phase
		double phase1_total = m_stats.timePartition + m_stats.timeTagPorts;
		printf(fmt_head, "[1] Partitioning Total:", phase1_total, fmt_pct(phase1_total));
		printf(fmt_sub, "Partition:", m_stats.timePartition, fmt_pct(m_stats.timePartition));
		printf(fmt_sub, "Tag SubNtk Ports:", m_stats.timeTagPorts, fmt_pct(m_stats.timeTagPorts));
		printf("----------------------------------------------------------\n");

		// [2] Parallel Opt + Map (Multi-process)
		printf(fmt_head, "[2] Parallel Opt+Map:", m_stats.timeOptTotal, fmt_pct(m_stats.timeOptTotal));
		if (!m_stats.timeSubNtksOpt.empty())
		{
			double sum = std::accumulate(m_stats.timeSubNtksOpt.begin(), m_stats.timeSubNtksOpt.end(), 0.0);
			double max = *std::max_element(m_stats.timeSubNtksOpt.begin(), m_stats.timeSubNtksOpt.end());
			double avg = sum / m_stats.timeSubNtksOpt.size();

			printf("      * Processes:                %lu\n", m_stats.timeSubNtksOpt.size());
			printf("      * Workload Stats (s):       Max=%.4f, Avg=%.4f, Sum=%.4f\n", max, avg, sum);
			printf("      * Load Balance:             %.1f%% (Avg/Max)\n", calc_balance(avg, max));
		}
		printf("      * Child Outcomes:            ok=%d fallback=%d failure=%d\n",
			   m_nChildOk, m_nChildFallback, m_nChildFailure);
		printf("      * Child Elapsed (s):         Max=%.4f, Sum=%.4f\n",
			   m_dChildElapsedMax, m_dChildElapsedSum);
		{
			double wall = m_stats.timeOptTotal > 1e-9 ? m_stats.timeOptTotal : 1.0;
			printf("      * Effective Concurrency:    %.2f (sum_elapsed/phase_wall)\n",
				   m_dChildElapsedSum / wall);
		}
		printf("      * Script: %s\n", m_optScript.c_str());
		printf("----------------------------------------------------------\n");

		// [3] Align + Merge
		double phase3_total = m_stats.timeAlign + m_stats.timeMerge;
		printf(fmt_head, "[3] Align + Merge:", phase3_total, fmt_pct(phase3_total));
		printf(fmt_sub, "Align Interfaces:", m_stats.timeAlign, fmt_pct(m_stats.timeAlign));
		printf(fmt_sub, "Merge:", m_stats.timeMerge, fmt_pct(m_stats.timeMerge));

		printf("==========================================================\n");
		printf("TOTAL RUNTIME:                      %8.4f s\n", m_stats.timeTotal);
		printf("==========================================================\n");
	}

	void PartNtk::Abc_NtkWriteBlif()
	{
		std::cout << "Now in write blif!" << endl;
		const char *outputFolder = m_dirName;

		int i = 0;
		for (auto pNtk : m_vSubNtks)
		{
			char filename[256];
			snprintf(filename, sizeof(filename), "%s/network_%d.blif", outputFolder, i);
			printf("Writing file: %s\n", filename);

			Abc_Ntk_t *pNtkNew = Abc_NtkToNetlist(pNtk);
			Io_WriteBlif(pNtkNew, filename, 1, 0, 0);
			i++;
		}
	}

	void PartNtk::Abc_NtkWriteAIG()
	{
		std::cout << "Now in write AIG!" << endl;
		const char *outputFolder = m_dirName;

		int i = 0;
		for (auto pNtk : m_vSubNtks)
		{
			char filename[256];
			snprintf(filename, sizeof(filename), "%s/network_%d.aig", outputFolder, i);
			printf("Writing file: %s\n", filename);

			Abc_NtkSetName(pNtk, const_cast<char *>(("network_" + to_string(i)).c_str()));
			Io_WriteAiger(pNtk, filename, 1, 0, 0);
			i++;
		}
	}

	void PartNtk::Abc_NtkWriteVerilog()
	{
		std::cout << "Now in write verilog!" << endl;
		const char *outputFolder = m_dirName;

		int i = 0;
		for (auto pNtk : m_vSubNtks)
		{
			char filename[256];
			snprintf(filename, sizeof(filename), "%s/network_%d.v", outputFolder, i);
			printf("Writing file: %s\n", filename);

			Abc_Ntk_t *pNtkNew = Abc_NtkToNetlist(pNtk);
			Abc_NtkSetName(pNtkNew, const_cast<char *>(("network_" + to_string(i)).c_str()));

			if (!Abc_NtkHasAig(pNtkNew) && !Abc_NtkHasMapping(pNtkNew))
				Abc_NtkToAig(pNtkNew);
			Io_WriteVerilog(pNtkNew, filename, 0, 0);
			i++;
		}
	}

	void PartNtk::Abc_NtkWriteMappedBlif()
	{
		std::cout << "Now in write mapped blif!" << endl;
		const char *outputFolder = m_dirName;

		int i = 0;
		for (auto pNtk : m_vSubNtksMapped)
		{
			char filename[256];
			snprintf(filename, sizeof(filename), "%s/Mappednetwork_%d.blif", outputFolder, i);
			printf("Writing file: %s\n", filename);

			Abc_Ntk_t *pNtkNew = Abc_NtkToNetlist(pNtk);
			Io_WriteBlif(pNtkNew, filename, 1, 0, 0);
			i++;
		}
	}

} // for namespace
