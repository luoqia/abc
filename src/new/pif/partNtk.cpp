#include "partNtk.h"
#include <sys/wait.h>
#include <unistd.h>
#include <sys/types.h>
#include <unordered_map>
#include <fcntl.h>
#include <cstdio>
#include <linux/limits.h> // PATH_MAX
#include <libgen.h>		  // dirname

using Clock = std::chrono::high_resolution_clock;

namespace ymc
{

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
		// Yosys passes the per-subnetwork ABC command sequence through a script
		// file because semicolons cannot be embedded safely in its ABC argument
		// encoding. Accept both the documented inline form and a readable file.
		if (!m_optScript.empty() && access(m_optScript.c_str(), R_OK) == 0)
		{
			FILE *scriptFile = fopen(m_optScript.c_str(), "r");
			if (scriptFile)
			{
				std::string scriptText;
				char buffer[1024];
				while (fgets(buffer, sizeof(buffer), scriptFile))
					scriptText += buffer;
				fclose(scriptFile);
				if (!scriptText.empty())
					m_optScript = scriptText;
			}
		}

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

		// === 推断可选的 abc.rc 路径 ===
		// 所有 PIF 子进程命令都使用完整命令名，不应因为缺少 abc.rc 而失败。
		// 源码树构建还需要检查 <bin>/abc/abc.rc，因为 yosys-abc 位于仓库根目录。
		m_abcRc.clear();
		if (const char *env = getenv("PIF_ABC_RC"))
		{
			if (access(env, R_OK) == 0)
				m_abcRc = env;
			else
				ylog("[Warn] PIF_ABC_RC is not readable, continuing without it: %s\n", env);
		}
		else
		{
			char binCopy[PATH_MAX];
			strncpy(binCopy, m_abcBin.c_str(), sizeof(binCopy) - 1);
			binCopy[sizeof(binCopy) - 1] = '\0';
			std::string binDir = dirname(binCopy);

			strncpy(binCopy, binDir.c_str(), sizeof(binCopy) - 1);
			binCopy[sizeof(binCopy) - 1] = '\0';
			std::string parentDir = dirname(binCopy);

			const std::string candidates[] = {
				binDir + "/abc.rc",
				binDir + "/abc/abc.rc",
				parentDir + "/abc.rc"
			};
			for (const auto &candidate : candidates)
				if (access(candidate.c_str(), R_OK) == 0)
				{
					m_abcRc = candidate;
					break;
				}
		}

		// === 默认优化脚本 ===
		if (m_optScript.empty())
		{
			const char *resyn2 = "balance; rewrite; refactor; balance; rewrite; rewrite -z; balance; refactor -z; rewrite -z; balance";
			if (m_mapType == "asic")
				m_optScript = std::string("strash; dc2; fraig; ") + resyn2 + "; map";
			else
				m_optScript = std::string("strash; dc2; fraig; ") + resyn2 + "; if -K 6 -C 8";
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

		ylog("ABC binary: %s\n", m_abcBin.c_str());
		ylog("ABC RC:     %s\n", m_abcRc.empty() ? "(not found; aliases disabled)" : m_abcRc.c_str());
		// The existing child command builder always emits a source command.
		// /dev/null is a valid empty source file and keeps that command well-formed.
		if (m_abcRc.empty())
			m_abcRc = "/dev/null";
		ylog("Opt script: %s\n", m_optScript.c_str());
		ylog("Map type:   %s\n", m_mapType.c_str());
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

		// Phase 3: Align & Merge
		if (!normalizeOptimizedNetworks())
		{
			ylog("[Error] Failed to normalize optimized sub-networks, skipping Output.\n");
			return;
		}
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

		int max_concurrent = std::thread::hardware_concurrency() / 2;
		if (max_concurrent < 1)
			max_concurrent = 1;

		int running_procs = 0;
		pid_t my_pid = getpid();
		std::unordered_map<pid_t, int> pid_to_idx;
		std::unordered_map<int, std::chrono::time_point<Clock>> task_start_times;

		auto reap_process = [&](pid_t pid, int status)
		{
			if (pid_to_idx.count(pid))
			{
				int idx = pid_to_idx[pid];
				auto end = Clock::now();
				m_stats.timeSubNtksOpt[idx] = std::chrono::duration<double>(end - task_start_times[idx]).count();

				if (WIFEXITED(status) && WEXITSTATUS(status) == 0)
				{
					char file_out[256];
					snprintf(file_out, sizeof(file_out), "/dev/shm/pif_sub_%d_pid%d_opt.blif", idx, my_pid);

					Abc_Ntk_t *pNetlist = Io_ReadBlif(file_out, 1);
					if (pNetlist)
					{
						// 根据用户脚本判断是否包含映射命令
						bool isMappedResult = false;
						if (m_mapType == "fpga" || m_mapType == "asic")
							isMappedResult = true;
						else if (m_mapType.empty())
							isMappedResult = (m_optScript.find("map") != std::string::npos) ||
											 (m_optScript.find("if ") != std::string::npos) ||
											 (m_optScript.find("if;") != std::string::npos) ||
											 (m_optScript.find("if -") != std::string::npos);

						Abc_Ntk_t *pLogic = Abc_NtkToLogic(pNetlist);
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
					snprintf(file_in, sizeof(file_in), "/dev/shm/pif_sub_%d_pid%d.blif", idx, my_pid);
					remove(file_in);
				}
				if (!m_vSubNtksOptimized[idx])
				{
					printf("[Warn] Optimization failed/aborted for sub %d, using original.\n", idx);
					Abc_Ntk_t *pDup = Abc_NtkDup(m_vSubNtks[idx]);

					if (Abc_NtkIsStrash(pDup))
					{
						Abc_Ntk_t *pLogic = Abc_NtkToLogic(pDup);
						Abc_NtkDelete(pDup);
						pDup = pLogic;
					}

					// 映射模式下: 在父进程中执行映射, 保证产出 ABC_FUNC_MAP
					bool isMappedFallback = false;
					if (m_mapType == "fpga" || m_mapType == "asic")
						isMappedFallback = true;
					else if (m_mapType.empty())
						isMappedFallback = (m_optScript.find("map") != std::string::npos) ||
										   (m_optScript.find("if ") != std::string::npos) ||
										   (m_optScript.find("if;") != std::string::npos) ||
										   (m_optScript.find("if -") != std::string::npos);

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
							snprintf(tmpIn, sizeof(tmpIn), "/dev/shm/pif_fb_%d_pid%d.blif", idx, getpid());
							snprintf(tmpOut, sizeof(tmpOut), "/dev/shm/pif_fb_%d_pid%d_opt.blif", idx, getpid());

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
								if (m_mapType == "asic" ||
									(m_mapType.empty() && m_optScript.find("map") != std::string::npos))
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
								if (m_mapType == "asic" ||
									(m_mapType.empty() && m_optScript.find("map") != std::string::npos))
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
							m_vSubNtksOptimized[idx] = pDup;
						}
					}
					else
					{
						m_vSubNtksOptimized[idx] = pDup;
					}
				}
				pid_to_idx.erase(pid);
			}
		};

		// 判断是否包含映射命令
		bool isMappedFlow = false;
		if (m_mapType == "fpga" || m_mapType == "asic")
			isMappedFlow = true;
		else if (m_mapType.empty())
			isMappedFlow = (m_optScript.find("map") != std::string::npos) ||
						   (m_optScript.find("if ") != std::string::npos) ||
						   (m_optScript.find("if;") != std::string::npos) ||
						   (m_optScript.find("if -") != std::string::npos);

		for (int i = 0; i < nTasks; ++i)
		{
			// === 空子网: 无论什么模式都直接跳过 ===
			if (Abc_NtkNodeNum(m_vSubNtks[i]) == 0)
			{
				// 空子网: 创建一个空的 logic 网络占位
				Abc_Ntk_t *pDup = Abc_NtkDup(m_vSubNtks[i]);
				if (Abc_NtkIsStrash(pDup))
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
					// 非映射模式: 直接转 logic 即可
					Abc_Ntk_t *pDup = Abc_NtkDup(m_vSubNtks[i]);
					if (Abc_NtkIsStrash(pDup))
					{
						Abc_Ntk_t *pLogic = Abc_NtkToLogic(pDup);
						Abc_NtkDelete(pDup);
						m_vSubNtksOptimized[i] = pLogic;
					}
					else
					{
						m_vSubNtksOptimized[i] = pDup;
					}
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
					snprintf(tmpIn, sizeof(tmpIn), "/dev/shm/pif_small_%d_pid%d.blif", i, getpid());
					snprintf(tmpOut, sizeof(tmpOut), "/dev/shm/pif_small_%d_pid%d_opt.blif", i, getpid());

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
						if (m_mapType == "asic" ||
							(m_mapType.empty() && m_optScript.find("map") != std::string::npos))
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
						if (m_mapType == "asic" ||
							(m_mapType.empty() && m_optScript.find("map") != std::string::npos))
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
										snprintf(tmpIn2, sizeof(tmpIn2), "/dev/shm/pif_small_fb_%d_pid%d.blif", i, getpid());
										snprintf(tmpOut2, sizeof(tmpOut2), "/dev/shm/pif_small_fb_%d_pid%d_opt.blif", i, getpid());
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
											if (m_mapType == "asic" ||
												(m_mapType.empty() && m_optScript.find("map") != std::string::npos))
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
											if (m_mapType == "asic" ||
												(m_mapType.empty() && m_optScript.find("map") != std::string::npos))
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
										m_vSubNtksOptimized[i] = pFB;
										ylog("[Warn] Small sub %d mapping produced non-mapped result\n", i);
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
								snprintf(tmpIn2, sizeof(tmpIn2), "/dev/shm/pif_small_fb_%d_pid%d.blif", i, getpid());
								snprintf(tmpOut2, sizeof(tmpOut2), "/dev/shm/pif_small_fb_%d_pid%d_opt.blif", i, getpid());
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
									if (m_mapType == "asic" ||
										(m_mapType.empty() && m_optScript.find("map") != std::string::npos))
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
									if (m_mapType == "asic" ||
										(m_mapType.empty() && m_optScript.find("map") != std::string::npos))
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
								m_vSubNtksOptimized[i] = pFB;
								ylog("[Warn] Small sub %d fork mapping failed\n", i);
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

			while (running_procs >= max_concurrent)
			{
				int status;
				pid_t pid = wait(&status);
				if (pid > 0)
				{
					reap_process(pid, status);
					running_procs--;
				}
			}

			task_start_times[i] = Clock::now();
			char file_in[256], file_out[256];
			snprintf(file_in, sizeof(file_in), "/dev/shm/pif_sub_%d_pid%d.blif", i, my_pid);
			snprintf(file_out, sizeof(file_out), "/dev/shm/pif_sub_%d_pid%d_opt.blif", i, my_pid);
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
				running_procs++;
			}
		}

		while (running_procs > 0)
		{
			int status;
			pid_t pid = wait(&status);
			if (pid > 0)
			{
				reap_process(pid, status);
				running_procs--;
			}
		}

		auto end_total = Clock::now();
		m_stats.timeOptTotal = std::chrono::duration<double>(end_total - start_total).count();
	}

	bool PartNtk::normalizeOptimizedNetworks()
	{
		// Abc_NtkMerge recursively traverses AIG nodes. Child-process failures and
		// BLIF fallbacks can leave a non-mapped result in logic/SOP form, so make
		// the merge contract explicit before interface pointers are attached.
		bool isMapped = false;
		if (m_mapType == "fpga" || m_mapType == "asic")
			isMapped = true;
		else if (m_mapType.empty())
			isMapped = (m_optScript.find("map") != std::string::npos) ||
					   (m_optScript.find("if ") != std::string::npos) ||
					   (m_optScript.find("if;") != std::string::npos) ||
					   (m_optScript.find("if -") != std::string::npos);

		if (isMapped)
			return true;

		for (size_t i = 0; i < m_vSubNtksOptimized.size(); i++)
		{
			Abc_Ntk_t *pNtk = m_vSubNtksOptimized[i];
			if (!pNtk || Abc_NtkIsStrash(pNtk))
				continue;

			Abc_Ntk_t *pStrash = Abc_NtkStrash(pNtk, 0, 1, 0);
			if (!pStrash)
			{
				ylog("[Error] Could not convert optimized sub-network %zu to AIG form.\n", i);
				return false;
			}

			Abc_NtkDelete(pNtk);
			m_vSubNtksOptimized[i] = pStrash;
		}
		return true;
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
		// isMapped 判断（和第2处逻辑完全一样）：
		bool isMapped = false;
		if (m_mapType == "fpga" || m_mapType == "asic")
			isMapped = true;
		else if (m_mapType.empty())
			isMapped = (m_optScript.find("map") != std::string::npos) ||
					   (m_optScript.find("if ") != std::string::npos) ||
					   (m_optScript.find("if;") != std::string::npos) ||
					   (m_optScript.find("if -") != std::string::npos);

		vector<Abc_Ntk_t *> &targetNtks = m_vSubNtksOptimized;

		if (targetNtks.empty())
			return;

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
		if (isMapped)
		{
			bool isAsic = (m_mapType == "asic") ||
						  (m_mapType.empty() && m_optScript.find("map") != std::string::npos);

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
