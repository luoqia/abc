#pragma once

#include "yaig.h"
#include "base/io/ioAbc.h"
#include <chrono>
#include <numeric>
#include "base/abc/abc.h"
#include <sys/time.h>

#include "new/pif/partNtkFuncs.h"

using std::vector;

namespace ymc
{
	struct PifTimeStats
	{
		// Phase 1: Partitioning
		double timePartition = 0.0;
		double timeTagPorts = 0.0;

		// Phase 2: Parallel Optimization + Mapping (multi-process)
		double timeOptTotal = 0.0;
		std::vector<double> timeSubNtksOpt;

		// Phase 3: Align + Merge
		double timeAlign = 0.0;
		double timeMerge = 0.0;

		// Total
		double timeTotal = 0.0;
	};

	// class for parallel mapping implemented by graph partitioning.
	class PartNtk
	{
	public:
		~PartNtk();
		PartNtk(Abc_Ntk_t *pNtkOrigin, uint32_t nParts, uint32_t sCluster,
				char *dirName, const char *optScript = nullptr,
				const char *mapType = nullptr, const char *libPath = nullptr)
			: m_nParts(nParts), m_pOriginNtk(pNtkOrigin), m_sCluster(sCluster)
		{
			strcpy(m_dirName, dirName);
			if (optScript && strlen(optScript) > 0)
				m_optScript = optScript;
			m_mapType = (mapType && strlen(mapType) > 0) ? mapType : "";
			if (libPath && strlen(libPath) > 0)
				m_libPath = libPath;
			init();
		};

		void init();
		void setOriginNtk(Abc_Ntk_t *pNtk)
		{
			m_pOriginNtk = pNtk;
			m_pMappedNtk = NULL;
		}
		Abc_Ntk_t *getResNtk() { return m_pMappedNtk; }
		uint32_t getNParts() { return m_nParts; }

		void runPipeline();
		void normalizeSkeletonInterfaces();
		void partOriginNtk();
		void tagSubNtkPorts();
		void optimizeSubNtks();
		bool normalizeOptimizedNetworks();
		void alignInterfaces();
		void mergeAndOutput();

		void printTimeStats();

		void Abc_NtkWriteBlif();
		void Abc_NtkWriteVerilog();
		void Abc_NtkWriteAIG();
		void Abc_NtkWriteMappedBlif();

	private:
		uint32_t m_nParts;
		Abc_Ntk_t *m_pOriginNtk;
		Abc_Ntk_t *m_pMappedNtk;
		vector<Abc_Ntk_t *> m_vSubNtks;
		vector<Abc_Ntk_t *> m_vSubNtksMapped; // 保留但不再主动使用，后续可删
		char m_dirName[100];
		uint32_t m_sCluster;
		vector<Abc_Ntk_t *> m_vSubNtksOptimized;

		// 新增
		std::string m_optScript; // 用户自定义优化(+映射)命令序列
		std::string m_mapType;	 // "fpga" 或 "asic"（预留）
		std::string m_abcBin;	 // abc 可执行文件路径
		std::string m_abcRc;	 // abc.rc 路径
		std::string m_libPath;	 // 标准单元库路径（ASIC 映射用）
		bool m_useMffc = true;	 // 使用MFFC-based partition (默认开启)

		PifTimeStats m_stats;
	};

} // for namespace
