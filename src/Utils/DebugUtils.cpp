#include "DebugUtils.h"

#include <omp.h>

#include <fstream>  // Necessário para arquivos
#include <iomanip>
#include <iostream>
#include <numeric>
#include <vector>

#include "City/City.h"

struct BenchmarkBaseline {
  double layout = 0.0;
  double buildCPU = 0.0;
  double buildGPU = 0.0;
};

void runBenchmark() {
  // --- CONFIGURAÇÕES ---
  const int NUM_SAMPLES = 5;
  const float MAP_SIZE = 10000.0f;  // 40km x 40km (Gigante para testar a GPU)
  const int SEED = 12345;
  const float BLOCK_SIZE = 60.0f;

  int iterations = (int) (MAP_SIZE / BLOCK_SIZE);

  // --- PREPARAÇÃO DO CSV ---
  std::ofstream csvFile("benchmark_results.csv");
  csvFile << "Threads,Layout_Time_s,Layout_Speedup,Build_CPU_s,Build_CPU_Speedup,Build_GPU_s,Build_"
             "GPU_Speedup_vs_Serial,Total_PureCPU_s,Total_Hybrid_s\n";

  // --- CONSOLE HEADER ---
  std::cout << "\n================================================================================="
               "================="
            << std::endl;
  std::cout << "                      BENCHMARK: CPU (OpenMP) vs GPU (CUDA)" << std::endl;
  std::cout << "==================================================================================="
               "==============="
            << std::endl;
  std::cout << "Hardware Concurrency: " << std::thread::hardware_concurrency() << " cores"
            << std::endl;
  std::cout << "Max OpenMP Threads:   " << omp_get_max_threads() << std::endl;
  std::cout << "Configuracao:         Mapa " << (int) MAP_SIZE << "x" << (int) MAP_SIZE
            << " | Iteracoes: " << iterations << std::endl;
  std::cout << "Exportando para:      benchmark_results.csv" << std::endl;
  std::cout << "-----------------------------------------------------------------------------------"
               "---------------"
            << std::endl;
  std::cout << "| Thr | Layout(s) | L.Spd | B.CPU(s)  | B.Spd | B.GPU(s)  | G.Spd*| T.CPU(s)  | "
               "T.Hybrid(s)|"
            << std::endl;
  std::cout << "|-----|-----------|-------|-----------|-------|-----------|-------|-----------|----"
               "--------|"
            << std::endl;

  // --- WARM-UP (Aquecimento) ---
  // GPUs demoram para inicializar o contexto na primeira chamada.
  // Vamos rodar uma vez sem medir para "acordar" a placa.
  std::cout << "Aquecendo GPU e CPU..." << std::flush;
  {
    City dummy(1000.0f, SEED);
    dummy.generateZones(5);
    dummy.generateCityLayout(10, 60.0f);
    dummy.generateBuildingsCUDA();
  }
  std::cout << " Pronto.\n" << std::endl;

  int maxThreads = omp_get_max_threads();
  BenchmarkBaseline baseline;

  // Loop de Threads da CPU
  for (int t = 1; t <= maxThreads;
       t = (t == maxThreads) ? t + 1 : (t * 2 > maxThreads ? maxThreads : t * 2)) {
    omp_set_num_threads(t);

    std::vector<double> samplesLayout;
    std::vector<double> samplesBuildCPU;
    std::vector<double> samplesBuildGPU;

    samplesLayout.reserve(NUM_SAMPLES);
    samplesBuildCPU.reserve(NUM_SAMPLES);
    samplesBuildGPU.reserve(NUM_SAMPLES);

    for (int i = 0; i < NUM_SAMPLES; ++i) {
      City city(MAP_SIZE, SEED);

      // 1. Zonas (Ignorado no timer detalhado)
      city.generateZones(20);

      // 2. Layout (CPU Parallel)
      double t1 = omp_get_wtime();
      city.generateCityLayout(iterations, BLOCK_SIZE);
      double t2 = omp_get_wtime();

      // 3. Buildings (CPU Parallel)
      city.generateBuildings();
      double t3 = omp_get_wtime();

      // 4. Buildings (GPU CUDA)
      // Nota: Rodamos de novo na mesma instância para ser justo com o estado dos dados
      // O tempo inclui Transferencia (PCIe) + Kernel Execution
      double t4 = omp_get_wtime();
      city.generateBuildingsCUDA();
      double t5 = omp_get_wtime();

      samplesLayout.push_back(t2 - t1);
      samplesBuildCPU.push_back(t3 - t2);
      samplesBuildGPU.push_back(t5 - t4);
    }

    // Médias
    auto calcAvg = [](const std::vector<double>& v) {
      return std::accumulate(v.begin(), v.end(), 0.0) / v.size();
    };

    double avgLayout = calcAvg(samplesLayout);
    double avgBuildCPU = calcAvg(samplesBuildCPU);
    double avgBuildGPU = calcAvg(samplesBuildGPU);

    // Define Baseline (Serial)
    if (t == 1) {
      baseline.layout = avgLayout;
      baseline.buildCPU = avgBuildCPU;
      baseline.buildGPU = avgBuildGPU;  // A GPU não muda com threads de CPU, mas guardamos pra ref
    }

    // Speedups
    double spdLayout = baseline.layout / avgLayout;
    double spdBuildCPU = baseline.buildCPU / avgBuildCPU;

    // O Speedup da GPU é comparado contra a CPU SERIAL (1 Thread)
    // Isso mostra "Quantas vezes a GPU é mais rápida que um núcleo de CPU"
    double spdBuildGPU_vs_Serial = baseline.buildCPU / avgBuildGPU;

    // Tempos Totais (Layout + Build)
    double totalPureCPU = avgLayout + avgBuildCPU;
    double totalHybrid = avgLayout + avgBuildGPU;  // Layout na CPU + Build na GPU

    // --- CONSOLE OUTPUT ---
    std::cout << "| " << std::setw(3) << t
              << " | "
              // Layout
              << std::setw(9) << std::fixed << std::setprecision(4) << avgLayout << " | "
              << std::setw(4) << std::setprecision(1) << spdLayout
              << "x | "
              // CPU Build
              << std::setw(9) << std::setprecision(4) << avgBuildCPU << " | " << std::setw(4)
              << std::setprecision(1) << spdBuildCPU
              << "x | "
              // GPU Build
              << std::setw(9) << std::setprecision(4) << avgBuildGPU << " | " << std::setw(4)
              << std::setprecision(1) << spdBuildGPU_vs_Serial
              << "x | "
              // Totais
              << std::setw(9) << std::setprecision(4) << totalPureCPU << " | " << std::setw(10)
              << std::setprecision(4) << totalHybrid << " |" << std::endl;

    // --- CSV OUTPUT ---
    csvFile << t << "," << avgLayout << "," << spdLayout << "," << avgBuildCPU << "," << spdBuildCPU
            << "," << avgBuildGPU << "," << spdBuildGPU_vs_Serial << "," << totalPureCPU << ","
            << totalHybrid << "\n";

    if (t == maxThreads)
      break;
  }
  std::cout << "-----------------------------------------------------------------------------------"
               "---------------"
            << std::endl;
  std::cout << "* G.Spd = Speedup da GPU comparado a CPU Serial (1 Thread)" << std::endl;
  csvFile.close();
  std::cout << "Dados salvos em 'benchmark_results.csv'" << std::endl;
  std::cout << "\n" << std::endl;
}
