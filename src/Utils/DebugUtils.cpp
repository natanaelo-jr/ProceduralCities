#include "DebugUtils.h"

#include <omp.h>

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <thread>

// --- BENCHMARK ---
void runBenchmark() {
  // --- CONFIGURAÇÕES DO TESTE ---
  const int NUM_SAMPLES = 10;       // Quantas vezes rodar para tirar a média
  const float MAP_SIZE = 10000.0f;  // Tamanho do mapa (quanto maior, mais trabalho)
  const int SEED = 42;              // Seed fixa para garantir que o trabalho seja IDÊNTICO

  std::cout << "\n=============================================================" << std::endl;
  std::cout << "   BENCHMARK DE PARALELISMO (CityGen - Hybrid Arch)" << std::endl;
  std::cout << "=============================================================" << std::endl;
  std::cout << "CPU Cores (Logicos): " << std::thread::hardware_concurrency() << std::endl;
  std::cout << "Max OpenMP Threads:  " << omp_get_max_threads() << std::endl;
  std::cout << "Amostras por teste:  " << NUM_SAMPLES << std::endl;
  std::cout << "Mapa: " << (int) MAP_SIZE << "x" << (int) MAP_SIZE;
  std::cout << "-------------------------------------------------------------" << std::endl;
  std::cout << "| Threads |  Media (s) | Min (s)  | Speedup | Eficiencia |" << std::endl;
  std::cout << "-------------------------------------------------------------" << std::endl;

  // Descobre o limite da máquina
  int maxThreads = omp_get_max_threads();
  double baseTimeSerial = 0.0;

  // Loop de Threads: 1, 2, 4, 8... até o máximo
  for (int t = 1; t <= maxThreads;
       t = (t == maxThreads) ? t + 1 : (t * 2 > maxThreads ? maxThreads : t * 2)) {
    omp_set_num_threads(t);

    std::vector<double> samples;
    samples.reserve(NUM_SAMPLES);

    // --- LOOP DE AMOSTRAGEM ---
    for (int i = 0; i < NUM_SAMPLES; ++i) {
      // 1. Instancia limpa (Sem lixo de memória anterior)
      City city(MAP_SIZE, SEED);

      // 2. Cronômetro
      double start = omp_get_wtime();

      // 3. Carga de Trabalho
      // Aqui chamamos o pipeline inteiro: Zonas -> Highways -> Subdivisão
      city.generateZones(20);
      city.generateCityLayout((int) MAP_SIZE / 40.0f, 40.0f);

      double end = omp_get_wtime();
      samples.push_back(end - start);
    }

    // --- ESTATÍSTICAS ---
    double sum = std::accumulate(samples.begin(), samples.end(), 0.0);
    double avgTime = sum / NUM_SAMPLES;
    double minTime = *std::min_element(samples.begin(), samples.end());

    // Define o tempo base (Serial) na primeira rodada
    if (t == 1)
      baseTimeSerial = avgTime;

    double speedup = baseTimeSerial / avgTime;
    double efficiency = (speedup / t) * 100.0;

    // --- PRINT FORMATEDO ---
    std::cout << "| " << std::setw(7) << t << " | " << std::setw(9) << std::fixed
              << std::setprecision(4) << avgTime << "s | " << std::setw(7) << std::setprecision(4)
              << minTime << "s | " << std::setw(6) << std::setprecision(2) << speedup << "x | "
              << std::setw(8) << std::setprecision(1) << efficiency << "% |" << std::endl;

    // Hack do loop for para garantir que pare exatamente no maxThreads
    if (t == maxThreads)
      break;
  }
  std::cout << "-------------------------------------------------------------" << std::endl;
  std::cout << "Benchmark Concluido.\n" << std::endl;
}
