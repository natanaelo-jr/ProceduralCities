#include "cuda_kernels.cuh"
#include "cuda_runtime.h"
#include <iostream>

__global__ void hello_kernel() {
  int id = threadIdx.x + blockIdx.x * blockDim.x;
  printf("Hello from GPU thread %d\n", id);
}

void hello() {
  hello_kernel<<<1, 5>>>();
  cudaDeviceSynchronize();
}
