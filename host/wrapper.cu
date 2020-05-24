//#include <stdio.h>
//#include <chrono>
#include "wrapper.cuh"

//#include "PowerSensor.h"

int compare_arrays(float *c, float *d, int n);

void vec_add(float *c, float *a, float *b, int n) {
    for (int i=0; i<n; i++) {
        c[i] = a[i] + b[i];
    }
}

extern "C" __global__ void vec_add_kernel(float *c, float *a, float *b, int n) {
    //int i = blockIdx.x*threadIdx.x+blockDim.x;   // Oops! Something is not right here, please fix it!
    int i = blockDim.x * blockIdx.x + threadIdx.x;

    if (i < n) {
        c[i] = a[i] + b[i];
    }
}

namespace Wrapper {
int n = 5e7;
cudaError_t err;

float *a = (float *) malloc(n * sizeof(float));
float *b = (float *) malloc(n * sizeof(float));
float *c = (float *) malloc(n * sizeof(float));
float *d = (float *) malloc(n * sizeof(float));
float *d_a; float *d_b; float *d_c;

void setup(void) {
    //printf("%d", n);
    //allocate arrays and fill them
    for (int i=0; i<n; i++) {
        a[i] = 1.0 / rand();
        b[i] = 1.0 / rand();
        c[i] = 0.0;
        d[i] = 0.0;
    }

    //allocate GPU memory
    err = cudaMalloc((void **)&d_a, n*sizeof(float));
    if (err != cudaSuccess) fprintf(stderr, "Error in cudaMalloc d_a: %s\n", cudaGetErrorString( err ));
    err = cudaMalloc((void **)&d_b, n*sizeof(float));
    if (err != cudaSuccess) fprintf(stderr, "Error in cudaMalloc d_b: %s\n", cudaGetErrorString( err ));
    err = cudaMalloc((void **)&d_c, n*sizeof(float));
    if (err != cudaSuccess) fprintf(stderr, "Error in cudaMalloc d_c: %s\n", cudaGetErrorString( err ));
    
    
    //powerSensor.mark("copy");
  
    //copy the input data to the GPU
    err = cudaMemcpy(d_a, a, n*sizeof(float), cudaMemcpyHostToDevice);
    if (err != cudaSuccess) fprintf(stderr, "Error in cudaMemcpy host to device a: %s\n", cudaGetErrorString( err ));
    err = cudaMemcpy(d_b, b, n*sizeof(float), cudaMemcpyHostToDevice);
    if (err != cudaSuccess) fprintf(stderr, "Error in cudaMemcpy host to device b: %s\n", cudaGetErrorString( err ));

    //zero the output array
    err = cudaMemset(d_c, 0, n*sizeof(float));
    if (err != cudaSuccess) fprintf(stderr, "Error in cudaMemset c: %s\n", cudaGetErrorString( err ));
}


void executeKernel(void) {    
    //setup the grid and thread blocks
    int block_size = 1024;                          //thread block size
    int nblocks = int(ceilf(n/(float)block_size));  //problem size divided by thread block size rounded up
    dim3 grid(nblocks, 1);
    dim3 threads(block_size, 1, 1);

    //measure the GPU function
    cudaDeviceSynchronize();
    vec_add_kernel<<<grid, threads>>>(d_c, d_a, d_b, n);
    cudaDeviceSynchronize();

    err = cudaGetLastError();
    if (err != cudaSuccess) fprintf(stderr, "Error during kernel launch vec_add_kernel: %s\n", cudaGetErrorString( err ));
}    
    //powerSensor.mark("done");
void cleanUp(void) {
    //check to see if all went well

    //copy the result back to host memory
    err = cudaMemcpy(d, d_c, n*sizeof(float), cudaMemcpyDeviceToHost);
    if (err != cudaSuccess) fprintf(stderr, "Error in cudaMemcpy device to host c: %s\n", cudaGetErrorString( err ));

    //powerSensor.dump(0);

    //clean up
    cudaFree(d_a);
    cudaFree(d_b);
    cudaFree(d_c);
    free(a);
    free(b);
    free(c);
    free(d);

}
}


int compare_arrays(float *a1, float *a2, int n) {
    int errors = 0;
    int print = 0;

    for (int i=0; i<n; i++) {

        if (isnan(a1[i]) || isnan(a2[i])) {
            errors++;
            if (print < 10) {
                print++;
                fprintf(stderr, "Error NaN detected at i=%d,\t a1= %10.7e \t a2= \t %10.7e\n",i,a1[i],a2[i]);
            }
        }

        unsigned int int_a1 = *(unsigned int *)(a1+i);
        unsigned int int_a2 = *(unsigned int *)(a2+i);
        unsigned int dist = (unsigned int)0;
        if (int_a1 > int_a2) {
            dist = int_a1 - int_a2;
        } else {
            dist = int_a2 - int_a1;
        }
        if (dist > 0) {
            errors++;
            if (print < 10) {
                print++;
                fprintf(stderr, "Error detected at i=%d, \t a1= \t %10.7e \t a2= \t %10.7e \t ulp_dist=\t %u\n",i,a1[i],a2[i],dist);
            }
        }

    }

    return errors;
}
