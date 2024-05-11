#pragma once

#ifdef __cplusplus
extern "C" {
#endif

int CompressPointXYZ(float resolution, float* input_vector, int input_size, void* output);

int CompressPointXYZI(float resolution, float* input_vector, int input_size, void* output);

#ifdef __cplusplus
}
#endif

