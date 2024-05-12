#pragma once

#ifdef __cplusplus
extern "C" {
#endif

enum PointType
{
    Undefined = 0,
    POINT_XYZ = 1,
    POINT_XYZI = 2,
    POINT_OUSTER = 3
};

int LossyPointcloudCompression(PointType type, float resolution,
                               unsigned char* input_vector,
                               int input_size,
                               unsigned char* output);


#ifdef __cplusplus
}
#endif

