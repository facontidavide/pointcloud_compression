#pragma once

#ifdef __cplusplus
extern "C" {
#endif

enum PointType
{
    Undefined = 0,
    POINT_XYZ = 1,
    POINT_XYZI = 2
};

enum class FieldType
{
    INT8    = 1,
    UINT8   = 2,
    INT16   = 3,
    UINT16  = 4,
    INT32   = 5,
    UINT32  = 6,
    FLOAT32 = 7,
    FLOAT64 = 8
};

struct Field
{
    FieldType type;
    int offset = 0;
    double mult = 1.0;
};

struct FieldVectorView
{
    Field* fields = nullptr;
    unsigned long size = 0;
};

struct BufferView
{
    unsigned char* data = nullptr;
    unsigned long size = 0;
};

int PointcloudCompressionField(FieldVectorView fields,
                               int point_step,
                               BufferView input,
                               BufferView output);

int PointcloudCompression(PointType type, float resolution,
                          BufferView input,
                          BufferView output);


#ifdef __cplusplus
}
#endif

