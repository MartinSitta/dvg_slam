#include "DvgVector.h"
DvgVector_t dvg_vector_normalize(DvgVector_t in){       
    DvgVector_t out;
    double vector_len = sqrt(in.x * in.x + in.y * in.y + in.z * in.z);
    out.x = in.x / vector_len;
    out.y = in.y / vector_len;
    out.z = in.z / vector_len;
    return out;
}