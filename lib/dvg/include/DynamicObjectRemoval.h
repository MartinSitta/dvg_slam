#ifndef DYNAMICOBJECTREMOVAL_H
#define DYNAMICOBJECTREMOVAL_H

#ifdef __cplusplus
extern "C"{
#endif
#include "Dvg.h"
#include "DvgVector.h"

void dynamic_object_removal(Dvg_t* dvg, int64_t org_x, int64_t org_y, int64_t org_z,
                    int64_t dest_x, int64_t dest_y, int64_t dest_z,
                    bool splash_delete,
                    double max_range_meters,
                    int64_t scalar);
#ifdef __cplusplus
}
#endif

#endif