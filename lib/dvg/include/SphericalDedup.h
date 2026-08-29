#ifndef SPHERICALDEDUP_H
#define SPHERICALDEDUP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <DvgVector.h>
#include <stdint.h>

/*
 * ---------------------------------------------------------------------------
 * Spherical de-duplication backend selection
 * ---------------------------------------------------------------------------
 *
 * Select exactly one backend at compile time.
 *
 * You can either edit DVG_SPHERICAL_BACKEND below, or override it from CMake:
 *
 *   target_compile_definitions(
 *       dvg PRIVATE
 *       DVG_SPHERICAL_BACKEND=DVG_SPHERICAL_BACKEND_EXACT_TRIG
 *   )
 *
 * Slots 3-6 are intentionally reserved for future approximation experiments.
 * Selecting one of those before implementing its function in SphericalDedup.c
 * produces a clear compile-time error rather than silently benchmarking the
 * wrong implementation.
 */

#define DVG_SPHERICAL_BACKEND_EXACT_TRIG      1
#define DVG_SPHERICAL_BACKEND_SQUARED_LUT     2
#define DVG_SPHERICAL_BACKEND_POLYNOMIAL      3
#define DVG_SPHERICAL_BACKEND_TAYLOR          4
#define DVG_SPHERICAL_BACKEND_MINMAX          5
#define DVG_SPHERICAL_BACKEND_RECURSIVE_VECTOR_SUBDIVISION 6

#ifndef DVG_SPHERICAL_BACKEND
#define DVG_SPHERICAL_BACKEND \
    DVG_SPHERICAL_BACKEND_EXACT_TRIG
#endif


/*
 * ---------------------------------------------------------------------------
 * Angular grid configuration
 * ---------------------------------------------------------------------------
 */

#define DVG_PI                  3.14159265358979323846
#define DVG_HALF_PI             (DVG_PI / 2.0)

#define DVG_SPHERICAL_DIVISIONS 128

#ifdef __cplusplus

static_assert(
    DVG_SPHERICAL_DIVISIONS > 0,
    "DVG spherical divisions must be greater than zero"
);

static_assert(
    DVG_SPHERICAL_DIVISIONS % 2 == 0,
    "DVG spherical divisions must be even"
);

static_assert(
    DVG_SPHERICAL_BACKEND >= DVG_SPHERICAL_BACKEND_EXACT_TRIG &&
    DVG_SPHERICAL_BACKEND <= DVG_SPHERICAL_BACKEND_RECURSIVE_VECTOR_SUBDIVISION,
    "Invalid DVG spherical backend"
);

#else

_Static_assert(
    DVG_SPHERICAL_DIVISIONS > 0,
    "DVG spherical divisions must be greater than zero"
);

_Static_assert(
    DVG_SPHERICAL_DIVISIONS % 2 == 0,
    "DVG spherical divisions must be even"
);

_Static_assert(
    DVG_SPHERICAL_BACKEND >= DVG_SPHERICAL_BACKEND_EXACT_TRIG &&
    DVG_SPHERICAL_BACKEND <=  DVG_SPHERICAL_BACKEND_RECURSIVE_VECTOR_SUBDIVISION,
    "Invalid DVG spherical backend"
);

#endif


#define DVG_ANGLE_INTERVAL \
    (DVG_PI / DVG_SPHERICAL_DIVISIONS)

#define DVG_AZIMUTH_BUCKETS \
    (DVG_SPHERICAL_DIVISIONS * 2u)

#define DVG_ELEVATION_BUCKETS \
    DVG_SPHERICAL_DIVISIONS

#define DVG_ANGLE_BUCKETS \
    (DVG_AZIMUTH_BUCKETS * DVG_ELEVATION_BUCKETS)


typedef struct
{
    float longest_dist_squared;
    float shortest_dist_squared;
    DvgVector_t dynamic_object_removal_target; //furthest away point
    DvgVector_t icp_target;//nearest point
} DvgSphericalAngleEntry;


/*
 * Returns a stable human-readable name for the backend compiled into the
 * library. Useful for benchmark output so runs cannot be mislabeled.
 */
const char *spherical_dedup_backend_name(void);

void spherical_dedup_wipe_arr(void);

void spherical_dedup_request(
    int64_t org_x,
    int64_t org_y,
    int64_t org_z,
    int64_t dest_x,
    int64_t dest_y,
    int64_t dest_z
);

/*
 * uint32_t is intentional: with 256 spherical divisions the complete
 * angle table contains 131072 entries and no longer fits in uint16_t.
 */
DvgSphericalAngleEntry spherical_dedup_get_entry(uint32_t index);


void spherical_dedup_set_icp_entry(uint32_t index, DvgVector_t vector);

void spherical_dedup_set_dynamic_object_removal_entry(uint32_t index, DvgVector_t vector);
/*
 * Legacy declarations retained for compatibility with existing callers.
 * They are unrelated to backend selection.
 */
void diff_vect_to_spherical_angle(
    DvgVector_t *input_vector,
    float *out_azimuth,
    float *out_elevation
);

uint32_t spherical_angle_to_index(
    float in_azimuth,
    float in_elevation
);

uint32_t spherical_dedup_get_evlev_by_lut(float q);


#ifdef __cplusplus
}
#endif

#endif
