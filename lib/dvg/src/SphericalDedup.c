#include "DvgVector.h"
#include <SphericalDedup.h>

#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

#define DVG_INVALID_SPHERICAL_INDEX UINT32_MAX


/*
 * ===========================================================================
 * Shared helpers
 * ===========================================================================
 *
 * All backends receive the same already-computed:
 *
 *   horizontal_squared = dx^2 + dz^2
 *   dist_squared       = horizontal_squared + dy^2
 *
 * This is deliberate. It keeps the benchmark semantics identical between
 * backends: only the angle-to-bucket approximation changes.
 */

static uint32_t quantize_elevation_exact_angle(double phi)
{
    uint32_t bucket =
        (uint32_t)(
            (phi + DVG_HALF_PI) /
            DVG_ANGLE_INTERVAL
        );

    if (bucket >= DVG_ELEVATION_BUCKETS)
        bucket = DVG_ELEVATION_BUCKETS - 1;

    return bucket;
}


#if DVG_SPHERICAL_BACKEND == DVG_SPHERICAL_BACKEND_EXACT_TRIG

static uint32_t quantize_azimuth_exact_angle(double alpha)
{
    /*
     * atan2() returns [-pi, +pi].
     *
     * +pi is intentionally clamped into the final bucket rather than wrapped
     * to bucket zero. The LUT backend uses the same boundary convention.
     */
    uint32_t bucket =
        (uint32_t)(
            (alpha + DVG_PI) /
            DVG_ANGLE_INTERVAL
        );

    if (bucket >= DVG_AZIMUTH_BUCKETS)
        bucket = DVG_AZIMUTH_BUCKETS - 1;

    return bucket;
}

#endif


/*
 * ===========================================================================
 * Backend 1: exact trigonometry
 * ===========================================================================
 */

#if DVG_SPHERICAL_BACKEND == DVG_SPHERICAL_BACKEND_EXACT_TRIG

static bool spherical_indices_exact_trig(
    double dx,
    double dy,
    double dz,
    double dist_squared,
    double horizontal_squared,
    uint32_t *azimuth_index,
    uint32_t *elevation_index)
{
    (void)dist_squared;

    /*
     * Elevation:
     *   phi = atan2(y, sqrt(x^2 + z^2))
     *
     * This remains well-defined at the vertical poles.
     */
    const double phi =
        atan2(
            dy,
            sqrt(horizontal_squared)
        );

    *elevation_index =
        quantize_elevation_exact_angle(phi);

    /*
     * Azimuth is undefined at an exact vertical pole.
     * Use deterministic bucket zero, matching the LUT backend.
     */
    if (horizontal_squared == 0.0)
    {
        *azimuth_index = 0;
        return true;
    }

    if (!(horizontal_squared > 0.0))
        return false;

    const double alpha =
        atan2(dz, dx);

    *azimuth_index =
        quantize_azimuth_exact_angle(alpha);

    return true;
}

#endif


/*
 * ===========================================================================
 * Backend 2: signed squared-trigonometric LUT
 * ===========================================================================
 */

#if DVG_SPHERICAL_BACKEND == DVG_SPHERICAL_BACKEND_SQUARED_LUT

#define DVG_Q_LUT_SIZE \
    ((DVG_SPHERICAL_DIVISIONS * DVG_SPHERICAL_DIVISIONS + 3u) / 4u)

static uint32_t elevation_lut[DVG_Q_LUT_SIZE];
static uint32_t azi_lut[DVG_Q_LUT_SIZE];

static bool first_elev_lut_hit = true;
static bool first_azi_lut_hit = true;


static void compute_azi_lut(uint32_t *lut)
{
    for (uint32_t i = 0; i < DVG_Q_LUT_SIZE; i++)
    {
        /*
         * LUT index -> representative signed q in [-1, 1].
         */
        const double q =
            -1.0 +
            2.0 *
            (((double)i + 0.5) /
             (double)DVG_Q_LUT_SIZE);

        /*
         * q = sign(cos(alpha)) * cos^2(alpha)
         *
         * Undo the square and restore the cosine sign.
         * This only runs during LUT construction.
         */
        const double cos_alpha =
            copysign(
                sqrt(fabs(q)),
                q
            );

        /*
         * acos gives the half-circle angle [0, pi].
         */
        const double theta =
            acos(cos_alpha);

        uint32_t bucket =
            (uint32_t)(
                theta /
                DVG_ANGLE_INTERVAL
            );

        if (bucket >= DVG_SPHERICAL_DIVISIONS)
            bucket = DVG_SPHERICAL_DIVISIONS - 1;

        lut[i] = bucket;
    }
}


static void compute_elev_lut(uint32_t *lut)
{
    for (uint32_t i = 0; i < DVG_Q_LUT_SIZE; i++)
    {
        /*
         * LUT index -> representative signed q in [-1, 1].
         */
        const double q =
            -1.0 +
            2.0 *
            (((double)i + 0.5) /
             (double)DVG_Q_LUT_SIZE);

        /*
         * q = sign(phi) * sin^2(phi)
         *
         * Undo the square and sine, then restore the sign.
         * This only runs during LUT construction.
         */
        const double phi =
            copysign(
                asin(sqrt(fabs(q))),
                q
            );

        lut[i] =
            quantize_elevation_exact_angle(phi);
    }
}


static uint32_t elev_lut_lookup(
    double dy,
    double dist_squared)
{
    if (first_elev_lut_hit)
    {
        compute_elev_lut(elevation_lut);
        first_elev_lut_hit = false;
    }

    /*
     * q_abs = sin^2(phi)
     */
    double q =
        (dy * dy) /
        dist_squared;

    /*
     * Restore elevation sign:
     * q in [-1, 1].
     */
    q = copysign(q, dy);

    /*
     * Protect against tiny floating-point excursions.
     */
    q = fmax(-1.0, fmin(1.0, q));

    /*
     * [-1, 1] -> [0, 1]
     */
    const double normalized_q =
        (q + 1.0) * 0.5;

    uint32_t index =
        (uint32_t)(
            normalized_q *
            (double)DVG_Q_LUT_SIZE
        );

    /*
     * q == +1 maps mathematically to LUT_SIZE.
     */
    if (index >= DVG_Q_LUT_SIZE)
        index = DVG_Q_LUT_SIZE - 1;

    return elevation_lut[index];
}


static uint32_t azi_lut_lookup(
    double dx,
    double dz,
    double horizontal_squared)
{
    if (first_azi_lut_hit)
    {
        compute_azi_lut(azi_lut);
        first_azi_lut_hit = false;
    }

    /*
     * At an exact vertical pole azimuth is undefined but irrelevant.
     * Use deterministic bucket zero, matching the exact-trig backend.
     */
    if (horizontal_squared == 0.0)
        return 0;

    if (!(horizontal_squared > 0.0))
        return DVG_INVALID_SPHERICAL_INDEX;

    /*
     * q_abs = cos^2(alpha)
     */
    double q =
        (dx * dx) /
        horizontal_squared;

    /*
     * Restore sign(cos(alpha)) from dx:
     * q in [-1, 1].
     */
    q = copysign(q, dx);

    /*
     * Protect against tiny floating-point excursions.
     */
    q = fmax(-1.0, fmin(1.0, q));

    /*
     * [-1, 1] -> [0, 1]
     */
    const double normalized_q =
        (q + 1.0) * 0.5;

    uint32_t index =
        (uint32_t)(
            normalized_q *
            (double)DVG_Q_LUT_SIZE
        );

    if (index >= DVG_Q_LUT_SIZE)
        index = DVG_Q_LUT_SIZE - 1;

    /*
     * LUT gives a half-circle acos bucket in [0, D-1].
     * sign(dz) unfolds it into the full azimuth range.
     */
    const uint32_t half_bucket =
        azi_lut[index];

    const uint32_t negative_z =
        (uint32_t)(signbit(dz) != 0);

    /*
     * dz >= 0: D + half_bucket
     * dz <  0: D - 1 - half_bucket
     *
     * Result range: [0, 2*D - 1]
     */
    return
        DVG_SPHERICAL_DIVISIONS
        + half_bucket
        - negative_z *
          (2u * half_bucket + 1u);
}


static bool spherical_indices_squared_lut(
    double dx,
    double dy,
    double dz,
    double dist_squared,
    double horizontal_squared,
    uint32_t *azimuth_index,
    uint32_t *elevation_index)
{
    *elevation_index =
        elev_lut_lookup(
            dy,
            dist_squared
        );

    *azimuth_index =
        azi_lut_lookup(
            dx,
            dz,
            horizontal_squared
        );

    return
        *elevation_index != DVG_INVALID_SPHERICAL_INDEX &&
        *azimuth_index != DVG_INVALID_SPHERICAL_INDEX;
}

#endif


/*
 * ===========================================================================
 * Backend 3: polynomial approximation
 * ===========================================================================
 *
 * The slot is wired into the dispatcher already. Implement this function when
 * you are ready to benchmark a polynomial inverse-trigonometric approximation.
 */

#if DVG_SPHERICAL_BACKEND == DVG_SPHERICAL_BACKEND_POLYNOMIAL

#error \
"DVG spherical polynomial backend selected, but spherical_indices_polynomial() is not implemented yet."

static bool spherical_indices_polynomial(
    double dx,
    double dy,
    double dz,
    double dist_squared,
    double horizontal_squared,
    uint32_t *azimuth_index,
    uint32_t *elevation_index)
{
    (void)dx;
    (void)dy;
    (void)dz;
    (void)dist_squared;
    (void)horizontal_squared;
    (void)azimuth_index;
    (void)elevation_index;
    return false;
}

#endif


/*
 * ===========================================================================
 * Backend 4: Taylor-series approximation
 * ===========================================================================
 */

#if DVG_SPHERICAL_BACKEND == DVG_SPHERICAL_BACKEND_TAYLOR

#error \
"DVG spherical Taylor backend selected, but spherical_indices_taylor() is not implemented yet."

static bool spherical_indices_taylor(
    double dx,
    double dy,
    double dz,
    double dist_squared,
    double horizontal_squared,
    uint32_t *azimuth_index,
    uint32_t *elevation_index)
{
    (void)dx;
    (void)dy;
    (void)dz;
    (void)dist_squared;
    (void)horizontal_squared;
    (void)azimuth_index;
    (void)elevation_index;
    return false;
}

#endif


/*
 * ===========================================================================
 * Backend 5: min/max approximation
 * ===========================================================================
 */

#if DVG_SPHERICAL_BACKEND == DVG_SPHERICAL_BACKEND_MINMAX

#error \
"DVG spherical min/max backend selected, but spherical_indices_minmax() is not implemented yet."

static bool spherical_indices_minmax(
    double dx,
    double dy,
    double dz,
    double dist_squared,
    double horizontal_squared,
    uint32_t *azimuth_index,
    uint32_t *elevation_index)
{
    (void)dx;
    (void)dy;
    (void)dz;
    (void)dist_squared;
    (void)horizontal_squared;
    (void)azimuth_index;
    (void)elevation_index;
    return false;
}

#endif


/*
 * ===========================================================================
 * Backend 6: free experimental slot
 * ===========================================================================
 */

#if DVG_SPHERICAL_BACKEND ==  DVG_SPHERICAL_BACKEND_RECURSIVE_VECTOR_SUBDIVISION

#error \
"DVG spherical experimental backend 6 selected, but vector_subdivision() is not implemented yet."

static bool spherical_indices_experimental_6(
    double dx,
    double dy,
    double dz,
    double dist_squared,
    double horizontal_squared,
    uint32_t *azimuth_index,
    uint32_t *elevation_index)
{
    (void)dx;
    (void)dy;
    (void)dz;
    (void)dist_squared;
    (void)horizontal_squared;
    (void)azimuth_index;
    (void)elevation_index;
    return false;
}

#endif


/*
 * ===========================================================================
 * Compile-time dispatcher
 * ===========================================================================
 */

static bool spherical_indices_backend(
    double dx,
    double dy,
    double dz,
    double dist_squared,
    double horizontal_squared,
    uint32_t *azimuth_index,
    uint32_t *elevation_index)
{
#if DVG_SPHERICAL_BACKEND == DVG_SPHERICAL_BACKEND_EXACT_TRIG

    return spherical_indices_exact_trig(
        dx,
        dy,
        dz,
        dist_squared,
        horizontal_squared,
        azimuth_index,
        elevation_index
    );

#elif DVG_SPHERICAL_BACKEND == DVG_SPHERICAL_BACKEND_SQUARED_LUT

    return spherical_indices_squared_lut(
        dx,
        dy,
        dz,
        dist_squared,
        horizontal_squared,
        azimuth_index,
        elevation_index
    );

#elif DVG_SPHERICAL_BACKEND == DVG_SPHERICAL_BACKEND_POLYNOMIAL

    return spherical_indices_polynomial(
        dx,
        dy,
        dz,
        dist_squared,
        horizontal_squared,
        azimuth_index,
        elevation_index
    );

#elif DVG_SPHERICAL_BACKEND == DVG_SPHERICAL_BACKEND_TAYLOR

    return spherical_indices_taylor(
        dx,
        dy,
        dz,
        dist_squared,
        horizontal_squared,
        azimuth_index,
        elevation_index
    );

#elif DVG_SPHERICAL_BACKEND == DVG_SPHERICAL_BACKEND_MINMAX

    return spherical_indices_minmax(
        dx,
        dy,
        dz,
        dist_squared,
        horizontal_squared,
        azimuth_index,
        elevation_index
    );

#elif DVG_SPHERICAL_BACKEND == DVG_SPHERICAL_BACKEND_EXPERIMENTAL_6

    return spherical_indices_experimental_6(
        dx,
        dy,
        dz,
        dist_squared,
        horizontal_squared,
        azimuth_index,
        elevation_index
    );

#else

#error "Invalid DVG_SPHERICAL_BACKEND"

#endif
}


/*
 * ===========================================================================
 * Public API
 * ===========================================================================
 */

const char *spherical_dedup_backend_name(void)
{
#if DVG_SPHERICAL_BACKEND == DVG_SPHERICAL_BACKEND_EXACT_TRIG
    return "exact_trig";
#elif DVG_SPHERICAL_BACKEND == DVG_SPHERICAL_BACKEND_SQUARED_LUT
    return "squared_trig_lut";
#elif DVG_SPHERICAL_BACKEND == DVG_SPHERICAL_BACKEND_POLYNOMIAL
    return "polynomial";
#elif DVG_SPHERICAL_BACKEND == DVG_SPHERICAL_BACKEND_TAYLOR
    return "taylor";
#elif DVG_SPHERICAL_BACKEND == DVG_SPHERICAL_BACKEND_MINMAX
    return "minmax";
#elif DVG_SPHERICAL_BACKEND == DVG_SPHERICAL_BACKEND_EXPERIMENTAL_6
    return "experimental_6";
#else
    return "invalid";
#endif
}


static DvgSphericalAngleEntry angle_table[DVG_ANGLE_BUCKETS];


DvgSphericalAngleEntry spherical_dedup_get_entry(uint32_t index)
{
    return angle_table[index];
}

DvgSphericalAngleEntry spherical_dedup_set_icp_entry(uint32_t index, DvgVector_t vector){
    angle_table[index].icp_target = vector;
}

DvgSphericalAngleEntry spherical_dedup_set_dynaminc_object_removal_entry(uint32_t index, DvgVector_t vector){
    angle_table[index].dynamic_object_removal_target = vector;
}

void spherical_dedup_wipe_arr(void)
{
    for (uint32_t i = 0; i < DVG_ANGLE_BUCKETS; i++)
    {
        angle_table[i].longest_dist_squared = 0.0f;
        angle_table[i].shortest_dist_squared = 9999.9f;

        angle_table[i].dynamic_object_removal_target.x = 0.0;
        angle_table[i].dynamic_object_removal_target.y = 0.0;
        angle_table[i].dynamic_object_removal_target.z = 0.0;
        
        angle_table[i].icp_target.x = 0.0;
        angle_table[i].icp_target.y = 0.0;
        angle_table[i].icp_target.z = 0.0;
    }
}


void spherical_dedup_request(
    int64_t org_x,
    int64_t org_y,
    int64_t org_z,
    int64_t dest_x,
    int64_t dest_y,
    int64_t dest_z)
{
    /*
     * DvgVector_t stores doubles. Cast before subtraction so the subtraction
     * cannot overflow int64_t before conversion.
     */
    const double dx =
        (double)dest_x -
        (double)org_x;

    const double dy =
        (double)dest_y -
        (double)org_y;

    const double dz =
        (double)dest_z -
        (double)org_z;

    /*
     * Compute these once, before backend dispatch.
     *
     * This is shared by every backend so A/B timing changes come from the
     * approximation itself rather than from different pipeline arithmetic.
     */
    const double horizontal_squared =
        dx * dx +
        dz * dz;

    const double dist_squared =
        horizontal_squared +
        dy * dy;

    /*
     * A zero-length or NaN vector has no defined spherical direction.
     */
    if (!(dist_squared > 0.0))
        return;

    uint32_t azimuth_index;
    uint32_t elevation_index;

    if (!spherical_indices_backend(
            dx,
            dy,
            dz,
            dist_squared,
            horizontal_squared,
            &azimuth_index,
            &elevation_index))
    {
        return;
    }

    /*
     * Same 2D table layout for every backend:
     *
     *   index =
     *       azimuth * elevation_bucket_count
     *       + elevation
     */
    const uint32_t index =
        DVG_ELEVATION_BUCKETS *
        azimuth_index +
        elevation_index;

    if (index >= DVG_ANGLE_BUCKETS)
        return;

    /*
     * Keep the farthest return in each angular bucket.
     * This downstream semantic is identical for all backends.
     */
    if (angle_table[index].longest_dist_squared < dist_squared)
    {
        angle_table[index].longest_dist_squared =
            (float)dist_squared;

        angle_table[index]
            .dynamic_object_removal_target.x =
            dest_x;

        angle_table[index]
            .dynamic_object_removal_target.y =
            dest_y;

        angle_table[index]
            .dynamic_object_removal_target.z =
            dest_z;
    }
    if (angle_table[index].shortest_dist_squared > dist_squared)
    {
        angle_table[index].shortest_dist_squared =
            (float)dist_squared;

        angle_table[index]
            .icp_target.x =
            dest_x;

        angle_table[index]
            .icp_target.y =
            dest_y;

        angle_table[index]
            .icp_target.z =
            dest_z;
    }
}
