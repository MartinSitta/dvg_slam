#include "DynamicObjectRemoval.h"
#include "DvgVector.h"

DvgVector_t build_voxel_local_normal(Dvg_t* dvg, DvgVector_t ray_vect_normalized, int64_t voxel_x, int64_t voxel_y, int64_t voxel_z, int64_t radius){
        DvgVector_t cumulative_vect;
        cumulative_vect.x = 0.0;
        cumulative_vect.y = 0.0;
        cumulative_vect.z = 0.0;
        int64_t voxel_count = 0;
        for(int64_t x_offset = -radius; x_offset <= radius; x_offset++){
            for(int64_t y_offset = -radius; y_offset <= radius; y_offset++){
                for(int64_t z_offset = -radius; z_offset <= radius; z_offset++){
                    if(x_offset * x_offset + y_offset * y_offset + z_offset * z_offset < radius * radius){
                        if(dvg_lookup(dvg, voxel_x + x_offset, voxel_y + y_offset, voxel_z + z_offset) >= 2){
                            cumulative_vect.x += x_offset;
                            cumulative_vect.y += y_offset;
                            cumulative_vect.z += z_offset;
                            voxel_count++;
                        }
                    }
                }
            }
        }
        double vector_len2 = cumulative_vect.x * cumulative_vect.x + cumulative_vect.y * cumulative_vect.y + cumulative_vect.z * cumulative_vect.z;
        if(voxel_count < 2 || vector_len2 < 1.0){
            DvgVector_t out_vect;
            out_vect.x = -ray_vect_normalized.x;
            out_vect.y = -ray_vect_normalized.y;
            out_vect.z = -ray_vect_normalized.z;
            return out_vect;
        }
        DvgVector_t cumulative_vect_norm = dvg_vector_normalize(cumulative_vect);
        DvgVector_t normal = {-cumulative_vect_norm.x, -cumulative_vect_norm.y, -cumulative_vect_norm.z};
        return normal;
    }

void dynamic_object_removal(Dvg_t* dvg, int64_t org_x, int64_t org_y, int64_t org_z,
                    int64_t dest_x, int64_t dest_y, int64_t dest_z,
                    bool splash_delete,
                    double max_range_meters, int64_t scalar){
    if(org_x == dest_x && org_y == dest_y && org_z == dest_z) return;
    DvgVector_t diff_vect;
    diff_vect.x = (double)(dest_x - org_x);
    diff_vect.y = (double)(dest_y - org_y);
    diff_vect.z = (double)(dest_z - org_z);
        double len2 = diff_vect.x * diff_vect.x
                    + diff_vect.y * diff_vect.y
                    + diff_vect.z * diff_vect.z;

        if(len2 < 1.0) return;
        double ray_len = sqrt(len2);

        // Convert max range from meters to your internal coordinate units.
        double max_range_units = max_range_meters * (double)scalar;

        // If the measured endpoint is closer than the clamp, use the real endpoint.
        // Otherwise only travel up to the clamp.
        double clamped_ray_len = fmin(ray_len, max_range_units);

        if(clamped_ray_len < 1.0) return;

        bool endpoint_inside_clamp = ray_len <= max_range_units;

        DvgVector_t normal = dvg_vector_normalize(diff_vect);

        int64_t current_del_count = 0;
        int64_t max_deletions = fmax(1, (double)scalar * 0.1f);
        const double DELETE_NORMAL_DOT_THRESHOLD = -0.2;
        uint32_t counter = 1;

        uint32_t threshold = 0.15f * (float)scalar;
        int64_t splash_radius = fmax(1, (double)scalar * 0.10f);

        if(splash_delete){
            threshold = splash_radius * 2 + threshold;
        }

        double threshold2 = (double)threshold * (double)threshold;

        uint32_t max_iter = (uint32_t)ceil(clamped_ray_len) + 1;

        int64_t travel_x = round(org_x + normal.x * counter);
        int64_t travel_y = round(org_y + normal.y * counter);
        int64_t travel_z = round(org_z + normal.z * counter);

        while(counter < max_iter){
            // Only use endpoint threshold if the real endpoint is inside the clamp.
            // For far-away points, we do not care about the endpoint threshold,
            // because the ray should stop at max_range_meters instead.
            if(endpoint_inside_clamp){
                double dx = (double)(travel_x - dest_x);
                double dy = (double)(travel_y - dest_y);
                double dz = (double)(travel_z - dest_z);

                double dist2_to_dest = dx * dx + dy * dy + dz * dz;

                if(dist2_to_dest <= threshold2){
                    return;
                }
            }

            bool point_detected = dvg_lookup(dvg, travel_x, travel_y, travel_z);

            if(current_del_count >= max_deletions){
                return;
            }

            if(point_detected){
                DvgVector_t voxel_normal =
                    build_voxel_local_normal(dvg, normal, travel_x, travel_y, travel_z, 6);

                double angle = voxel_normal.x * normal.x
                            + voxel_normal.y * normal.y
                            + voxel_normal.z * normal.z;

                if(angle <= DELETE_NORMAL_DOT_THRESHOLD){
                    if(splash_delete){
                        for(int64_t sx = travel_x - splash_radius; sx <= travel_x + splash_radius; sx++){
                            for(int64_t sy = travel_y - splash_radius; sy <= travel_y + splash_radius; sy++){
                                for(int64_t sz = travel_z - splash_radius; sz <= travel_z + splash_radius; sz++){
                                    dvg_delete(dvg, sx, sy, sz);
                                }
                            }
                        }
                        current_del_count++;
                    }
                    else{
                        current_del_count++;
                        dvg_delete(dvg, travel_x, travel_y, travel_z);
                    }
                }
            }

            counter++;

            travel_x = round(org_x + normal.x * counter);
            travel_y = round(org_y + normal.y * counter);
            travel_z = round(org_z + normal.z * counter);
        }
    }