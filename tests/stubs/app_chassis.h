#ifndef TEST_APP_CHASSIS_H
#define TEST_APP_CHASSIS_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    float distance_mm;
    uint32_t age_ms, reset_count, frame_id;
} Chassis_DistanceSample_t;
int app_chassis_move(float vx, float vy, float omega);
float app_chassis_get_distance(void);
bool app_chassis_get_distance_sample(Chassis_DistanceSample_t *out);
#endif
