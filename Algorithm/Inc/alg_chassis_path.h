#ifndef ALG_CHASSIS_PATH_H
#define ALG_CHASSIS_PATH_H

#include <stdbool.h>
#include <stdint.h>

#define ALG_PATH_MAX_SEGMENTS 16U

typedef enum {
    ALG_PATH_LINE = 0,
    ALG_PATH_ARC
} AlgPathType_t;

typedef struct {
    AlgPathType_t type;
    float distance;       /* line: forward distance, mm; unused for an arc */
    float radius;         /* arc: nominal radius, mm */
    float angle;          /* arc: signed angle, rad, left positive; |angle| <= PI */
    float max_speed;      /* mm/s */
    float accel;          /* tangential mm/s^2 */
    float decel;          /* tangential mm/s^2 */
} AlgPathSegment_t;

typedef struct {
    float transition_mm;       /* curvature ramp length on the nominal arc */
    float max_omega;           /* reference angular speed, rad/s */
    float max_angular_accel;   /* reference angular acceleration, rad/s^2 */
    float max_lateral_accel;   /* normal acceleration, mm/s^2 */
} AlgPathLimits_t;

typedef struct {
    float x;                  /* reference displacement from path start, mm */
    float y;
    float yaw;                /* continuous absolute reference heading, rad */
    float speed;              /* reference forward speed, mm/s */
    float omega;              /* reference angular speed, rad/s */
    float angular_accel;      /* analytical reference angular acceleration */
    float curvature;           /* signed 1/mm */
    float distance;            /* reference path length traversed, mm */
    float elapsed;
    float duration;
    uint8_t segment;
    uint8_t segment_count;
    bool finished;
} AlgPathSample_t;

/* One main-loop-owned path, static storage, no allocation or I/O.
 * Copies inputs. Call only while the owner is idle. Invalid input clears plan.
 * Lines and arcs are tangent-connected; arcs preserve the nominal circular
 * endpoint and angle, with quintic curvature ramps (not constant-radius arcs).
 * transition_mm/(radius*abs(angle)) must be in [0.001, 0.25] for every arc.
 * |start_yaw| <= 10000 rad avoids unusable float heading resolution.
 * Planning work is bounded by 16 segments / 48 phases; total time <= 1 hour.
 */
bool alg_chassis_path_plan(const AlgPathSegment_t *segments, uint8_t count,
                           const AlgPathLimits_t *limits, float start_yaw);
void alg_chassis_path_reset(void);
/* Finite positive dt consumes time across any phase boundaries in this call. */
bool alg_chassis_path_update(float dt);
bool alg_chassis_path_get_sample(AlgPathSample_t *out);

#endif
