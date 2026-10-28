#include "alg_chassis_path.h"
#include "alg_motion_profile.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#define PATH_PI 3.14159265358979323846f
#define PATH_MAX_PHASES (3U * ALG_PATH_MAX_SEGMENTS)

typedef struct {
    AlgPathSegment_t input;
    float x, y, yaw;
    float end_x, end_y;
    float length, offset;
    float beta, curvature_peak, curvature_derivative_peak;
} PathGeometry_t;

typedef struct {
    MotionProfilePlan_t profile;
    float length, offset;
    float speed_limit, accel, decel;
    uint8_t segment;
} PathPhase_t;

static PathGeometry_t s_geometry[ALG_PATH_MAX_SEGMENTS];
static PathPhase_t s_phase[PATH_MAX_PHASES];
static float s_boundary_speed[PATH_MAX_PHASES + 1U];
static uint8_t s_phase_count, s_current_phase;
static float s_phase_time;
static AlgPathSample_t s_sample;

static float smooth(float t)
{
    return t * t * t * (10.0f + t * (-15.0f + 6.0f * t));
}

static float smooth_integral(float t)
{
    return t * t * t * t * (2.5f + t * (-3.0f + t));
}

static void curve_shape(float u, float beta, float *heading, float *h, float *dh)
{
    u = fmaxf(0.0f, fminf(1.0f, u));
    if (u < beta) {
        float t = u / beta;
        *heading = beta * smooth_integral(t) / (1.0f - beta);
        *h = smooth(t);
        *dh = 30.0f * t * t * (1.0f - t) * (1.0f - t) / beta;
    } else if (u > 1.0f - beta) {
        float t = (1.0f - u) / beta;
        *heading = 1.0f - beta * smooth_integral(t) / (1.0f - beta);
        *h = smooth(t);
        *dh = -30.0f * t * t * (1.0f - t) * (1.0f - t) / beta;
    } else {
        *heading = (u - 0.5f * beta) / (1.0f - beta);
        *h = 1.0f;
        *dh = 0.0f;
    }
}

/* Eight-point Gauss-Legendre, split at the two curvature-ramp boundaries. */
static void integrate_piece(float a, float b, float beta, float yaw, float angle,
                             float *dx, float *dy)
{
    static const float node[4] = {0.1834346425f, 0.5255324099f, 0.7966664774f, 0.9602898565f};
    static const float weight[4] = {0.3626837834f, 0.3137066459f, 0.2223810345f, 0.1012285363f};
    float mid = 0.5f * (a + b), half = 0.5f * (b - a);
    for (unsigned n = 0; n < 4U; ++n) {
        for (int sign = -1; sign <= 1; sign += 2) {
            float fraction, h, dh;
            curve_shape(mid + sign * half * node[n], beta, &fraction, &h, &dh);
            float heading = yaw + angle * fraction;
            *dx -= half * weight[n] * sinf(heading);
            *dy += half * weight[n] * cosf(heading);
        }
    }
}

static void integrate_curve(float u, float beta, float yaw, float angle,
                             float *dx, float *dy)
{
    float cuts[4] = {0.0f, beta, 1.0f - beta, 1.0f};
    *dx = *dy = 0.0f;
    for (unsigned n = 0; n < 3U && u > cuts[n]; ++n) {
        integrate_piece(cuts[n], fminf(u, cuts[n + 1U]), beta, yaw, angle, dx, dy);
    }
}

void alg_chassis_path_reset(void)
{
    s_phase_count = 0U;
    s_current_phase = 0U;
    s_phase_time = 0.0f;
    s_sample = (AlgPathSample_t){0};
}

static bool add_phase(uint8_t segment, float offset, float length,
                      float speed, float accel, float decel)
{
    if (s_phase_count >= PATH_MAX_PHASES || !isfinite(length) || length <= 0.0f ||
        !isfinite(speed) || speed <= 0.0f || !isfinite(accel) || accel <= 0.0f ||
        !isfinite(decel) || decel <= 0.0f) {
        return false;
    }
    s_phase[s_phase_count++] = (PathPhase_t){
        .length = length, .offset = offset, .speed_limit = speed,
        .accel = accel, .decel = decel, .segment = segment
    };
    return true;
}

static void evaluate_geometry(const PathGeometry_t *g, float progress,
                              float *k, float *dk)
{
    progress = fmaxf(0.0f, fminf(progress, g->length));
    if (g->input.type == ALG_PATH_LINE) {
        s_sample.x = g->x - progress * sinf(g->yaw);
        s_sample.y = g->y + progress * cosf(g->yaw);
        s_sample.yaw = g->yaw;
        *k = *dk = 0.0f;
    } else {
        float fraction, h, dh, dx, dy;
        float u = progress / g->length;
        curve_shape(u, g->beta, &fraction, &h, &dh);
        integrate_curve(u, g->beta, g->yaw, g->input.angle, &dx, &dy);
        s_sample.x = g->x + g->length * dx;
        s_sample.y = g->y + g->length * dy;
        s_sample.yaw = g->yaw + g->input.angle * fraction;
        float peak = g->input.angle / ((1.0f - g->beta) * g->length);
        *k = peak * h;
        *dk = peak * dh / g->length;
    }
    if (progress >= g->length) {
        s_sample.x = g->end_x;
        s_sample.y = g->end_y;
        s_sample.yaw = g->yaw + (g->input.type == ALG_PATH_ARC ? g->input.angle : 0.0f);
    }
}

static void update_sample(void)
{
    PathPhase_t *phase = &s_phase[s_current_phase];
    PathGeometry_t *g = &s_geometry[phase->segment];
    float progress, speed, accel, k, dk;
    (void)alg_motion_profile_sample(&phase->profile, s_phase_time, &progress, &speed, &accel);
    evaluate_geometry(g, phase->offset + progress, &k, &dk);
    s_sample.segment = phase->segment;
    s_sample.distance = g->offset + phase->offset + progress;
    s_sample.curvature = k;
    s_sample.speed = speed;
    s_sample.omega = speed * k;
    s_sample.angular_accel = accel * k + speed * speed * dk;
}

bool alg_chassis_path_plan(const AlgPathSegment_t *segments, uint8_t count,
                           const AlgPathLimits_t *limits, float start_yaw)
{
    alg_chassis_path_reset();
    if (segments == NULL || limits == NULL || count == 0U || count > ALG_PATH_MAX_SEGMENTS ||
        !isfinite(start_yaw) || fabsf(start_yaw) > 10000.0f ||
        !isfinite(limits->transition_mm) || limits->transition_mm <= 0.0f ||
        !isfinite(limits->max_omega) || limits->max_omega <= 0.0f ||
        !isfinite(limits->max_angular_accel) || limits->max_angular_accel <= 0.0f ||
        !isfinite(limits->max_lateral_accel) || limits->max_lateral_accel <= 0.0f) {
        return false;
    }
    float x = 0.0f, y = 0.0f, yaw = start_yaw, offset = 0.0f;
    for (uint8_t n = 0U; n < count; ++n) {
        const AlgPathSegment_t *in = &segments[n];
        if (!isfinite(in->max_speed) || in->max_speed <= 0.0f ||
            !isfinite(in->accel) || in->accel <= 0.0f ||
            !isfinite(in->decel) || in->decel <= 0.0f) {
            goto invalid;
        }
        PathGeometry_t *g = &s_geometry[n];
        *g = (PathGeometry_t){.input = *in, .x = x, .y = y, .yaw = yaw, .offset = offset};
        if (in->type == ALG_PATH_LINE) {
            if (!isfinite(in->distance) || in->distance < 0.01f) {
                goto invalid;
            }
            g->length = in->distance;
            g->end_x = x - g->length * sinf(yaw);
            g->end_y = y + g->length * cosf(yaw);
            if (!add_phase(n, 0.0f, g->length, in->max_speed, in->accel, in->decel)) {
                goto invalid;
            }
        } else if (in->type == ALG_PATH_ARC) {
            float angle = fabsf(in->angle);
            if (!isfinite(in->radius) || in->radius < 1.0f ||
                !isfinite(angle) || angle < 0.0001f || angle > PATH_PI ||
                !isfinite(in->radius * angle)) {
                goto invalid;
            }
            g->beta = limits->transition_mm / (in->radius * angle);
            if (!isfinite(g->beta) || g->beta < 0.001f || g->beta > 0.25f) {
                goto invalid;
            }
            float dx, dy;
            integrate_curve(1.0f, g->beta, -0.5f * in->angle, in->angle, &dx, &dy);
            g->length = 2.0f * in->radius * sinf(0.5f * angle) / dy;
            g->curvature_peak = angle / ((1.0f - g->beta) * g->length);
            g->curvature_derivative_peak = 1.875f * g->curvature_peak / (g->beta * g->length);
            /* Half-angle chord avoids cancellation for very small turn angles. */
            float chord = 2.0f * in->radius * sinf(0.5f * angle);
            g->end_x = x - chord * sinf(yaw + 0.5f * in->angle);
            g->end_y = y + chord * cosf(yaw + 0.5f * in->angle);
            float speed = fminf(in->max_speed, limits->max_omega / g->curvature_peak);
            speed = fminf(speed, sqrtf(limits->max_lateral_accel / g->curvature_peak));
            /* Split the angular-acceleration budget between k*a and k'*v^2. */
            float accel = fminf(in->accel, 0.5f * limits->max_angular_accel / g->curvature_peak);
            float decel = fminf(in->decel, 0.5f * limits->max_angular_accel / g->curvature_peak);
            float ramp_speed = fminf(speed, sqrtf(0.5f * limits->max_angular_accel /
                                                               g->curvature_derivative_peak));
            float ramp = g->beta * g->length;
            if (!add_phase(n, 0.0f, ramp, ramp_speed, accel, decel) ||
                !add_phase(n, ramp, g->length - 2.0f * ramp, speed, accel, decel) ||
                !add_phase(n, g->length - ramp, ramp, ramp_speed, accel, decel)) {
                goto invalid;
            }
            yaw += in->angle;
        } else {
            goto invalid;
        }
        x = g->end_x;
        y = g->end_y;
        offset += g->length;
        if (!isfinite(x) || !isfinite(y) || !isfinite(yaw) || !isfinite(offset)) {
            goto invalid;
        }
    }
    s_boundary_speed[0] = s_boundary_speed[s_phase_count] = 0.0f;
    for (uint8_t n = 1U; n < s_phase_count; ++n) {
        s_boundary_speed[n] = fminf(s_phase[n - 1U].speed_limit, s_phase[n].speed_limit);
    }
    for (unsigned n = s_phase_count; n-- > 0U;) {
        float reachable = sqrtf(s_boundary_speed[n + 1U] * s_boundary_speed[n + 1U] +
                                 2.0f * s_phase[n].decel * s_phase[n].length);
        s_boundary_speed[n] = fminf(s_boundary_speed[n], reachable);
    }
    for (uint8_t n = 0U; n < s_phase_count; ++n) {
        float reachable = sqrtf(s_boundary_speed[n] * s_boundary_speed[n] +
                                 2.0f * s_phase[n].accel * s_phase[n].length);
        s_boundary_speed[n + 1U] = fminf(s_boundary_speed[n + 1U], reachable);
    }
    float duration = 0.0f;
    for (uint8_t n = 0U; n < s_phase_count; ++n) {
        PathPhase_t *p = &s_phase[n];
        if (!alg_motion_profile_plan(&p->profile, p->length, s_boundary_speed[n],
                                     s_boundary_speed[n + 1U], p->speed_limit, p->accel, p->decel)) {
            goto invalid;
        }
        duration += p->profile.duration;
        if (!isfinite(duration) || duration > 3600.0f) {
            goto invalid;
        }
    }
    s_sample.segment_count = count;
    s_sample.duration = duration;
    update_sample();
    return true;

invalid:
    alg_chassis_path_reset();
    return false;
}

bool alg_chassis_path_update(float dt)
{
    if (s_phase_count == 0U || s_sample.finished || !isfinite(dt) || dt <= 0.0f) {
        return s_sample.finished;
    }
    float remaining = dt;
    while (remaining > 0.0f) {
        float available = s_phase[s_current_phase].profile.duration - s_phase_time;
        if (remaining < available) {
            s_phase_time += remaining;
            break;
        }
        remaining -= fmaxf(available, 0.0f);
        if (s_current_phase + 1U >= s_phase_count) {
            s_phase_time = s_phase[s_current_phase].profile.duration;
            s_sample.finished = true;
            break;
        }
        ++s_current_phase;
        s_phase_time = 0.0f;
    }
    s_sample.elapsed = fminf(s_sample.duration, s_sample.elapsed + dt);
    update_sample();
    if (s_sample.finished) {
        s_sample.speed = s_sample.omega = s_sample.angular_accel = 0.0f;
        s_sample.elapsed = s_sample.duration;
    }
    return s_sample.finished;
}

bool alg_chassis_path_get_sample(AlgPathSample_t *out)
{
    if (out == NULL || s_phase_count == 0U) {
        return false;
    }
    *out = s_sample;
    return true;
}
