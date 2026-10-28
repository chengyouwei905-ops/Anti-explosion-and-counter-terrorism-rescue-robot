#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include "alg_chassis_path.h"
#include "alg_motion_profile.h"
#include "app_chassis_motion.h"
#include "app_chassis_closeloop.h"

#define PI 3.14159265358979323846f
#define LINE(d,v,a,b) {ALG_PATH_LINE,(d),0,0,(v),(a),(b)}
#define ARC(r,angle,v,a,b) {ALG_PATH_ARC,0,(r),(angle),(v),(a),(b)}
static const AlgPathLimits_t limits = {30.0f,0.9f,2.0f,400.0f};
static bool imu_valid = true;
static float imu_yaw,vx,vy,omega,world_x,world_y;

bool app_imu_get_euler(float e[3])
{
    e[0]=e[1]=0; e[2]=imu_yaw; return imu_valid;
}
int app_chassis_move(float x,float y,float w)
{
    assert(isfinite(x) && isfinite(y) && isfinite(w));
    vx=x;vy=y;omega=w; return 0;
}
static void near(float actual,float expected,float tolerance)
{
    if(!isfinite(actual) || fabsf(actual-expected)>tolerance) {
        fprintf(stderr,"actual=%g expected=%g tolerance=%g\n",actual,expected,tolerance);
        assert(0);
    }
}
static void tick(float dt)
{
    app_chassis_motion_update(dt);
    app_chassis_closeloop_update();
    float mid=imu_yaw+omega*dt*0.5f;
    world_x+=(cosf(mid)*vx-sinf(mid)*vy)*dt;
    world_y+=(sinf(mid)*vx+cosf(mid)*vy)*dt;
    imu_yaw+=omega*dt;
}
static void reset(void)
{
    imu_valid=true;imu_yaw=world_x=world_y=0;
    app_chassis_motion_init();
}
static void test_boundary_profiles(void)
{
    MotionProfilePlan_t p;
    float s,v,a;
    assert(alg_motion_profile_plan(&p,200,50,80,100,100,100));
    assert(alg_motion_profile_sample(&p,0,&s,&v,&a));
    near(s,0,1e-6f);near(v,50,1e-6f);
    float previous=0;
    for(float t=0;t<p.duration;t+=0.001f) {
        assert(alg_motion_profile_sample(&p,t,&s,&v,&a));
        assert(s>=previous-1e-4f && v<=100.001f);
        previous=s;
    }
    assert(alg_motion_profile_sample(&p,p.duration,&s,&v,&a));
    near(s,200,1e-5f);near(v,80,1e-5f);
    assert(!alg_motion_profile_plan(&p,1,0,100,100,1,1));
    assert(!alg_motion_profile_plan(&p,1,100,0,100,1,1));
    assert(!alg_motion_profile_plan(&p,10,0,101,100,100,100));
    assert(!alg_motion_profile_plan(&p,NAN,0,0,100,100,100));
    assert(!alg_motion_profile_plan(&p,FLT_MAX,0,0,1,1,1));
    assert(!alg_motion_profile_plan(NULL,10,0,0,100,100,100));
}
static void test_geometry_and_continuity(void)
{
    AlgPathSegment_t path[]={LINE(400,250,300,300),ARC(200,PI/2,140,200,250),
                            LINE(350,220,250,300),ARC(200,-PI/2,100,150,150),
                            LINE(250,200,250,250)};
    assert(alg_chassis_path_plan(path,5,&limits,0));
    AlgPathSample_t prev,now;
    assert(alg_chassis_path_get_sample(&prev));
    unsigned seams=0,ticks=0;
    float maximum_deviation=0;
    while(!prev.finished) {
        (void)alg_chassis_path_update(0.001f);
        assert(alg_chassis_path_get_sample(&now));
        assert(now.distance>=prev.distance-1e-3f);
        assert(now.speed<=path[now.segment].max_speed+0.002f);
        assert(fabsf(now.omega)<=limits.max_omega+1e-4f);
        assert(fabsf(now.angular_accel)<=limits.max_angular_accel+0.002f);
        assert(now.speed*now.speed*fabsf(now.curvature)<=limits.max_lateral_accel+0.01f);
        assert(fabsf(now.omega-prev.omega)<=limits.max_angular_accel*0.001f+1e-4f);
        assert(fabsf(now.speed-prev.speed)<=300.0f*0.001f+0.01f);
        if(now.segment!=prev.segment) {
            ++seams; assert(now.speed>1.0f && prev.speed>1.0f);
            assert(fabsf(now.curvature)<1e-4f);
        }
        if(now.segment==1U) {
            float deviation=fabsf(hypotf(now.x+200.0f,now.y-400.0f)-200.0f);
            maximum_deviation=fmaxf(maximum_deviation,deviation);
        }
        prev=now;assert(++ticks<100000U);
    }
    assert(seams==4U);
    near(now.x,-750,0.03f);near(now.y,1050,0.03f);near(now.yaw,0,1e-5f);
    near(now.speed,0,1e-6f);near(now.omega,0,1e-6f);
    assert(maximum_deviation<12.0f);
    printf("PASS: tangent seams stay moving; angular speed/acceleration bounded; maximum R200 departure=%.2f mm\n",maximum_deviation);
}
static void test_limits_and_time_steps(void)
{
    AlgPathSegment_t short_path[ALG_PATH_MAX_SEGMENTS];
    for(unsigned n=0;n<ALG_PATH_MAX_SEGMENTS;++n) short_path[n]=(AlgPathSegment_t)LINE(.02f,100,300,300);
    assert(alg_chassis_path_plan(short_path,ALG_PATH_MAX_SEGMENTS,&limits,0));
    AlgPathSample_t a,b;
    assert(alg_chassis_path_get_sample(&a));
    assert(!alg_chassis_path_update(NAN));
    assert(!alg_chassis_path_update(-1));
    assert(!alg_chassis_path_update(0));
    assert(alg_chassis_path_get_sample(&b));near(a.distance,b.distance,0);
    assert(alg_chassis_path_update(100));
    assert(alg_chassis_path_get_sample(&b));near(b.y,.32f,.00001f);
    assert(!alg_chassis_path_plan(short_path,ALG_PATH_MAX_SEGMENTS+1U,&limits,0));
    assert(!alg_chassis_path_get_sample(&a));
    assert(!alg_chassis_path_plan(NULL,1,&limits,0));
    assert(!alg_chassis_path_plan(short_path,0,&limits,0));
    assert(!alg_chassis_path_plan(short_path,1,&limits,FLT_MAX));
    AlgPathSegment_t bad=ARC(200,PI/2,100,150,150);
    AlgPathLimits_t bad_limits=limits;
    bad_limits.transition_mm=100;
    assert(!alg_chassis_path_plan(&bad,1,&bad_limits,0));
    bad_limits=limits;bad_limits.max_angular_accel=NAN;
    assert(!alg_chassis_path_plan(&bad,1,&bad_limits,0));
    bad.angle=PI+.01f;
    assert(!alg_chassis_path_plan(&bad,1,&limits,0));
    bad=(AlgPathSegment_t)LINE(100,0,100,100);
    assert(!alg_chassis_path_plan(&bad,1,&limits,0));
    /* More than one phase may end in one control tick: preserve the leftover dt. */
    AlgPathSegment_t turns[]={ARC(200,PI/2,140,200,250),ARC(200,-PI/2,140,200,250)};
    assert(alg_chassis_path_plan(turns,2,&limits,.4f));
    assert(alg_chassis_path_get_sample(&a));
    float duration=a.duration;
    for(unsigned n=0;n<100U;++n) (void)alg_chassis_path_update(duration/100.0f);
    (void)alg_chassis_path_update(.001f);
    assert(alg_chassis_path_get_sample(&a) && a.finished);
    near(a.yaw,.4f,1e-5f);
    AlgPathSegment_t full[ALG_PATH_MAX_SEGMENTS];
    for(unsigned n=0;n<ALG_PATH_MAX_SEGMENTS;++n) {
        full[n]=(AlgPathSegment_t)ARC(200,(n%2U)?-PI/2:PI/2,140,200,250);
    }
    assert(alg_chassis_path_plan(full,ALG_PATH_MAX_SEGMENTS,&limits,0));
    assert(alg_chassis_path_update(FLT_MAX));
    assert(alg_chassis_path_get_sample(&a) && a.segment==15U);
    near(a.x,-3200,.02f);near(a.y,3200,.02f);
}
static uint32_t random_state=0x46182431U;
static float random_unit(void)
{
    random_state=random_state*1664525U+1013904223U;
    return (float)(random_state>>8)/16777216.0f;
}
static void test_varied_paths(void)
{
    for(unsigned trial=0;trial<40U;++trial) {
        AlgPathSegment_t path[5];
        AlgPathLimits_t bound=limits;
        double x=0,y=0,heading=.7;
        for(unsigned n=0;n<5U;++n) {
            float v=50+250*random_unit(),a=50+300*random_unit(),d=50+300*random_unit();
            if(n%2U==0U) {
                float distance=20+800*random_unit();
                path[n]=(AlgPathSegment_t)LINE(distance,v,a,d);
                x-=distance*sin(heading);y+=distance*cos(heading);
            } else {
                float r=100+400*random_unit();
                float angle=(.3f+2.8f*random_unit())*(random_unit()<.5f?-1:1);
                path[n]=(AlgPathSegment_t)ARC(r,angle,v,a,d);
                bound.transition_mm=fminf(bound.transition_mm,.15f*r*fabsf(angle));
                double chord=2*r*sin(fabs(angle)/2);
                x-=chord*sin(heading+angle/2);y+=chord*cos(heading+angle/2);
                heading+=angle;
            }
        }
        assert(alg_chassis_path_plan(path,5,&bound,.7f));
        AlgPathSample_t prev,now;
        assert(alg_chassis_path_get_sample(&prev));
        unsigned iterations=0;
        do {
            (void)alg_chassis_path_update(.01f);
            assert(alg_chassis_path_get_sample(&now));
            assert(isfinite(now.x) && isfinite(now.y) && isfinite(now.speed));
            assert(fabsf(now.angular_accel)<=bound.max_angular_accel+.003f);
            assert(fabsf(now.omega-prev.omega)<=bound.max_angular_accel*.01f+.0002f);
            /* Independent position/velocity consistency check, including seams. */
            float travel=hypotf(now.x-prev.x,now.y-prev.y);
            assert(travel<=.01f*fmaxf(prev.speed,now.speed)+.03f);
            prev=now;assert(++iterations<100000U);
        } while(!now.finished);
        near(now.x,(float)x,.03f);near(now.y,(float)y,.03f);near(now.yaw,(float)heading,1e-5f);
    }
}
static void test_app_execution(void)
{
    reset();
    AlgPathSegment_t path[]={LINE(400,250,300,300),ARC(200,PI/2,140,200,250),LINE(350,220,250,300)};
    assert(app_chassis_motion_path_start(path,3,&limits)==MOTION_START_OK);
    AlgPathSample_t before,after;
    assert(app_chassis_motion_path_get_sample(&before));
    assert(app_chassis_motion_path_start(NULL,0,NULL)==MOTION_START_BUSY);
    assert(app_chassis_motion_path_get_sample(&after));near(after.duration,before.duration,0);
    path[0].distance=99999; /* API copied the caller's data. */
    unsigned ticks=0;
    while(app_chassis_motion_get_state()==MOTION_STATE_RUNNING) { tick(.01f);assert(++ticks<10000U); }
    for(unsigned n=0;n<100U;++n) tick(.01f);
    assert(app_chassis_motion_get_result()==MOTION_RESULT_DONE);
    near(world_x,-550,4);near(world_y,600,4);near(imu_yaw,PI/2,.002f);
    AlgPathLimits_t excessive=limits;excessive.max_omega=2;
    assert(app_chassis_motion_path_start(path,3,&excessive)==MOTION_START_INVALID);
    assert(app_chassis_motion_path_start(path,3,&limits)==MOTION_START_OK);
    tick(.01f);app_chassis_motion_stop();tick(.01f);
    assert(app_chassis_motion_get_result()==MOTION_RESULT_CANCELLED);
    near(vx,0,1e-5f);near(vy,0,1e-5f);
    assert(!app_chassis_motion_path_get_sample(&after));
    assert(app_chassis_motion_path_start(path,3,&limits)==MOTION_START_OK);
    imu_valid=false;tick(.01f);
    assert(app_chassis_motion_get_result()==MOTION_RESULT_IMU_ERROR);
    assert(app_chassis_motion_get_state()==MOTION_STATE_IDLE);
    assert(app_chassis_motion_path_start(path,3,&limits)==MOTION_START_NO_IMU);
}
int main(void)
{
    test_boundary_profiles();test_geometry_and_continuity();test_limits_and_time_steps();test_varied_paths();test_app_execution();
    puts("PASS: continuous path profiles, geometry, limits, boundaries, validation and app lifecycle");
    return 0;
}
