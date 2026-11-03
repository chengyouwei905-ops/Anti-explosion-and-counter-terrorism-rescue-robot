/* Real application, packet encoder and FK/IK; only UART is replaced. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "app_roboarm.h"
#include "bsp_uart.h"

static unsigned sends;
static uint8_t frame[64];
static int tx_result = RET_OK;
int bsp_uart_attach(bsp_uart_id_t id, UART_HandleTypeDef *u)
{ (void)id; (void)u; return RET_OK; }
int bsp_uart_send(bsp_uart_id_t id, const uint8_t *p, uint16_t n, uint32_t timeout)
{
    (void)timeout;
    assert(id == BSP_UART_ROBOARM && n <= sizeof(frame));
    if (tx_result == RET_OK) { memcpy(frame, p, n); ++sends; }
    return tx_result;
}
int bsp_uart_send_it(bsp_uart_id_t id, const uint8_t *p, uint16_t n)
{ return bsp_uart_send(id, p, n, 0); }
static unsigned position(unsigned i)
{ return frame[8 + 3*i] + 256U * frame[9 + 3*i]; }
static void close_to(double a, double b)
{ assert(fabs(a-b) < 1e-6); }

int main(void)
{
    /* Subsets and reordering use servo ID, not array index. */
    Roboarm_Servo_t servo = {JOINT3, 0};
    assert(app_roboarm_servo_move_direct(&servo, 1, 1000) == RET_OK);
    assert(frame[7] == 4 && position(0) == 443);
    const Roboarm_Servo_t reordered[] = {{JOINT3,0}, {YAW_SERVO,0},
        {JOINT2,1.5707963267948966}, {JOINT1,0}};
    assert(app_roboarm_servo_move_direct(reordered, 4, 1000) == RET_OK);
    assert(position(0)==443 && position(1)==480 && position(2)==960 && position(3)==766);

    unsigned before = sends;
    assert(app_roboarm_servo_move_direct(NULL,1,0)==RET_PARAM);
    assert(app_roboarm_servo_move_direct(&servo,0,0)==RET_PARAM);
    assert(app_roboarm_servo_move_direct(&servo,6,0)==RET_PARAM);
    servo.servoname = (Roboarm_Servo_ID_t)99;
    assert(app_roboarm_servo_move_direct(&servo,1,0)==RET_PARAM);
    servo.servoname = JOINT3; servo.angle = NAN;
    assert(app_roboarm_servo_move_direct(&servo,1,0)==RET_PARAM);
    servo.angle = INFINITY;
    assert(app_roboarm_servo_move_direct(&servo,1,0)==RET_PARAM);
    servo.angle = 1e200;
    assert(app_roboarm_servo_move_direct(&servo,1,0)==RET_PARAM);
    const Roboarm_Servo_t duplicate[] = {{JOINT3,0},{JOINT3,0}};
    assert(app_roboarm_servo_move_direct(duplicate,2,0)==RET_PARAM);
    assert(app_roboarm_servo_move_to(NAN,0,0,0)==RET_PARAM);
    assert(app_roboarm_servo_move_to(1e300,1e300,1e300,0)!=RET_OK);
    assert(sends==before);

    app_roboarm_clamp_open(); assert(frame[7]==5 && position(0)==437);
    app_roboarm_clamp_close(); assert(frame[7]==5 && position(0)==747);

    /* FK/IK agree on output yaw and keep move_to's XYZ/radian signature. */
    const double output_yaws[] = {-.3, 0, .5, 1.4};
    for (unsigned i=0; i<4; ++i) {
        Roboarm_State_t j = {output_yaws[i] / ROBOARM_YAW_OUTPUT_PER_SERVO,.2,1.5,.1};
        End_Effector_Pos_t p;
        assert(alg_roboarm_forward(&j,&p)==ROBOARM_KINEMATICS_OK);
        close_to(atan2(p.x,p.y),output_yaws[i]);
        Roboarm_State_t recovered;
        assert(alg_roboarm_invert(&p,&recovered)==ROBOARM_KINEMATICS_OK);
        close_to(j.yaw_angle,recovered.yaw_angle);
        close_to(j.joint1_angle,recovered.joint1_angle);
        close_to(j.joint2_angle,recovered.joint2_angle);
        close_to(j.joint3_angle,recovered.joint3_angle);
        assert(app_roboarm_servo_move_to(p.x,p.y,p.z,p.pitch)==RET_OK);
    }
    assert(alg_roboarm_forward(NULL,NULL)==ROBOARM_KINEMATICS_ERR);
    assert(alg_roboarm_invert(NULL,NULL)==ROBOARM_KINEMATICS_ERR);

    /* Newly allowed yaw, and both sides of the shared geometric limit. */
    const double radians = 3.14159265358979323846 / 180.0;
    Roboarm_State_t edge = {99.9*radians/ROBOARM_YAW_OUTPUT_PER_SERVO,.2,1.5,.1};
    End_Effector_Pos_t edge_pose;
    Roboarm_State_t edge_inverse;
    assert(alg_roboarm_forward(&edge,&edge_pose)==ROBOARM_KINEMATICS_OK);
    assert(alg_roboarm_invert(&edge_pose,&edge_inverse)==ROBOARM_KINEMATICS_OK);
    close_to(edge_inverse.yaw_angle,edge.yaw_angle);
    /* Geometric validity must not bypass physical 0..1000 position checks. */
    before=sends;
    assert(app_roboarm_servo_move_to(edge_pose.x,edge_pose.y,edge_pose.z,edge_pose.pitch)!=RET_OK);
    assert(sends==before);
    double radius=hypot(edge_pose.x,edge_pose.y);
    edge.yaw_angle=-91.63636363636364*radians/ROBOARM_YAW_OUTPUT_PER_SERVO;
    assert(alg_roboarm_forward(&edge,&edge_pose)==ROBOARM_KINEMATICS_OK);
    assert(alg_roboarm_invert(&edge_pose,&edge_inverse)==ROBOARM_KINEMATICS_OK);
    close_to(edge.yaw_angle,edge_inverse.yaw_angle);
    assert(app_roboarm_servo_move_to(edge_pose.x,edge_pose.y,edge_pose.z,edge_pose.pitch)==RET_OK);
    edge.yaw_angle=-99.9*radians/ROBOARM_YAW_OUTPUT_PER_SERVO;
    assert(alg_roboarm_forward(&edge,&edge_pose)==ROBOARM_KINEMATICS_OK);
    assert(alg_roboarm_invert(&edge_pose,&edge_inverse)==ROBOARM_KINEMATICS_OK);
    close_to(edge.yaw_angle,edge_inverse.yaw_angle);
    const double outside[] = {100.1,-100.1};
    for(unsigned i=0;i<2;++i) {
        edge.yaw_angle=outside[i]*radians/ROBOARM_YAW_OUTPUT_PER_SERVO;
        assert(alg_roboarm_forward(&edge,&edge_pose)==ROBOARM_KINEMATICS_ERR);
        edge_pose.x=radius*sin(outside[i]*radians);
        edge_pose.y=radius*cos(outside[i]*radians);
        assert(alg_roboarm_invert(&edge_pose,&edge_inverse)==ROBOARM_KINEMATICS_ERR);
    }
    /* Corrected QR raw pose now satisfies the existing joint limits. */
    Roboarm_State_t qr={0,-11*.24*radians,435*.24*radians,-8*.24*radians};
    assert(alg_roboarm_forward(&qr,&edge_pose)==ROBOARM_KINEMATICS_OK);
    assert(app_roboarm_pose_it(ROBOARM_POSE_QR_OBSERVE,3000)==RET_OK);
    assert(position(0)==480 && position(1)==755 && position(2)==900 && position(3)==435);

    for (unsigned i=0;i<ROBOARM_POSE_COUNT;++i) {
        const uint16_t *p=app_roboarm_pose_positions((Roboarm_Pose_t)i);
        Roboarm_State_t measured = {
            (YAW_SERVO_OFFSET-(double)p[0])*.24*radians,
            ((double)p[1]-JOINT1_OFFSET)*.24*radians,
            (JOINT2_OFFSET-(double)p[2])*.24*radians,
            ((double)p[3]-JOINT3_OFFSET)*.24*radians
        };
        End_Effector_Pos_t measured_pose;
        Roboarm_State_t recovered;
        assert(alg_roboarm_forward(&measured,&measured_pose)==ROBOARM_KINEMATICS_OK);
        assert(alg_roboarm_invert(&measured_pose,&recovered)==ROBOARM_KINEMATICS_OK);
        close_to(recovered.yaw_angle,measured.yaw_angle);
        close_to(recovered.joint1_angle,measured.joint1_angle);
        close_to(recovered.joint2_angle,measured.joint2_angle);
        close_to(recovered.joint3_angle,measured.joint3_angle);
        assert(app_roboarm_servo_move_to(measured_pose.x,measured_pose.y,
                                         measured_pose.z,measured_pose.pitch)==RET_OK);
        for(unsigned j=0;j<4;++j) {
            assert(frame[7+3*j]==j+1 && position(j)==p[j]);
        }
        assert(app_roboarm_pose_it((Roboarm_Pose_t)i,3000)==RET_OK);
        for(unsigned j=0;j<4;++j) {
            assert(frame[7+3*j]==j+1 && position(j)==p[j]);
        }
    }

    /* Published rounded XYZ/pitch values must survive the full command path,
     * including poses whose yaw position is exactly zero.
     */
    const End_Effector_Pos_t rounded_poses[] = {
        {0.000000,204.662676,99.824932,-0.217817091},
        {-225.141372,-6.431771,28.898666,-0.196873140},
        {222.130620,-6.345761,-41.874941,-1.055575132},
        {190.624889,-5.445715,20.177960,-0.594808209},
        {154.215426,-4.405580,-4.916010,-0.950855376},
        {250.683197,-7.161443,14.658957,-0.234572251},
        {186.472176,-5.327081,29.339224,-0.900589894},
        {154.215426,-4.405580176,-4.916010291,-0.950855376}
    };
    const unsigned expected_positions[][4] = {
        {480,755,900,435}, {960,920,780,310}, {0,985,800,535},
        {0,800,800,425}, {0,735,800,510}, {0,1000,800,339},
        {0,770,865,563}, {0,735,800,510}
    };
    for(unsigned i=0;i<sizeof(rounded_poses)/sizeof(rounded_poses[0]);++i) {
        const End_Effector_Pos_t *p=&rounded_poses[i];
        assert(app_roboarm_servo_move_to(p->x,p->y,p->z,p->pitch)==RET_OK);
        for(unsigned j=0;j<4;++j) {
            assert(frame[7+3*j]==j+1 && position(j)==expected_positions[i][j]);
        }
    }

    /* Numerical tolerance at both endpoints must not admit real overtravel. */
    const double accepted_positions[] = {0,1000,-0.00005,1000.00005,734.6};
    const unsigned quantized_positions[] = {0,1000,0,1000,735};
    servo.servoname=JOINT3;
    for(unsigned i=0;i<sizeof(accepted_positions)/sizeof(accepted_positions[0]);++i) {
        servo.angle=(accepted_positions[i]-JOINT3_OFFSET)*.24*radians;
        assert(app_roboarm_servo_move_direct(&servo,1,3000)==RET_OK);
        assert(position(0)==quantized_positions[i]);
    }
    before=sends;
    const double rejected_positions[] = {-0.001,1000.001,-0.49,1000.49};
    for(unsigned i=0;i<sizeof(rejected_positions)/sizeof(rejected_positions[0]);++i) {
        servo.angle=(rejected_positions[i]-JOINT3_OFFSET)*.24*radians;
        assert(app_roboarm_servo_move_direct(&servo,1,3000)==RET_PARAM);
    }
    assert(sends==before);
    before=sends;
    tx_result=RET_BUSY;
    assert(app_roboarm_pose_it(ROBOARM_POSE_QR_OBSERVE,3000)==RET_BUSY);
    tx_result=RET_ERROR;
    assert(app_roboarm_pose_it(ROBOARM_POSE_QR_OBSERVE,3000)==RET_ERROR);
    tx_result=RET_OK;
    assert(app_roboarm_pose_it(ROBOARM_POSE_COUNT,3000)==RET_PARAM);
    const proto_roboarm_servo_t invalid[]={{1,1001}};
    assert(app_roboarm_positions_it(invalid,1,3000)==RET_PARAM);
    assert(sends==before);
    puts("PASS: arm offsets, guards, clamp, 35:44 FK/IK, rounded pose commands, endpoint tolerance and raw pose IT");
    return 0;
}
