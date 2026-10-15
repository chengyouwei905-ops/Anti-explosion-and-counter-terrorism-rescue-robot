#ifndef __ALG_PID_H__
#define __ALG_PID_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	float Target;
	float Actual;
	float Out;
	
	float Kp;
	float Ki;
	float Kd;
	
	float Error0;
	float Error1;
	float ErrorInt;
	
	float OutMax;
	float OutMin;
} PID_t;

/* 初始化：清零状态与积分，写入增益和输出限幅 */
void PID_Init(PID_t *p, float kp, float ki, float kd, float out_min, float out_max);

void PID_Update(PID_t *p);

#ifdef __cplusplus
}
#endif

#endif /* __ALG_PID_H__ */
