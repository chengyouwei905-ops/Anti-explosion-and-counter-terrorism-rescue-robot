/**
  ******************************************************************************
  * @file    common_def.h
  * @brief   全项目通用的宏与类型定义（不含任何硬件相关代码）
  ******************************************************************************
  */

#ifndef __COMMON_DEF_H
#define __COMMON_DEF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/** @brief 求数组元素个数 */
#define ARRAY_SIZE(a)               (sizeof(a) / sizeof((a)[0]))

/** @brief 消除"参数未使用"告警 */
#define UNUSED(x)                   ((void)(x))

/** @brief 取较小/较大值 */
#define MIN_OF(a, b)                (((a) < (b)) ? (a) : (b))
#define MAX_OF(a, b)                (((a) > (b)) ? (a) : (b))

/** @brief 上下限截断 */
#define CLAMP(x, lo, hi)            (MIN_OF(MAX_OF((x), (lo)), (hi)))

/** @brief 无符号类型按位取反的快捷写法（避免`signed`提升带来的告警） */
#define INV_MSK(r, m)               ((r) = (r) & ~(m))

/** @brief 小端字节序解析/写入工具（协议解析统一走这里，避免结构体对齐问题） */
static inline uint16_t le_u16_get(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static inline void le_u16_put(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
}

static inline int16_t le_i16_get(const uint8_t *p)
{
    return (int16_t)le_u16_get(p);
}

static inline void le_i16_put(uint8_t *p, int16_t v)
{
    le_u16_put(p, (uint16_t)v);
}

/** @brief 统一的返回码，避免各模块各自定义 -1/0/1 的语义 */
typedef enum {
    RET_OK       = 0,   /**< 成功 */
    RET_ERROR    = -1,  /**< 通用错误 */
    RET_BUSY     = -2,  /**< 忙 */
    RET_TIMEOUT  = -3,  /**< 超时 */
    RET_PARAM    = -4,  /**< 参数非法 */
    RET_NOMEM    = -5,  /**< 空间不足 */
    RET_NOTREADY = -6,  /**< 尚未初始化 */
    RET_IDLE     = -7,  /**< 本轮无需处理（如采样间隔未到），**不是错误** */
} ret_code_t;

#ifdef __cplusplus
}
#endif

#endif /* __COMMON_DEF_H */
