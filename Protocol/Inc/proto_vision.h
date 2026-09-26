/**
  ******************************************************************************
  * @file    proto_vision.h
  * @brief   视觉链路帧协议：打包 / 流式解包 / 校验
  * @note    本层只关心"一帧字节流"，不知道 payload 里装的是坐标还是别的，
  *          业务含义由 App 层（app_vision.c）解释，便于更换视觉端协议。
  ******************************************************************************
  */

#ifndef __PROTO_VISION_H
#define __PROTO_VISION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ==========================================================================
 * 帧格式（默认定义 —— 请与视觉端对齐后同步修改这里，其它文件不用动）
 *
 *   +------+------+-----+-----+-----+---------------+------+
 *   | 0xAA | 0x55 | LEN | SEQ | CMD |    PAYLOAD    | CRC8 |
 *   +------+------+-----+-----+-----+---------------+------+
 *     LEN  : PAYLOAD 字节数        (0 ~ PROTO_VISION_MAX_PAYLOAD)
 *     SEQ  : 帧序号，用于丢帧检测
 *     CMD  : 命令字，决定 PAYLOAD 含义
 *     CRC8 : 对 LEN/SEQ/CMD/PAYLOAD 计算，CRC-8/MAXIM（poly 0x8C 反射, init 0x00）
 * ========================================================================== */

#define PROTO_VISION_SOF1           0xAAU   /**< 帧头第 1 字节 */
#define PROTO_VISION_SOF2           0x55U   /**< 帧头第 2 字节 */
#define PROTO_VISION_MAX_PAYLOAD    64U     /**< 单帧最大载荷字节数 */
#define PROTO_VISION_OVERHEAD       6U      /**< SOF1+SOF2+LEN+SEQ+CMD+CRC */

/** @brief 命令字定义 */
typedef enum {
    PROTO_VISION_CMD_TARGET = 0x01U,    /**< 视觉 -> 底盘：目标位姿 */
    PROTO_VISION_CMD_STATUS = 0x02U,    /**< 视觉 -> 底盘：状态/心跳 */
} proto_vision_cmd_t;

/** @brief 解析出来的一帧 */
typedef struct {
    uint8_t seq;                              /**< 帧序号 */
    uint8_t cmd;                              /**< 命令字 */
    uint8_t len;                              /**< payload 有效长度 */
    uint8_t payload[PROTO_VISION_MAX_PAYLOAD];/**< 载荷 */
} proto_vision_frame_t;

/** @brief 运行统计，调试/上位机观测用 */
typedef struct {
    uint32_t rx_bytes;      /**< 累计接收字节数 */
    uint32_t ok_frames;     /**< 校验通过帧数 */
    uint32_t crc_errors;    /**< CRC 错误帧数 */
    uint32_t len_errors;    /**< 长度非法帧数 */
    uint32_t buf_errors;    /**< 环形缓冲溢出（丢字节）次数 */
} proto_vision_stat_t;

/** @brief 收到完整合法帧时的回调（在主循环上下文被调用，可做耗时操作） */
typedef void (*proto_vision_frame_cb_t)(const proto_vision_frame_t *frame);

/** @brief 初始化解析器（清空状态机与统计） */
void    proto_vision_init(void);

/** @brief 注册整帧回调 */
void    proto_vision_set_cb(proto_vision_frame_cb_t cb);

/**
  * @brief  向解析器喂入原始字节流（支持跨包/半帧，内部有状态机）
  * @param  data 原始字节
  * @param  len  字节数
  */
void    proto_vision_feed(const uint8_t *data, uint16_t len);

/**
  * @brief  打包一帧
  * @param  cmd      命令字
  * @param  payload  载荷，可为 NULL（len 为 0 时）
  * @param  len      载荷字节数
  * @param  out      输出缓冲区
  * @param  out_size 输出缓冲区大小
  * @retval 整帧字节数；0 表示失败（参数非法或缓冲区不足）
  */
uint8_t proto_vision_pack(uint8_t cmd, const uint8_t *payload, uint8_t len,
                          uint8_t *out, uint8_t out_size);

/** @brief 计算 CRC8（供上位机/单测对齐用） */
uint8_t proto_vision_crc8(const uint8_t *data, uint16_t len);

/** @brief 获取统计信息（只读） */
const proto_vision_stat_t *proto_vision_get_stat(void);

/** @brief 记录一次接收缓冲溢出（由上层在检测到溢出时上报） */
void    proto_vision_notify_buf_error(void);

#ifdef __cplusplus
}
#endif

#endif /* __PROTO_VISION_H */
