/**
  ******************************************************************************
  * @file    ring_buffer.h
  * @brief   通用单生产者/单消费者环形缓冲区（无动态内存、无锁）
  * @note    典型用法：中断里写(rb_putc/rb_write)，主循环里读(rb_read/rb_getc)。
  *          容量必须是 2 的幂，实现上用位与代替取模，速度更快。
  *          会牺牲 1 字节空间用于区分"满"和"空"，可用容量 = size - 1。
  ******************************************************************************
  */

#ifndef __RING_BUFFER_H
#define __RING_BUFFER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t          *buf;    /**< 外部提供的存储区 */
    uint16_t          size;   /**< 容量（2 的幂） */
    uint16_t          mask;   /**< size - 1 */
    volatile uint16_t head;   /**< 写索引，仅生产者修改 */
    volatile uint16_t tail;   /**< 读索引，仅消费者修改 */
} ring_buffer_t;

/**
  * @brief  初始化环形缓冲区
  * @param  storage 存储区首地址
  * @param  size    容量，必须为 2 的幂（如 64/128/256/512）
  * @retval 无
  */
void     rb_init(ring_buffer_t *rb, uint8_t *storage, uint16_t size);

/** @brief 清空缓冲区（非线程安全，请在确定无读写并发时调用） */
void     rb_reset(ring_buffer_t *rb);

/** @brief 写入数据，返回实际写入的字节数（空间不足时可能小于 len） */
uint16_t rb_write(ring_buffer_t *rb, const uint8_t *data, uint16_t len);

/** @brief 读出数据并移动读指针，返回实际读出的字节数 */
uint16_t rb_read(ring_buffer_t *rb, uint8_t *data, uint16_t len);

/** @brief 只查看数据不移动读指针，返回实际查看的字节数 */
uint16_t rb_peek(const ring_buffer_t *rb, uint8_t *data, uint16_t len);

/** @brief 丢弃 n 字节（用于解析出错时跳过无效数据） */
void     rb_skip(ring_buffer_t *rb, uint16_t n);

/** @brief 当前已缓存字节数 */
uint16_t rb_used(const ring_buffer_t *rb);

/** @brief 剩余可写字节数 */
uint16_t rb_free(const ring_buffer_t *rb);

bool     rb_is_empty(const ring_buffer_t *rb);
bool     rb_is_full(const ring_buffer_t *rb);

/** @brief 取 1 字节，空则返回 -1 */
int      rb_getc(ring_buffer_t *rb);

/** @brief 存 1 字节，满则返回 false */
bool     rb_putc(ring_buffer_t *rb, uint8_t byte);

#ifdef __cplusplus
}
#endif

#endif /* __RING_BUFFER_H */
