/**
  ******************************************************************************
  * @file    ring_buffer.c
  * @brief   通用环形缓冲区实现
  ******************************************************************************
  */

#include "ring_buffer.h"
#include <stddef.h>

static bool rb_size_is_valid(uint16_t size)
{
    return (size >= 2U) && ((size & (uint16_t)(size - 1U)) == 0U);
}

void rb_init(ring_buffer_t *rb, uint8_t *storage, uint16_t size)
{
    if (rb == NULL) {
        return;
    }
    if ((storage == NULL) || (!rb_size_is_valid(size))) {
        /* 参数非法时退化为"容量 0"的空缓冲区，后续操作都会安全地返回 0 */
        rb->buf  = storage;
        rb->size = 0U;
        rb->mask = 0U;
    } else {
        rb->buf  = storage;
        rb->size = size;
        rb->mask = (uint16_t)(size - 1U);
    }
    rb->head = 0U;
    rb->tail = 0U;
}

void rb_reset(ring_buffer_t *rb)
{
    if (rb == NULL) {
        return;
    }
    rb->head = 0U;
    rb->tail = 0U;
}

uint16_t rb_write(ring_buffer_t *rb, const uint8_t *data, uint16_t len)
{
    uint16_t written = 0U;

    if ((rb == NULL) || (data == NULL) || (rb->size == 0U)) {
        return 0U;
    }

    while (written < len) {
        uint16_t head = rb->head;
        uint16_t next = (uint16_t)((head + 1U) & rb->mask);

        if (next == rb->tail) {
            break;                      /* 缓冲区已满 */
        }
        rb->buf[head] = data[written];
        rb->head = next;
        written++;
    }

    return written;
}

uint16_t rb_read(ring_buffer_t *rb, uint8_t *data, uint16_t len)
{
    uint16_t read_cnt = 0U;

    if ((rb == NULL) || (data == NULL) || (rb->size == 0U)) {
        return 0U;
    }

    while (read_cnt < len) {
        if (rb->tail == rb->head) {
            break;                      /* 缓冲区已空 */
        }
        data[read_cnt] = rb->buf[rb->tail];
        rb->tail = (uint16_t)((rb->tail + 1U) & rb->mask);
        read_cnt++;
    }

    return read_cnt;
}

uint16_t rb_peek(const ring_buffer_t *rb, uint8_t *data, uint16_t len)
{
    uint16_t idx = 0U;

    if ((rb == NULL) || (data == NULL) || (rb->size == 0U)) {
        return 0U;
    }

    while ((idx < len) && (((uint16_t)(rb->tail + idx) & rb->mask) != rb->head)) {
        data[idx] = rb->buf[(uint16_t)(rb->tail + idx) & rb->mask];
        idx++;
    }

    return idx;
}

void rb_skip(ring_buffer_t *rb, uint16_t n)
{
    if ((rb == NULL) || (rb->size == 0U)) {
        return;
    }
    for (uint16_t i = 0U; i < n; i++) {
        if (rb->tail == rb->head) {
            break;
        }
        rb->tail = (uint16_t)((rb->tail + 1U) & rb->mask);
    }
}

uint16_t rb_used(const ring_buffer_t *rb)
{
    if ((rb == NULL) || (rb->size == 0U)) {
        return 0U;
    }
    return (uint16_t)((uint16_t)(rb->head - rb->tail) & rb->mask);
}

uint16_t rb_free(const ring_buffer_t *rb)
{
    if ((rb == NULL) || (rb->size == 0U)) {
        return 0U;
    }
    return (uint16_t)((uint16_t)(rb->mask - rb_used(rb)));
}

bool rb_is_empty(const ring_buffer_t *rb)
{
    return (rb_used(rb) == 0U);
}

bool rb_is_full(const ring_buffer_t *rb)
{
    if ((rb == NULL) || (rb->size == 0U)) {
        return false;
    }
    return (rb_free(rb) == 0U);
}

int rb_getc(ring_buffer_t *rb)
{
    uint8_t byte;

    if ((rb == NULL) || (rb->size == 0U) || (rb->tail == rb->head)) {
        return -1;
    }
    byte = rb->buf[rb->tail];
    rb->tail = (uint16_t)((rb->tail + 1U) & rb->mask);
    return (int)byte;
}

bool rb_putc(ring_buffer_t *rb, uint8_t byte)
{
    uint16_t next;

    if ((rb == NULL) || (rb->size == 0U)) {
        return false;
    }
    next = (uint16_t)((rb->head + 1U) & rb->mask);
    if (next == rb->tail) {
        return false;                   /* 满 */
    }
    rb->buf[rb->head] = byte;
    rb->head = next;
    return true;
}
