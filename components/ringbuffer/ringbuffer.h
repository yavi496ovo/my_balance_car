#ifndef __RINGBUFFER_H
#define __RINGBUFFER_H

#include <stdint.h>

typedef struct {
    uint8_t *buf;
    uint16_t size;
    volatile uint16_t head;
    volatile uint16_t tail;
} ringbuffer_t;

void ringbuffer_init(ringbuffer_t *rb, uint8_t *buf, uint16_t size);
void ringbuffer_clear(ringbuffer_t *rb);
uint8_t ringbuffer_is_empty(const ringbuffer_t *rb);
uint8_t ringbuffer_is_full(const ringbuffer_t *rb);
uint8_t ringbuffer_write(ringbuffer_t *rb, uint8_t data);
uint8_t ringbuffer_read(ringbuffer_t *rb, uint8_t *data);
uint16_t ringbuffer_get_count(const ringbuffer_t *rb);
uint16_t ringbuffer_get_free(const ringbuffer_t *rb);

#endif
