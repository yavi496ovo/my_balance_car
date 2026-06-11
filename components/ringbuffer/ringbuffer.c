#include "ringbuffer.h"

static uint16_t ringbuffer_next_index(uint16_t index, uint16_t size)
{
    index++;
    if (index >= size) {
        index = 0U;
    }

    return index;
}

void ringbuffer_init(ringbuffer_t *rb, uint8_t *buf, uint16_t size)
{
    rb->buf = buf;
    rb->size = size;
    rb->head = 0U;
    rb->tail = 0U;
}

void ringbuffer_clear(ringbuffer_t *rb)
{
    rb->head = 0U;
    rb->tail = 0U;
}

uint8_t ringbuffer_is_empty(const ringbuffer_t *rb)
{
    return (rb->head == rb->tail) ? 1U : 0U;
}

uint8_t ringbuffer_is_full(const ringbuffer_t *rb)
{
    uint16_t next_head;

    next_head = ringbuffer_next_index(rb->head, rb->size);
    return (next_head == rb->tail) ? 1U : 0U;
}

uint8_t ringbuffer_write(ringbuffer_t *rb, uint8_t data)
{
    if (ringbuffer_is_full(rb) != 0U) {
        return 0U;
    }

    rb->buf[rb->head] = data;
    rb->head = ringbuffer_next_index(rb->head, rb->size);

    return 1U;
}

uint8_t ringbuffer_read(ringbuffer_t *rb, uint8_t *data)
{
    if (ringbuffer_is_empty(rb) != 0U) {
        return 0U;
    }

    *data = rb->buf[rb->tail];
    rb->tail = ringbuffer_next_index(rb->tail, rb->size);

    return 1U;
}

uint16_t ringbuffer_get_count(const ringbuffer_t *rb)
{
    if (rb->head >= rb->tail) {
        return (uint16_t)(rb->head - rb->tail);
    }

    return (uint16_t)(rb->size - rb->tail + rb->head);
}

uint16_t ringbuffer_get_free(const ringbuffer_t *rb)
{
    return (uint16_t)(rb->size - ringbuffer_get_count(rb) - 1U);
}
