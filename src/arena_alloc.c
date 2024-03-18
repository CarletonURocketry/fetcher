#include "arena_alloc.h"
#include <stdint.h>

/**
 * Allocates memory of size `nbytes` on the arena.
 * @param arena The arena to allocate on.
 * @param nbytes The number of bytes to allocate on the arena.
 * @return NULL if there is not enough space in the arena to allocate `nbytes`, a pointer to `nbytes` of memory
 * otherwise.
 */
void *aalloc(arena_t *arena, size_t nbytes) {
    if (nbytes == 0) return NULL;
    if ((uint8_t *)(arena->cur) + nbytes > (uint8_t *)(arena->start) + arena->size) return NULL;
    void *allocated_memory = arena->cur;
    arena->cur = (uint8_t *)(arena->cur) + nbytes;
    return allocated_memory;
}

/**
 * Frees all memory on the arena.
 * @param arena The arena to free memory on.
 */
void afree(arena_t *arena) { arena->cur = arena->start; }
