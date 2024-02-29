#ifndef _ARENA_ALLOC_H_
#define _ARENA_ALLOC_H_

#include <stdlib.h>

/** Represents an arena of memory that can be allocated from. */
typedef struct {
    /** The start of the arena backing memory. */
    void *start;
    /** The current position within the backing memory that is not allocated. */
    void *cur;
    /** The size of the arena's backing memory in bytes. */
    size_t size;
} arena_t;

void *aalloc(arena_t *arena, size_t nbytes);
void afree(arena_t *arena);

#endif // _ARENA_ALLOC_H_
