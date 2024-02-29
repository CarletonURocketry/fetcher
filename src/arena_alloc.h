#ifndef _ARENA_ALLOC_H_
#define _ARENA_ALLOC_H_

#include <stdlib.h>

typedef struct {
    void *start;
    void *cur;
    size_t size;
} arena_t;

void *aalloc(arena_t *arena, size_t nbytes);
void afree(arena_t *arena);

#endif // _ARENA_ALLOC_H_
