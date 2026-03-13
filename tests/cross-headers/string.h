/* Minimal string.h for bare-metal cross-compilation of sdp.c */
#ifndef CROSS_STRING_H
#define CROSS_STRING_H

#ifndef NULL
#  define NULL ((void *)0)
#endif

typedef unsigned int size_t;

void *memset(void *s, int c, size_t n);
void *memcpy(void *dst, const void *src, size_t n);

#endif /* CROSS_STRING_H */
