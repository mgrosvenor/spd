/* Minimal stdint.h for bare-metal cross-compilation of sdp.c */
#ifndef CROSS_STDINT_H
#define CROSS_STDINT_H

typedef unsigned char      uint8_t;
typedef unsigned short     uint16_t;
typedef unsigned int       uint32_t;
typedef signed char        int8_t;
typedef signed short       int16_t;
typedef signed int         int32_t;

#endif /* CROSS_STDINT_H */
