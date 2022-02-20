/*! ----------------------------------------------------------------------------
 *  @file   deca_types.h
 *  @brief  Decawave general type definitions
 *
 * @attention
 *
 * Copyright 2013 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */

#ifndef _DECA_TYPES_H_
#define _DECA_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#ifndef USE_USER_TYPES 
#define USE_USER_TYPES 1
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef uint8_t     uint8;
typedef uint16_t    uint16;
typedef uint32_t    uint32;
typedef int8_t      int8;
typedef int16_t     int16;
typedef int32_t     int32;
typedef uint8_t     u8;
typedef uint16_t    u16;
typedef uint32_t    u32;
typedef int8_t      s8;
typedef int16_t     s16;
typedef int32_t     s32;
typedef signed long long int64;
typedef unsigned long long uint64;
#endif
//#ifndef uint8
//#ifndef _DECA_UINT8_
//#define _DECA_UINT8_
//typedef unsigned char uint8;
//#endif
//#endif

//#ifndef uint16
//#ifndef _DECA_UINT16_
//#define _DECA_UINT16_
//typedef unsigned short uint16;
//#endif
//#endif

//#ifndef uint32
//#ifndef _DECA_UINT32_
//#define _DECA_UINT32_
//typedef unsigned long uint32;
//#endif
//#endif

//#ifndef int8
//#ifndef _DECA_INT8_
//#define _DECA_INT8_
//typedef signed char int8;
//#endif
//#endif

//#ifndef int16
//#ifndef _DECA_INT16_
//#define _DECA_INT16_
//typedef signed short int16;
//#endif
//#endif

//#ifndef int32
//#ifndef _DECA_INT32_
//#define _DECA_INT32_
//typedef signed long int32;
//#endif
//#endif

#ifndef NULL
#define NULL ((void *)0UL)
#endif

#ifdef __cplusplus
}
#endif

#endif /* DECA_TYPES_H_ */


