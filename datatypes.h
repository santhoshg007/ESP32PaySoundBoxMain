/*
 * Copyright (c) 2025 SoundBoxPay
 * All rights reserved.
 *
 * This software is confidential and proprietary to SoundBoxPay.
 * The use, reproduction, distribution, or disclosure of this software outside
 * of the company premises is strictly prohibited without prior written consent.
 *
 * No part of this software may be copied, modified, or distributed for any
 * purpose other than as expressly permitted by Your Company Name.
 *
 * File Name        : datatypes.h
 * File Description : datatypes
 * Author           : SanthoshG
 * Date             : 04Mar25
 * 
 * =========
 * History :
 * =========
 * Date         Author              Created/Modified Details 
 * -----        -------             ------------------------
 * 04Mar25      SanthoshG           Create the Display Manager Handling.           
 */

#ifndef __DATATYPES_H__
#define __DATATYPES_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*=====================================Include====================================================() */

/*=====================================Macro======================================================() */

/*=====================================typedef=====================================================() */
// Integer Types (Signed)
typedef signed char INT8;
typedef signed short int INT16;
typedef signed long int INT32;
typedef signed long long int INT64;

// Integer Types (Unsigned)
typedef unsigned char UINT8;
typedef unsigned short int UINT16;
typedef unsigned long int UINT32;
typedef unsigned long long int UINT64;

// Floating-Point Types
typedef float FLOAT32;
typedef double FLOAT64;

// Boolean Type (using a simple integer representation)
typedef unsigned char BOOL;
#define TRUE 1
#define FALSE 0

// Character Type
typedef char CHAR;


#define PSB_FALSE 0
#define PSB_TRUE  1


#define ENABLE_DEBUG_PRINT
#define ENABLE_INFO_PRINT
#define ENABLE_ERROR_PRINT

#ifdef ENABLE_DEBUG_PRINT
    #define PSB_DEBUG_PRINT         printf ( __XX__ )
#else
     #define PSB_DEBUG_PRINT        /* NOP */
#endif /* ENABLE_DEBUG_PRINT */

#ifdef ENABLE_INFO_PRINT
    #define PSB_INFO_PRINT         printf ( __XX__ )
#else
     #define PSB_INFO_PRINT        /* NOP */
#endif /* ENABLE_INFO_PRINT */

#ifdef ENABLE_ERROR_PRINT
    #define PSB_ERROR_PRINT         printf ( __XX__ )
#else
     #define PSB_ERROR_PRINT        /* NOP */
#endif /* ENABLE_ERROR_PRINT */

/*=====================================Global Variable=============================================() */



/*=====================================Private Variable============================================() */

/*=====================================Function Prototype==========================================() */

/*=====================================Function Defination=========================================() */




#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __DATATYPES_H__ */