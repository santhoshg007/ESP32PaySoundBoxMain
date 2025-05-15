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
 * File Name        : SoundBoxProcessMgr.h
 * File Description : SoundBoxProcessMgr configuration
 * Author           : SanthoshG
 * Date             : 05Mar25
 *
 * =========
 * History :
 * =========
 * Date         Author              Created/Modified Details
 * -----        -------             ------------------------
 * 05Mar25      SanthoshG           SoundBoxProcessMgr.h header files
 */

#ifndef __SOUNDBOX_PROCESS_MGR__
#define __SOUNDBOX_PROCESS_MGR__

#if 0 //def __cplusplus
extern "C" {
#endif /* __cplusplus */

/*=====================================Include====================================================() */

/*=====================================Define====================================================() */


/*===================================== GPIO Pin Macro ===========================================() */


/*=====================================Macro======================================================() */

/*=====================================typedef=====================================================() */


/*=====================================Global Variable=============================================() */

/*=====================================Private Variable============================================() */

/*=====================================Function Prototype==========================================() */
String getEncryptionType(wifi_auth_mode_t authMode) ;
bool initDisplay( void );

void InitLTEModule();
char* PSB_GetUniqueIDString( void );
void InitAllmodules( void );
void MainRunLoop();
void drawUIListScreen( char ** ppcMenuIcons, char ** ppcMenuItems, int MaximumListSize );

/*=====================================Function Defination=========================================() */




#if 0 //def __cplusplus
}
#endif /* __cplusplus */
#endif /* __SOUNDBOX_PROCESS_MGR__ */
