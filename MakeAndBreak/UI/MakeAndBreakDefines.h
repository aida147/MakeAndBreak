/*
 * MakeAndBreakDefines.h
 *
 *  Created on: Jul 5, 2013
 *      Author: arm_user
 */

#ifndef MAKEANDBREAKDEFINES_H_
#define MAKEANDBREAKDEFINES_H_

typedef enum{
    MAKE_BREAK_APP_STATE_IDLE   = 0, // Application is waiting until the OCS is done building
    MAKE_BREAK_APP_STATE_DESIGN     // User is placing blocks
} MakeAndBreakAppState;

typedef enum{
    MAKE_BREAK_OCS_STATE_IDLE   = 0, // OCS is waiting the next design
    MAKE_BREAK_OCS_STATE_BUILD       // OCS is building the design
} MakeAndBreakOcsState;

typedef enum{
    KINECT_RGB   = 0,
    OCS_VIEW
} MakeAndBreakImageSource;

#endif /* MAKEANDBREAKDEFINES_H_ */
