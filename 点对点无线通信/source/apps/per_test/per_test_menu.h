/***********************************************************************************
    Filename:     per_test_menu.h

    Description:  PER test menu header file

***********************************************************************************/

#ifndef PER_TEST_MENU_H
#define PER_TEST_MENU_H

/***********************************************************************************
* INCLUDES
*/
#include "hal_types.h"
#include "hal_defs.h"
#include "hal_board.h"

/***********************************************************************************
* CONSTANTS AND DEFINES
*/
// Application modes
#define MODE_TX                   0
#define MODE_RX                   1
#define MODES                     2

// Burst Sizes
#define BURST_SIZE_1            1000
#define BURST_SIZE_2            10000
#define BURST_SIZE_3            100000
#define BURST_SIZE_4            1000000
#define BURSTSIZES              4

// Channel 11, first channel in band
#define CHANNEL_11              11
#define CHANNELS                16

/***********************************************************************************
* GLOBAL FUNCTIONS
*/
uint8 appSelectMode(void);
uint8 appSelectChannel(void);
uint8 appSelectOutputPower(void);
uint8 appSelectGain(void);          // Modules with CC2590/91 only
uint8 appSelectRate(void);
uint32 appSelectBurstSize(void);

#endif