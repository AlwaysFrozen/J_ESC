#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__



#define HAVE_BUS_CURRENT_SENSOR             0

#define HAVE_3_PHASE_CURRENT_SENSOR         1
#define HAVE_2_PHASE_CURRENT_SENSOR         0

#if HAVE_3_PHASE_CURRENT_SENSOR && HAVE_2_PHASE_CURRENT_SENSOR
#error Phase Current Sensor Set Incorrect
#endif


#endif
