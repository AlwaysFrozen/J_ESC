#ifndef __BLDC_CONFIG_H__
#define __BLDC_CONFIG_H__

#define VIRTUAL_MID_CAL_BY_3PHASE_AVERAGE               0
#define VIRTUAL_MID_CAL_BY_HALF_PHASE                   1
#define VIRTUAL_MID_CAL_BY_BUS_DUTY                     0
#if (VIRTUAL_MID_CAL_BY_3PHASE_AVERAGE + VIRTUAL_MID_CAL_BY_HALF_PHASE + VIRTUAL_MID_CAL_BY_BUS_DUTY) != 1
#error Virtual Mid Cal Error
#endif

#define ANTI_WIND_STARTUP_ENABLE                        1

#endif
