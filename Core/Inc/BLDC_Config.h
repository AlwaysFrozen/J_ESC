#ifndef __BLDC_CONFIG_H__
#define __BLDC_CONFIG_H__

#define VIRTUAL_MID_3PHASE_AVERAGE                      1
#define VIRTUAL_MID_HALF_PHASE                          0
#define VIRTUAL_MID_HALF_MAX_PHASE                      0
#if (VIRTUAL_MID_3PHASE_AVERAGE + VIRTUAL_MID_HALF_PHASE + VIRTUAL_MID_HALF_MAX_PHASE) != 1
#error Virtual Mid Cal Error
#endif

#define ANTI_WIND_STARTUP_ENABLE                        1

#endif
