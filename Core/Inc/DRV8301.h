#ifndef __DRV8301__
#define __DRV8301__

#include "math.h"
#include "stdint.h"
#include "stdbool.h"
#include "string.h"

#define DRV8301_WIRTE               0
#define DRV8301_READ                1

#define DRV8301_REG_STATUS1_ADDR    0x00
#define DRV8301_REG_STATUS2_ADDR    0x01
#define DRV8301_REG_CONTROL1_ADDR   0x02
#define DRV8301_REG_CONTROL2_ADDR   0x03

#define GATE_CURRENT1_7A            0X0000
#define GATE_CURRENT0_7A            0X0001
#define GATE_CURRENT0_25A           0X0002
#define GATE_RST_NORMAL             0X0000
#define GATE_RST_LATCH              0X0004
#define PWM_MODE_6PWM               0X0000
#define PWM_MODE_3PWM               0X0008
#define OCP_MODE_CURRENTLIMIT       0x0000
#define OCP_MODE_OCLATCHSTDOWN      0X0010
#define OCP_MODE_REPORTONLY         0X0020
#define OCP_MODE_OCDISABLE          0X0030
#define OCTW_MODE_OCOTBOTH          0X0000
#define OCTW_MODE_OTONLY            0X0001
#define OCTW_MODE_OCONLY            0X0002
#define AMP_GAIN_10                 0X0000
#define AMP_GAIN_20                 0X0004
#define AMP_GAIN_40                 0X0008
#define AMP_GAIN_80                 0X000C
#define DC_CAL_CH1_DISABLE          0X0000
#define DC_CAL_CH1_ENABLE           0X0010
#define DC_CAL_CH2_DISABLE          0X0000
#define DC_CAL_CH2_ENABLE           0X0020
#define OC_TOFF_CYCLEBYCYCLE        0X0000
#define OC_TOFF_OFFTIMECTRL         0X0040

typedef struct
{
    uint16_t data   :   11;
    uint16_t addr   :   4;
    uint16_t rw     :   1;
}DRV8301_data_t;

typedef union DRV8301
{
    DRV8301_data_t bits;
    uint16_t data;
}DRV8301_Data_Type_t;


void DRV8301_Transmit(void);

#endif
