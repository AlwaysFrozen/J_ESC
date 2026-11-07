#ifndef TEST_OD_GLOBAL_H
#define TEST_OD_GLOBAL_H
#include <stdint.h>
#define BOARD_SUPPORTED_MOTOR_NUMBER 1U
typedef enum { MS_Init, MS_IDLE, MS_Err, MS_Running } Motor_State_t;
typedef struct { int is_enabled; Motor_State_t motor_state; } Motor_Status_t;
typedef struct { Motor_Status_t motor_status[1]; } Global_Status_t;
extern volatile Global_Status_t global_status;
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) { }
static inline void __set_PRIMASK(uint32_t state) { (void)state; }
static inline void __DMB(void) { }
#endif
