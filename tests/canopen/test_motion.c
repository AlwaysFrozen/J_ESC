#include "od_motion.h"
#include "od_wire.h"
#include "global.h"

#define CHECK(x) do{if(!(x))return __LINE__;}while(0)
volatile Global_Status_t global_status;
Global_Configs_t global_config;
Motion_Planner_t motion_planner[1];
uint32_t test_ipsr,test_primask;
static int plans,disables;
static bool config_pending;
static bool cancel_during_plan;
bool od_commands_config_pending(void){return config_pending;}
float axis_units_to_motor_units(int index,float value){(void)index;return value;}
bool motor_disable(int index){(void)index;++disables;global_status.motor_status[0].is_enabled=0;global_status.motor_status[0].motor_ctrl_mode=MCM_None;return true;}
bool motion_plan(Motion_Planner_t *p,float a,float b,float c,float d,float e,float f,int g){(void)a;(void)b;(void)c;(void)d;(void)e;(void)f;(void)g;++plans;p->total_t=1;return true;}
bool velocity_plan(Motion_Planner_t *p,float a,float b,float c,float d,float e,int f){(void)a;(void)b;(void)c;(void)d;(void)e;(void)f;++plans;p->total_t=1;
    if(cancel_during_plan){od_motion_cancel();od_motion_tick(false);}return true;}
#ifdef OD_TEST_NO_CRT
int _fltused;
void *memcpy(void *d,const void *s,size_t n){unsigned char *a=d;const unsigned char *b=s;for(size_t i=0;i<n;++i)a[i]=b[i];return d;}
void *memset(void *d,int c,size_t n){unsigned char *a=d;for(size_t i=0;i<n;++i)a[i]=(unsigned char)c;return d;}
#endif
static void f32(uint8_t *p,float f){union{float f;uint32_t u;}v={.f=f};od_wire_put32(p,v.u);}
int main(void)
{
    uint8_t command[20]={1},status[16];
    CHECK(od_motion_submit(command,20)==OD_BUSY);
    volatile Motor_Status_t *m=&global_status.motor_status[0];
    m->is_enabled=1;m->is_calibrated=1;m->is_aligned=1;m->motor_state=MS_Running;
    m->negative_current_limit=-5;m->positive_current_limit=5;m->negative_vel_limit=-100;m->positive_vel_limit=100;
    f32(command+4,2);CHECK(od_motion_submit(command,20)==OD_OK);CHECK(m->motor_ctrl_mode==MCM_None);
    od_motion_tick(false);CHECK(m->motor_ctrl_mode==MCM_None);
    CHECK(od_motion_submit(command,20)==OD_BUSY);od_motion_tick(true);CHECK(m->motor_ctrl_mode==MCM_Current && m->current_ref==2);
    for(unsigned i=0;i<BOARD_HRT_FREQ/2-1;++i)od_motion_tick(true);CHECK(disables==0);
    od_motion_heartbeat();od_motion_tick(true);CHECK(disables==0);
    od_motion_cancel();od_motion_tick(true);CHECK(disables==1 && !m->is_enabled);
    m->is_enabled=1;f32(command+4,6);CHECK(od_motion_submit(command,20)==OD_VALUE_INVALID);
    command[0]=2;f32(command+4,20);f32(command+12,10);f32(command+16,100);
    CHECK(od_motion_submit(command,20)==OD_OK);CHECK(plans==1);od_motion_tick(true);CHECK(plans==1 && m->motor_ctrl_mode==MCM_Speed);
    for(unsigned i=0;i<BOARD_HRT_FREQ/2+1;++i)od_motion_tick(true);CHECK(disables==2 && !m->is_enabled);
    od_motion_status(status);CHECK(status[4]==5);
    m->is_enabled=1;config_pending=true;CHECK(od_motion_submit(command,20)==OD_BUSY);config_pending=false;
    CHECK(od_motion_submit(command,20)==OD_OK);m->is_error=1;od_motion_tick(true);od_motion_status(status);CHECK(status[4]==4);
    CHECK(m->motor_ctrl_mode==MCM_None);
    command[0]=3;m->is_error=0;f32(command+8,10);CHECK(od_motion_submit(command,20)==OD_OK);
    od_motion_cancel();od_motion_tick(true);CHECK(m->motor_ctrl_mode==MCM_None);
    CHECK(od_motion_submit(command,20)==OD_OK);od_motion_tick(true);CHECK(m->motor_ctrl_mode==MCM_Pos);
    m->is_arrived=1;od_motion_tick(true);od_motion_status(status);CHECK(status[4]==3);
    od_motion_cancel();od_motion_tick(true);CHECK(disables==3);
    m->is_enabled=1;od_wire_put32(command+4,0x7FC00000U);CHECK(od_motion_submit(command,20)==OD_VALUE_INVALID);
    command[0]=2;f32(command+4,20);f32(command+8,0);cancel_during_plan=true;
    CHECK(od_motion_submit(command,20)==OD_BUSY);od_motion_tick(true);CHECK(m->motor_ctrl_mode==MCM_None);
    return 0;
}


