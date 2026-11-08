#include "od_motion.h"
#include "od_wire.h"
#include "od_commands.h"
#include "global.h"
#include "motor_api.h"
#include <stdatomic.h>



_Static_assert(ATOMIC_INT_LOCK_FREE == 2, "motion mailbox must be lock-free");
typedef struct {uint8_t mode;float value,start,speed;Motion_Planner_t plan;bool planning;} Target;
static Target mailbox;
static atomic_uint pending, cancel, heartbeat, generation;
static uint32_t tick, accepted_tick, lease_tick, identifier;
static uint8_t phase, result_code, requested_mode;
static bool owned;

static bool gate(void)
{
    const volatile Motor_Status_t *m=&global_status.motor_status[0];
    return m->is_enabled && m->motor_state==MS_Running && !m->is_error && !m->is_braked &&
        m->is_calibrated && m->is_aligned && !od_commands_config_pending() &&
        (m->motor_ctrl_mode==MCM_None || m->motor_ctrl_mode==MCM_Current ||
         m->motor_ctrl_mode==MCM_Speed || m->motor_ctrl_mode==MCM_Pos);
}
static bool finite32(float f){union{float f;uint32_t u;}v={.f=f};return (v.u & 0x7F800000U)!=0x7F800000U;}
static float get_float(const uint8_t *p){union{uint32_t u;float f;}v={.u=od_wire_u32(p)};return v.f;}
OD_Result od_motion_submit(const uint8_t *p,size_t length)
{
    if(length!=20)return OD_LENGTH_MISMATCH;
    if(p[0]<1 || p[0]>3 || p[1] || p[2] || p[3])return OD_VALUE_INVALID;
    if(global_config.motor_config[0].motor_type!=Motor_Type_DC &&
       (global_config.motor_config[0].motor_type!=Motor_Type_BLDC_3_Phase ||
        global_config.motor_controller_config[0].motor_controller_type!=Motor_Controller_Type_FOC))return OD_UNSUPPORTED;
    if(atomic_load_explicit(&pending,memory_order_acquire))return OD_BUSY;
    if(atomic_load_explicit(&cancel,memory_order_acquire))return OD_BUSY;
    unsigned epoch=atomic_load_explicit(&generation,memory_order_acquire);
    float v[4];for(unsigned i=0;i<4;++i){v[i]=get_float(p+4+4*i);if(!finite32(v[i]))return OD_VALUE_INVALID;}
    uint32_t irq=__get_PRIMASK();__disable_irq();bool allowed=gate();uint32_t prepared_tick=tick;
    float pos=global_status.motor_status[0].pos_fb,vel=global_status.motor_status[0].vel_fb;
    float low=global_status.motor_status[0].negative_vel_limit,high=global_status.motor_status[0].positive_vel_limit;
    float ilow=global_status.motor_status[0].negative_current_limit,ihigh=global_status.motor_status[0].positive_current_limit;
    __set_PRIMASK(irq);
    if(!allowed)return OD_BUSY;
    Target t={0};t.mode=p[0];
    if(t.mode==1) {
        if(!finite32(ilow) || !finite32(ihigh) || ihigh<=0 || ilow>=0 || v[0]<ilow || v[0]>ihigh || v[1]!=0 || v[2]!=0 || v[3]!=0)return OD_VALUE_INVALID;
        t.value=v[0];
    } else {
        if(!finite32(pos) || !finite32(vel) || !finite32(low) || !finite32(high) || high<=0 || low>=0)return OD_CONFIG_INVALID;
        float value=axis_units_to_motor_units(0,v[0]);
        float speed=axis_units_to_motor_units(0,v[1]);
        float acc=axis_units_to_motor_units(0,v[2]),jerk=axis_units_to_motor_units(0,v[3]);
        if(!finite32(value) || !finite32(speed) || !finite32(acc) || !finite32(jerk) || acc<=0 || jerk<=0)return OD_VALUE_INVALID;
        t.value=value;t.start=t.mode==2?vel:pos;t.speed=speed;
        if(t.mode==2) {
            if(v[1]!=0 || value<low || value>high)return OD_VALUE_INVALID;
            if(value!=vel)t.planning=velocity_plan(&t.plan,vel,value,acc,jerk,1.0f/BOARD_RT_FREQ,1);
        } else {
            if(speed<=0 || speed>high || speed>-low)return OD_VALUE_INVALID;
            if(value!=pos)t.planning=motion_plan(&t.plan,pos,value,acc,speed,jerk,1.0f/BOARD_RT_FREQ,1);
        }
        /* Planning is foreground-only. Reject overflow, invalid trajectories
         * and excessive horizons before publishing any real-time state. */
        if(value!=t.start && (!t.planning || !finite32(t.plan.total_t) || t.plan.total_t<=0 || t.plan.total_t>3600))return OD_VALUE_INVALID;
        const float coefficients[]={t.plan.feedback,t.plan.target,t.plan.vmax,t.plan.accmax,t.plan.dt,t.plan.total_ticks,
            t.plan.total_t,t.plan.dir,t.plan.v2,t.plan.dt1,t.plan.dt2,t.plan.dt3,t.plan.x1,t.plan.x2,t.plan.cx1,t.plan.cx2,t.plan.a0,t.plan.v0};
        for(unsigned i=0;i<sizeof(coefficients)/sizeof(coefficients[0]);++i)if(!finite32(coefficients[i]))return OD_VALUE_INVALID;
    }
    irq=__get_PRIMASK();__disable_irq();
    if(!gate() || atomic_load_explicit(&pending,memory_order_acquire) || epoch!=atomic_load_explicit(&generation,memory_order_acquire) ||
       atomic_load_explicit(&cancel,memory_order_acquire) || tick-prepared_tick>BOARD_HRT_FREQ/100U){__set_PRIMASK(irq);return OD_BUSY;}
    mailbox=t;accepted_tick=prepared_tick;++identifier;requested_mode=t.mode;phase=1;result_code=OD_OK;
    atomic_store_explicit(&pending,1,memory_order_release);__set_PRIMASK(irq);return OD_OK;
}
OD_Result od_motion_heartbeat(void)
{atomic_store_explicit(&heartbeat,1,memory_order_release);return OD_OK;}
void od_motion_cancel(void)
{atomic_fetch_add_explicit(&generation,1,memory_order_acq_rel);atomic_store_explicit(&cancel,1,memory_order_release);}
void od_motion_tick(bool can_apply)
{
    ++tick;
    if(atomic_exchange_explicit(&cancel,0,memory_order_acq_rel)) {
        atomic_store_explicit(&pending,0,memory_order_release);
        if(owned){bool error=global_status.motor_status[0].is_error;(void)motor_disable(0);if(error)global_status.motor_status[0].motor_state=MS_Err;}
        owned=false;phase=0;
    }
    if(atomic_exchange_explicit(&heartbeat,0,memory_order_acq_rel) && owned)lease_tick=tick;
    if(can_apply && atomic_load_explicit(&pending,memory_order_acquire)) {
        volatile Motor_Status_t *m=&global_status.motor_status[0];
        bool limited=(mailbox.mode==1 && (mailbox.value<m->negative_current_limit || mailbox.value>m->positive_current_limit)) ||
            (mailbox.mode==2 && (mailbox.value<m->negative_vel_limit || mailbox.value>m->positive_vel_limit)) ||
            (mailbox.mode==3 && (mailbox.speed>m->positive_vel_limit || mailbox.speed>-m->negative_vel_limit));
        if(!gate() || limited || tick-accepted_tick>BOARD_HRT_FREQ/100U){phase=4;result_code=OD_BUSY;}
        else {
            if(mailbox.mode==1){m->is_planning=0;m->current_ref=mailbox.value;m->motor_ctrl_mode=MCM_Current;}
            else {
                motion_planner[0]=mailbox.plan;m->is_planning=mailbox.planning;
                if(mailbox.mode==2){m->vel_ref=mailbox.planning?mailbox.start:mailbox.value;m->motor_ctrl_mode=MCM_Speed;}
                else {m->pos_ref=mailbox.planning?mailbox.start:mailbox.value;m->is_arrived=0;m->arrive_tick=0;m->motor_ctrl_mode=MCM_Pos;}
                m->vel_ff=0;m->acc_ref=0;m->torque_ff=0;m->follow_err_tick=0;
            }
            m->is_blocked=0;m->block_tick=0;owned=true;lease_tick=tick;phase=2;
        }
        atomic_store_explicit(&pending,0,memory_order_release);
    }
    if(owned && (!gate() || tick-lease_tick>BOARD_HRT_FREQ/2U)) {
        bool error=global_status.motor_status[0].is_error;
        (void)motor_disable(0);if(error)global_status.motor_status[0].motor_state=MS_Err;
        owned=false;phase=5;result_code=OD_BUSY;
    }
    if(owned && requested_mode==3 && global_status.motor_status[0].is_arrived)phase=3;
}
void od_motion_status(uint8_t out[16])
{
    uint32_t irq=__get_PRIMASK();__disable_irq();
    od_wire_put32(out,identifier);out[4]=phase;out[5]=result_code;out[6]=requested_mode;out[7]=0;
    od_wire_put32(out+8,global_status.motor_status[0].motor_ctrl_mode);
    uint32_t elapsed=tick-lease_tick;od_wire_put32(out+12,owned && elapsed<BOARD_HRT_FREQ/2U?BOARD_HRT_FREQ/2U-elapsed:0);
    __set_PRIMASK(irq);
}

