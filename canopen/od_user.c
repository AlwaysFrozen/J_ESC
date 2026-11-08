/* USER-OWNED FILE: the generator never changes this file. */
#include "od_user.h"
#include "global.h"
#include "od_commands.h"

/* Preserve the existing GNU ARM short-enum layout in configuration and status.
 * Enum values are explicitly widened to the CSV's UNSIGNED32 wire format. */
#define CHECK_ENUM_ABI(type) \
    _Static_assert(_Generic((type *)0, uint8_t *: 1, default: 0), \
                   #type " must retain the existing byte-sized enum ABI")
CHECK_ENUM_ABI(Motor_Type_t);
CHECK_ENUM_ABI(Motor_Struct_t);
CHECK_ENUM_ABI(Encoder_Type_t);
CHECK_ENUM_ABI(Encoder_Struct_t);
CHECK_ENUM_ABI(Velocity_Solver_Type_t);
CHECK_ENUM_ABI(Motor_Controller_Type_t);
CHECK_ENUM_ABI(Current_Decoupling_Type_t);
CHECK_ENUM_ABI(Align_Method_t);
CHECK_ENUM_ABI(Observer_Type_t);
CHECK_ENUM_ABI(Controller_Type_t);
CHECK_ENUM_ABI(Force_Controller_Type_t);
CHECK_ENUM_ABI(Filter_Struct_t);
CHECK_ENUM_ABI(Filter_Type_t);
CHECK_ENUM_ABI(Motor_State_t);
CHECK_ENUM_ABI(Motor_Control_Mode_t);
CHECK_ENUM_ABI(Brake_State_t);
CHECK_ENUM_ABI(FOC_Operate_Area_t);
CHECK_ENUM_ABI(HFI_Pole_Identify_State_t);
#undef CHECK_ENUM_ABI
_Static_assert(_Generic((FOC_Run_State_t *)0, unsigned int *: 1, default: 0),
               "FOC run state must retain its native 32-bit unsigned enum ABI");
_Static_assert(sizeof(int) == 4, "configuration requires 32-bit int");
_Static_assert(BOARD_SUPPORTED_MOTOR_NUMBER == 1U, "Extend the CSV and mappings for additional axes");

/* Each record has contiguous subindices. Generated slots are address-sorted,
 * so these differences yield the maximum subindex without a second count. */
static const uint8_t motor0_max_subindex =
    OD_NAME_motor0_saliency_ratio - OD_NAME_motor0_max_subindex;
static const uint8_t encoder0_max_subindex =
    OD_NAME_encoder0_velocity_solver_band_width - OD_NAME_encoder0_max_subindex;

static const uint8_t motor_controller0_max_subindex =
    OD_NAME_motor_controller0_gravity_compensation_enable - OD_NAME_motor_controller0_max_subindex;
static const uint8_t motor_observer0_max_subindex =
    OD_NAME_motor_observer0_neso_config_neso_z3_speed_threshold - OD_NAME_motor_observer0_max_subindex;
static const uint8_t loop_controller0_max_subindex =
    OD_NAME_loop_controller0_compliance_config_virtual_stiffness - OD_NAME_loop_controller0_max_subindex;
static const uint8_t loop_filter0_max_subindex =
    OD_NAME_loop_filter0_force_fb_filter_config_filter_depth - OD_NAME_loop_filter0_max_subindex;
static const uint8_t hfi0_max_subindex =
    OD_NAME_hfi0_switch_off_hz - OD_NAME_hfi0_max_subindex;

static const uint8_t core_max_subindex =
    OD_NAME_core_pwm_update_time_0 - OD_NAME_core_max_subindex;
static const uint8_t power_max_subindex =
    OD_NAME_power_inv_v_bus - OD_NAME_power_max_subindex;
static const uint8_t peripheral_raw_max_subindex =
    OD_NAME_peripheral_raw_ain_adc_raw - OD_NAME_peripheral_raw_max_subindex;
static const uint8_t motor_basic0_max_subindex =
    OD_NAME_motor_basic0_inv_v_bus - OD_NAME_motor_basic0_max_subindex;
static const uint8_t encoder_status0_max_subindex =
    OD_NAME_encoder_status0_encoder_pos_fractional_second - OD_NAME_encoder_status0_max_subindex;
static const uint8_t motor_status0_max_subindex =
    OD_NAME_motor_status0_ext_warn_code - OD_NAME_motor_status0_max_subindex;
static const uint8_t motor_observer_status0_max_subindex =
    OD_NAME_motor_observer_status0_observer_outputs_tor_disturb_estimate - OD_NAME_motor_observer_status0_max_subindex;
static const uint8_t loop_controller_status0_max_subindex =
    OD_NAME_loop_controller_status0_force_compliance_status_force_cmd - OD_NAME_loop_controller_status0_max_subindex;
static const uint8_t foc_status0_max_subindex =
    OD_NAME_foc_status0_dt_compensation_value - OD_NAME_foc_status0_max_subindex;
static const uint8_t hfi_status0_max_subindex =
    OD_NAME_hfi_status0_theta - OD_NAME_hfi_status0_max_subindex;

OD_Result od_user_init(const OD_Entry **first_missing)
{
    if (first_missing != NULL) *first_missing = NULL;
    if (od_dictionary.frozen) return OD_FROZEN;
    OD_Result result;

    result = od_commands_bind();
    if (result != OD_OK) return result;

    result = OD_BIND_RO(od_dictionary, motor0_max_subindex, &motor0_max_subindex);
    if (result != OD_OK) return result;
    result = OD_BIND_RO(od_dictionary, encoder0_max_subindex, &encoder0_max_subindex);
    if (result != OD_OK) return result;

    result = OD_BIND_RO(od_dictionary, motor_controller0_max_subindex, &motor_controller0_max_subindex);
    if (result != OD_OK) return result;
    result = OD_BIND_RO(od_dictionary, motor_observer0_max_subindex, &motor_observer0_max_subindex);
    if (result != OD_OK) return result;
    result = OD_BIND_RO(od_dictionary, loop_controller0_max_subindex, &loop_controller0_max_subindex);
    if (result != OD_OK) return result;
    result = OD_BIND_RO(od_dictionary, loop_filter0_max_subindex, &loop_filter0_max_subindex);
    if (result != OD_OK) return result;
    result = OD_BIND_RO(od_dictionary, hfi0_max_subindex, &hfi0_max_subindex);
    if (result != OD_OK) return result;

    result = OD_BIND_RO(od_dictionary, core_max_subindex, &core_max_subindex);
    if (result != OD_OK) return result;
    result = OD_BIND_RO(od_dictionary, power_max_subindex, &power_max_subindex);
    if (result != OD_OK) return result;
    result = OD_BIND_RO(od_dictionary, peripheral_raw_max_subindex, &peripheral_raw_max_subindex);
    if (result != OD_OK) return result;
    result = OD_BIND_RO(od_dictionary, motor_basic0_max_subindex, &motor_basic0_max_subindex);
    if (result != OD_OK) return result;
    result = OD_BIND_RO(od_dictionary, encoder_status0_max_subindex, &encoder_status0_max_subindex);
    if (result != OD_OK) return result;
    result = OD_BIND_RO(od_dictionary, motor_status0_max_subindex, &motor_status0_max_subindex);
    if (result != OD_OK) return result;
    result = OD_BIND_RO(od_dictionary, motor_observer_status0_max_subindex, &motor_observer_status0_max_subindex);
    if (result != OD_OK) return result;
    result = OD_BIND_RO(od_dictionary, loop_controller_status0_max_subindex, &loop_controller_status0_max_subindex);
    if (result != OD_OK) return result;
    result = OD_BIND_RO(od_dictionary, foc_status0_max_subindex, &foc_status0_max_subindex);
    if (result != OD_OK) return result;
    result = OD_BIND_RO(od_dictionary, hfi_status0_max_subindex, &hfi_status0_max_subindex);
    if (result != OD_OK) return result;

#define BIND_CONFIG(name, group, member) do { \
    result = OD_BIND(od_dictionary, name, &global_config.group[0].member); \
    if (result != OD_OK) return result; \
} while (0)

#define BIND_ENUM(name, group, member) do { \
    result = od_bind_u8_as_u32(&od_dictionary, OD_NAME_##name, &global_config.group[0].member); \
    if (result != OD_OK) return result; \
} while (0)

    BIND_ENUM(motor0_type, motor_config, motor_type);
    BIND_ENUM(motor0_struct, motor_config, motor_struct);
    BIND_CONFIG(motor0_rated_voltage, motor_config, rated_voltage);
    BIND_CONFIG(motor0_rated_current, motor_config, rated_current);
    BIND_CONFIG(motor0_peak_current, motor_config, peak_current);
    BIND_CONFIG(motor0_rated_speed, motor_config, rated_speed);
    BIND_CONFIG(motor0_peak_speed, motor_config, peak_speed);
    BIND_CONFIG(motor0_kt, motor_config, kt);
    BIND_CONFIG(motor0_ke, motor_config, ke);
    BIND_CONFIG(motor0_pp, motor_config, pp);
    BIND_CONFIG(motor0_rs, motor_config, Rs);
    BIND_CONFIG(motor0_ld, motor_config, Ld);
    BIND_CONFIG(motor0_lq, motor_config, Lq);
    BIND_CONFIG(motor0_flux, motor_config, Flux);
    BIND_CONFIG(motor0_inertia_rotor, motor_config, inertia_rotor);
    BIND_CONFIG(motor0_inertia_load, motor_config, inertia_load);
    BIND_CONFIG(motor0_peak_current_time_s, motor_config, peak_current_time_s);
    BIND_CONFIG(motor0_phase_over_current_threshold, motor_config, phase_over_current_threshold);
    BIND_CONFIG(motor0_bus_over_current_threshold, motor_config, bus_over_current_threshold);
    BIND_CONFIG(motor0_cooling_time_s, motor_config, cooling_time_s);
    BIND_CONFIG(motor0_dir, motor_config, motor_dir);
    BIND_CONFIG(motor0_have_brake, motor_config, have_brake);
    BIND_CONFIG(motor0_brake_open_time, motor_config, brake_open_time);
    BIND_CONFIG(motor0_brake_close_time, motor_config, brake_close_time);
    BIND_CONFIG(motor0_l_diff, motor_config, L_diff);
    BIND_CONFIG(motor0_saliency_ratio, motor_config, saliency_ratio);

    BIND_ENUM(encoder0_type, encoder_config, encoder_type);
    BIND_ENUM(encoder0_struct, encoder_config, encoder_struct);
    BIND_CONFIG(encoder0_delta_cpr, encoder_config, encoder_delta_cpr);
    BIND_CONFIG(encoder0_abs_cpr, encoder_config, encoder_abs_cpr);
    BIND_CONFIG(encoder0_baudrate, encoder_config, encoder_baudrate);
    BIND_CONFIG(encoder0_single_turn_bits, encoder_config, encoder_single_turn_bits);
    BIND_CONFIG(encoder0_mult_turn_bits, encoder_config, encoder_mult_turn_bits);
    BIND_CONFIG(encoder0_delta_dir, encoder_config, encoder_delta_dir);
    BIND_CONFIG(encoder0_abs_dir, encoder_config, encoder_abs_dir);
    BIND_ENUM(encoder0_velocity_solver_type, encoder_config, velocity_solver_type);
    BIND_CONFIG(encoder0_velocity_solver_band_width, encoder_config, velocity_solver_band_width);

    /* 0x2002: Motor_Controller_Config_t, nested members in declaration order. */
    BIND_CONFIG(motor_controller0_vel_feedforward_rate, motor_controller_config, vel_feedforward_rate);
    BIND_CONFIG(motor_controller0_acc_feedforward_rate, motor_controller_config, acc_feedforward_rate);
    BIND_ENUM(motor_controller0_motor_controller_type, motor_controller_config, motor_controller_type);
    BIND_CONFIG(motor_controller0_dead_zone_compensation_enable, motor_controller_config, dead_zone_compensation_enable);
    BIND_CONFIG(motor_controller0_max_modulation_ratio, motor_controller_config, max_modulation_ratio);
    BIND_ENUM(motor_controller0_current_decoupling_type, motor_controller_config, current_decoupling_type);
    BIND_CONFIG(motor_controller0_mtpa_enable, motor_controller_config, mtpa_enable);
    BIND_CONFIG(motor_controller0_fw_enable, motor_controller_config, fw_enable);
    BIND_CONFIG(motor_controller0_mtpv_enable, motor_controller_config, mtpv_enable);
    BIND_ENUM(motor_controller0_align_method, motor_controller_config, align_method);
    BIND_CONFIG(motor_controller0_align_force, motor_controller_config, align_force);
    BIND_CONFIG(motor_controller0_align_time, motor_controller_config, align_time);
    BIND_CONFIG(motor_controller0_align_threshold, motor_controller_config, align_threshold);
    BIND_CONFIG(motor_controller0_cogging_compensation_enable, motor_controller_config, cogging_compensation_enable);
    BIND_CONFIG(motor_controller0_friction_compensation_enable, motor_controller_config, friction_compensation_enable);
    BIND_CONFIG(motor_controller0_spring_compensation_enable, motor_controller_config, spring_compensation_enable);
    BIND_CONFIG(motor_controller0_gravity_compensation_enable, motor_controller_config, gravity_compensation_enable);

    /* 0x2003: Motor_Observer_Config_t, nested members in declaration order. */
    BIND_ENUM(motor_observer0_observser_config_observer_type, motor_observer_config, observser_config.observer_type);
    BIND_CONFIG(motor_observer0_observser_config_auto_tune, motor_observer_config, observser_config.auto_tune);
    BIND_CONFIG(motor_observer0_observser_config_observer_band_width, motor_observer_config, observser_config.observer_band_width);
    BIND_CONFIG(motor_observer0_neso_config_neso_gain, motor_observer_config, neso_config.neso_gain);
    BIND_CONFIG(motor_observer0_neso_config_neso_k_smooth, motor_observer_config, neso_config.neso_k_smooth);
    BIND_CONFIG(motor_observer0_neso_config_neso_z1_speed_threshold, motor_observer_config, neso_config.neso_z1_speed_threshold);
    BIND_CONFIG(motor_observer0_neso_config_neso_z2_speed_threshold, motor_observer_config, neso_config.neso_z2_speed_threshold);
    BIND_CONFIG(motor_observer0_neso_config_neso_z3_speed_threshold, motor_observer_config, neso_config.neso_z3_speed_threshold);

    /* 0x2004: Loop_Controller_Config_t, nested members in declaration order. */
    BIND_ENUM(loop_controller0_current_controller_config_controller_type, loop_controller_config, current_controller_config.controller_type);
    BIND_CONFIG(loop_controller0_current_controller_config_auto_tune, loop_controller_config, current_controller_config.auto_tune);
    BIND_CONFIG(loop_controller0_current_controller_config_controller_band_width, loop_controller_config, current_controller_config.controller_band_width);
    BIND_CONFIG(loop_controller0_current_controller_config_controller_integral_time, loop_controller_config, current_controller_config.controller_integral_time);
    BIND_CONFIG(loop_controller0_id_pid_config_kp, loop_controller_config, id_pid_config.kp);
    BIND_CONFIG(loop_controller0_id_pid_config_ki, loop_controller_config, id_pid_config.ki);
    BIND_CONFIG(loop_controller0_id_pid_config_kd, loop_controller_config, id_pid_config.kd);
    BIND_CONFIG(loop_controller0_id_pid_config_kb, loop_controller_config, id_pid_config.kb);
    BIND_CONFIG(loop_controller0_id_pid_config_alpha, loop_controller_config, id_pid_config.alpha);
    BIND_CONFIG(loop_controller0_id_pid_config_beta, loop_controller_config, id_pid_config.beta);
    BIND_CONFIG(loop_controller0_id_pid_config_gamma, loop_controller_config, id_pid_config.gamma);
    BIND_CONFIG(loop_controller0_iq_pid_config_kp, loop_controller_config, iq_pid_config.kp);
    BIND_CONFIG(loop_controller0_iq_pid_config_ki, loop_controller_config, iq_pid_config.ki);
    BIND_CONFIG(loop_controller0_iq_pid_config_kd, loop_controller_config, iq_pid_config.kd);
    BIND_CONFIG(loop_controller0_iq_pid_config_kb, loop_controller_config, iq_pid_config.kb);
    BIND_CONFIG(loop_controller0_iq_pid_config_alpha, loop_controller_config, iq_pid_config.alpha);
    BIND_CONFIG(loop_controller0_iq_pid_config_beta, loop_controller_config, iq_pid_config.beta);
    BIND_CONFIG(loop_controller0_iq_pid_config_gamma, loop_controller_config, iq_pid_config.gamma);
    BIND_ENUM(loop_controller0_fw_controller_config_controller_type, loop_controller_config, fw_controller_config.controller_type);
    BIND_CONFIG(loop_controller0_fw_controller_config_auto_tune, loop_controller_config, fw_controller_config.auto_tune);
    BIND_CONFIG(loop_controller0_fw_controller_config_controller_band_width, loop_controller_config, fw_controller_config.controller_band_width);
    BIND_CONFIG(loop_controller0_fw_controller_config_controller_integral_time, loop_controller_config, fw_controller_config.controller_integral_time);
    BIND_CONFIG(loop_controller0_fw_pid_config_kp, loop_controller_config, fw_pid_config.kp);
    BIND_CONFIG(loop_controller0_fw_pid_config_ki, loop_controller_config, fw_pid_config.ki);
    BIND_CONFIG(loop_controller0_fw_pid_config_kd, loop_controller_config, fw_pid_config.kd);
    BIND_CONFIG(loop_controller0_fw_pid_config_kb, loop_controller_config, fw_pid_config.kb);
    BIND_CONFIG(loop_controller0_fw_pid_config_alpha, loop_controller_config, fw_pid_config.alpha);
    BIND_CONFIG(loop_controller0_fw_pid_config_beta, loop_controller_config, fw_pid_config.beta);
    BIND_CONFIG(loop_controller0_fw_pid_config_gamma, loop_controller_config, fw_pid_config.gamma);
    BIND_ENUM(loop_controller0_speed_controller_config_controller_type, loop_controller_config, speed_controller_config.controller_type);
    BIND_CONFIG(loop_controller0_speed_controller_config_auto_tune, loop_controller_config, speed_controller_config.auto_tune);
    BIND_CONFIG(loop_controller0_speed_controller_config_controller_band_width, loop_controller_config, speed_controller_config.controller_band_width);
    BIND_CONFIG(loop_controller0_speed_controller_config_controller_integral_time, loop_controller_config, speed_controller_config.controller_integral_time);
    BIND_CONFIG(loop_controller0_speed_pid_config_kp, loop_controller_config, speed_pid_config.kp);
    BIND_CONFIG(loop_controller0_speed_pid_config_ki, loop_controller_config, speed_pid_config.ki);
    BIND_CONFIG(loop_controller0_speed_pid_config_kd, loop_controller_config, speed_pid_config.kd);
    BIND_CONFIG(loop_controller0_speed_pid_config_kb, loop_controller_config, speed_pid_config.kb);
    BIND_CONFIG(loop_controller0_speed_pid_config_alpha, loop_controller_config, speed_pid_config.alpha);
    BIND_CONFIG(loop_controller0_speed_pid_config_beta, loop_controller_config, speed_pid_config.beta);
    BIND_CONFIG(loop_controller0_speed_pid_config_gamma, loop_controller_config, speed_pid_config.gamma);
    BIND_ENUM(loop_controller0_pos_controller_config_controller_type, loop_controller_config, pos_controller_config.controller_type);
    BIND_CONFIG(loop_controller0_pos_controller_config_auto_tune, loop_controller_config, pos_controller_config.auto_tune);
    BIND_CONFIG(loop_controller0_pos_controller_config_controller_band_width, loop_controller_config, pos_controller_config.controller_band_width);
    BIND_CONFIG(loop_controller0_pos_controller_config_controller_integral_time, loop_controller_config, pos_controller_config.controller_integral_time);
    BIND_CONFIG(loop_controller0_pos_pid_config_kp, loop_controller_config, pos_pid_config.kp);
    BIND_CONFIG(loop_controller0_pos_pid_config_ki, loop_controller_config, pos_pid_config.ki);
    BIND_CONFIG(loop_controller0_pos_pid_config_kd, loop_controller_config, pos_pid_config.kd);
    BIND_CONFIG(loop_controller0_pos_pid_config_kb, loop_controller_config, pos_pid_config.kb);
    BIND_CONFIG(loop_controller0_pos_pid_config_alpha, loop_controller_config, pos_pid_config.alpha);
    BIND_CONFIG(loop_controller0_pos_pid_config_beta, loop_controller_config, pos_pid_config.beta);
    BIND_CONFIG(loop_controller0_pos_pid_config_gamma, loop_controller_config, pos_pid_config.gamma);
    BIND_ENUM(loop_controller0_force_controller_config_controller_type, loop_controller_config, force_controller_config.controller_type);
    BIND_CONFIG(loop_controller0_force_controller_config_auto_tune, loop_controller_config, force_controller_config.auto_tune);
    BIND_CONFIG(loop_controller0_force_controller_config_controller_band_width, loop_controller_config, force_controller_config.controller_band_width);
    BIND_CONFIG(loop_controller0_force_controller_config_Md, loop_controller_config, force_controller_config.Md);
    BIND_CONFIG(loop_controller0_compliance_config_virtual_mass, loop_controller_config, compliance_config.virtual_mass);
    BIND_CONFIG(loop_controller0_compliance_config_virtual_damping, loop_controller_config, compliance_config.virtual_damping);
    BIND_CONFIG(loop_controller0_compliance_config_virtual_stiffness, loop_controller_config, compliance_config.virtual_stiffness);

    /* 0x2005: Loop_Filter_Config_t, nested members in declaration order. */
    BIND_ENUM(loop_filter0_current_fb_filter_config_filter_struct, loop_filter_config, current_fb_filter_config.filter_struct);
    BIND_ENUM(loop_filter0_current_fb_filter_config_filter_type, loop_filter_config, current_fb_filter_config.filter_type);
    BIND_CONFIG(loop_filter0_current_fb_filter_config_filter_order, loop_filter_config, current_fb_filter_config.filter_order);
    BIND_CONFIG(loop_filter0_current_fb_filter_config_filter_freq, loop_filter_config, current_fb_filter_config.filter_freq);
    BIND_CONFIG(loop_filter0_current_fb_filter_config_filter_width, loop_filter_config, current_fb_filter_config.filter_width);
    BIND_CONFIG(loop_filter0_current_fb_filter_config_filter_depth, loop_filter_config, current_fb_filter_config.filter_depth);
    BIND_ENUM(loop_filter0_current_ref_filter_config_filter_struct, loop_filter_config, current_ref_filter_config.filter_struct);
    BIND_ENUM(loop_filter0_current_ref_filter_config_filter_type, loop_filter_config, current_ref_filter_config.filter_type);
    BIND_CONFIG(loop_filter0_current_ref_filter_config_filter_order, loop_filter_config, current_ref_filter_config.filter_order);
    BIND_CONFIG(loop_filter0_current_ref_filter_config_filter_freq, loop_filter_config, current_ref_filter_config.filter_freq);
    BIND_CONFIG(loop_filter0_current_ref_filter_config_filter_width, loop_filter_config, current_ref_filter_config.filter_width);
    BIND_CONFIG(loop_filter0_current_ref_filter_config_filter_depth, loop_filter_config, current_ref_filter_config.filter_depth);
    BIND_ENUM(loop_filter0_speed_out_filter_config_filter_struct, loop_filter_config, speed_out_filter_config.filter_struct);
    BIND_ENUM(loop_filter0_speed_out_filter_config_filter_type, loop_filter_config, speed_out_filter_config.filter_type);
    BIND_CONFIG(loop_filter0_speed_out_filter_config_filter_order, loop_filter_config, speed_out_filter_config.filter_order);
    BIND_CONFIG(loop_filter0_speed_out_filter_config_filter_freq, loop_filter_config, speed_out_filter_config.filter_freq);
    BIND_CONFIG(loop_filter0_speed_out_filter_config_filter_width, loop_filter_config, speed_out_filter_config.filter_width);
    BIND_CONFIG(loop_filter0_speed_out_filter_config_filter_depth, loop_filter_config, speed_out_filter_config.filter_depth);
    BIND_ENUM(loop_filter0_speed_ref_filter_config_filter_struct, loop_filter_config, speed_ref_filter_config.filter_struct);
    BIND_ENUM(loop_filter0_speed_ref_filter_config_filter_type, loop_filter_config, speed_ref_filter_config.filter_type);
    BIND_CONFIG(loop_filter0_speed_ref_filter_config_filter_order, loop_filter_config, speed_ref_filter_config.filter_order);
    BIND_CONFIG(loop_filter0_speed_ref_filter_config_filter_freq, loop_filter_config, speed_ref_filter_config.filter_freq);
    BIND_CONFIG(loop_filter0_speed_ref_filter_config_filter_width, loop_filter_config, speed_ref_filter_config.filter_width);
    BIND_CONFIG(loop_filter0_speed_ref_filter_config_filter_depth, loop_filter_config, speed_ref_filter_config.filter_depth);
    BIND_ENUM(loop_filter0_pos_ref_filter_config_filter_struct, loop_filter_config, pos_ref_filter_config.filter_struct);
    BIND_ENUM(loop_filter0_pos_ref_filter_config_filter_type, loop_filter_config, pos_ref_filter_config.filter_type);
    BIND_CONFIG(loop_filter0_pos_ref_filter_config_filter_order, loop_filter_config, pos_ref_filter_config.filter_order);
    BIND_CONFIG(loop_filter0_pos_ref_filter_config_filter_freq, loop_filter_config, pos_ref_filter_config.filter_freq);
    BIND_CONFIG(loop_filter0_pos_ref_filter_config_filter_width, loop_filter_config, pos_ref_filter_config.filter_width);
    BIND_CONFIG(loop_filter0_pos_ref_filter_config_filter_depth, loop_filter_config, pos_ref_filter_config.filter_depth);
    BIND_ENUM(loop_filter0_force_fb_filter_config_filter_struct, loop_filter_config, force_fb_filter_config.filter_struct);
    BIND_ENUM(loop_filter0_force_fb_filter_config_filter_type, loop_filter_config, force_fb_filter_config.filter_type);
    BIND_CONFIG(loop_filter0_force_fb_filter_config_filter_order, loop_filter_config, force_fb_filter_config.filter_order);
    BIND_CONFIG(loop_filter0_force_fb_filter_config_filter_freq, loop_filter_config, force_fb_filter_config.filter_freq);
    BIND_CONFIG(loop_filter0_force_fb_filter_config_filter_width, loop_filter_config, force_fb_filter_config.filter_width);
    BIND_CONFIG(loop_filter0_force_fb_filter_config_filter_depth, loop_filter_config, force_fb_filter_config.filter_depth);

    /* 0x2006: HFI_Config_t, nested members in declaration order. */
    BIND_CONFIG(hfi0_div, hfi_config, div);
    BIND_CONFIG(hfi0_pll_fc, hfi_config, pll_fc);
    BIND_CONFIG(hfi0_amp, hfi_config, amp);
    BIND_CONFIG(hfi0_pole_identify_current, hfi_config, pole_identify_current);
    BIND_CONFIG(hfi0_pole_identify_amp, hfi_config, pole_identify_amp);
    BIND_CONFIG(hfi0_pole_identify_stable_us, hfi_config, pole_identify_stable_us);
    BIND_CONFIG(hfi0_pole_identify_pulse_us, hfi_config, pole_identify_pulse_us);
    BIND_CONFIG(hfi0_pole_identify_interval_us, hfi_config, pole_identify_interval_us);
    BIND_CONFIG(hfi0_switch_on_hz, hfi_config, switch_on_hz);
    BIND_CONFIG(hfi0_switch_off_hz, hfi_config, switch_off_hz);


#define BIND_STATUS(name, member) do { \
    result = OD_BIND_RO(od_dictionary, name, &global_status.member); \
    if (result != OD_OK) return result; \
} while (0)

#define BIND_STATUS_ENUM(name, member) do { \
    result = od_bind_ro_u8_as_u32(&od_dictionary, OD_NAME_##name, &global_status.member); \
    if (result != OD_OK) return result; \
} while (0)

    /* 0x2100: read-only Core_Status_t. */
    BIND_STATUS(core_hrt_interval_us, core_status.hrt_interval_us);
    BIND_STATUS(core_hrt_consume_us, core_status.hrt_consume_us);
    BIND_STATUS(core_hrt_consume_us_max, core_status.hrt_consume_us_max);
    BIND_STATUS(core_hrt_cnt, core_status.hrt_cnt);
    BIND_STATUS(core_rt_interval_us, core_status.rt_interval_us);
    BIND_STATUS(core_rt_consume_us, core_status.rt_consume_us);
    BIND_STATUS(core_rt_consume_us_max, core_status.rt_consume_us_max);
    BIND_STATUS(core_rt_cnt, core_status.rt_cnt);
    BIND_STATUS(core_adc_isr_time, core_status.adc_isr_time);
    BIND_STATUS(core_pwm_update_time_0, core_status.pwm_update_time[0]);

    /* 0x2101: read-only Power_Status_t. */
    BIND_STATUS(power_v_bus, power_status.v_bus);
    BIND_STATUS(power_v_bus_f, power_status.v_bus_f);
    BIND_STATUS(power_i_bus, power_status.i_bus);
    BIND_STATUS(power_i_bus_f, power_status.i_bus_f);
    BIND_STATUS(power_core_temp, power_status.core_temp);
    BIND_STATUS(power_board_temp, power_status.board_temp);
    BIND_STATUS(power_board_power, power_status.board_power);
    BIND_STATUS(power_inv_v_bus, power_status.inv_v_bus);

    /* 0x2102: read-only Peripheral_Raw_Status_t. */
    BIND_STATUS(peripheral_raw_ia_adc_raw_0, peripheral_raw_status.ia_adc_raw[0]);
    BIND_STATUS(peripheral_raw_ib_adc_raw_0, peripheral_raw_status.ib_adc_raw[0]);
    BIND_STATUS(peripheral_raw_ic_adc_raw_0, peripheral_raw_status.ic_adc_raw[0]);
    BIND_STATUS(peripheral_raw_va_adc_raw_0, peripheral_raw_status.va_adc_raw[0]);
    BIND_STATUS(peripheral_raw_vb_adc_raw_0, peripheral_raw_status.vb_adc_raw[0]);
    BIND_STATUS(peripheral_raw_vc_adc_raw_0, peripheral_raw_status.vc_adc_raw[0]);
    BIND_STATUS(peripheral_raw_v_bus_adc_raw, peripheral_raw_status.v_bus_adc_raw);
    BIND_STATUS(peripheral_raw_board_temp_adc_raw, peripheral_raw_status.board_temp_adc_raw);
    BIND_STATUS(peripheral_raw_motor_temp_adc_raw, peripheral_raw_status.motor_temp_adc_raw);
    BIND_STATUS(peripheral_raw_ain_adc_raw, peripheral_raw_status.ain_adc_raw);

    /* 0x2103: read-only Motor_Basic_Status_t. */
    BIND_STATUS(motor_basic0_phase_voltage_offset_U, motor_basic_status[0].phase_voltage_offset.U);
    BIND_STATUS(motor_basic0_phase_voltage_offset_V, motor_basic_status[0].phase_voltage_offset.V);
    BIND_STATUS(motor_basic0_phase_voltage_offset_W, motor_basic_status[0].phase_voltage_offset.W);
    BIND_STATUS(motor_basic0_phase_voltage_U, motor_basic_status[0].phase_voltage.U);
    BIND_STATUS(motor_basic0_phase_voltage_V, motor_basic_status[0].phase_voltage.V);
    BIND_STATUS(motor_basic0_phase_voltage_W, motor_basic_status[0].phase_voltage.W);
    BIND_STATUS(motor_basic0_phase_current_offset_U, motor_basic_status[0].phase_current_offset.U);
    BIND_STATUS(motor_basic0_phase_current_offset_V, motor_basic_status[0].phase_current_offset.V);
    BIND_STATUS(motor_basic0_phase_current_offset_W, motor_basic_status[0].phase_current_offset.W);
    BIND_STATUS(motor_basic0_phase_current_U, motor_basic_status[0].phase_current.U);
    BIND_STATUS(motor_basic0_phase_current_V, motor_basic_status[0].phase_current.V);
    BIND_STATUS(motor_basic0_phase_current_W, motor_basic_status[0].phase_current.W);
    BIND_STATUS(motor_basic0_pwm_duty_raw_U, motor_basic_status[0].pwm_duty_raw.U);
    BIND_STATUS(motor_basic0_pwm_duty_raw_V, motor_basic_status[0].pwm_duty_raw.V);
    BIND_STATUS(motor_basic0_pwm_duty_raw_W, motor_basic_status[0].pwm_duty_raw.W);
    BIND_STATUS(motor_basic0_pwm_duty_comp_U, motor_basic_status[0].pwm_duty_comp.U);
    BIND_STATUS(motor_basic0_pwm_duty_comp_V, motor_basic_status[0].pwm_duty_comp.V);
    BIND_STATUS(motor_basic0_pwm_duty_comp_W, motor_basic_status[0].pwm_duty_comp.W);
    BIND_STATUS(motor_basic0_pwm_duty_out_U, motor_basic_status[0].pwm_duty_out.U);
    BIND_STATUS(motor_basic0_pwm_duty_out_V, motor_basic_status[0].pwm_duty_out.V);
    BIND_STATUS(motor_basic0_pwm_duty_out_W, motor_basic_status[0].pwm_duty_out.W);
    BIND_STATUS(motor_basic0_phase_voltage_out_U, motor_basic_status[0].phase_voltage_out.U);
    BIND_STATUS(motor_basic0_phase_voltage_out_V, motor_basic_status[0].phase_voltage_out.V);
    BIND_STATUS(motor_basic0_phase_voltage_out_W, motor_basic_status[0].phase_voltage_out.W);
    BIND_STATUS(motor_basic0_i_sum, motor_basic_status[0].i_sum);
    BIND_STATUS(motor_basic0_i_rms, motor_basic_status[0].i_rms);
    BIND_STATUS(motor_basic0_v_bus, motor_basic_status[0].v_bus);
    BIND_STATUS(motor_basic0_i_bus, motor_basic_status[0].i_bus);
    BIND_STATUS(motor_basic0_e_power, motor_basic_status[0].e_power);
    BIND_STATUS(motor_basic0_m_power, motor_basic_status[0].m_power);
    BIND_STATUS(motor_basic0_efficiency, motor_basic_status[0].efficiency);
    BIND_STATUS(motor_basic0_inv_v_bus, motor_basic_status[0].inv_v_bus);

    /* 0x2104: read-only Encoder_Status_t. */
    BIND_STATUS(encoder_status0_encoder_delta_max_cnt, encoder_status[0].encoder_delta_max_cnt);
    BIND_STATUS(encoder_status0_encoder_delta_cnt, encoder_status[0].encoder_delta_cnt);
    BIND_STATUS(encoder_status0_encoder_delta_cnt_last, encoder_status[0].encoder_delta_cnt_last);
    BIND_STATUS(encoder_status0_encoder_delta_turn, encoder_status[0].encoder_delta_turn);
    BIND_STATUS(encoder_status0_encoder_delta_fractional, encoder_status[0].encoder_delta_fractional);
    BIND_STATUS(encoder_status0_encoder_delta_home_offset, encoder_status[0].encoder_delta_home_offset);
    BIND_STATUS(encoder_status0_encoder_delta_pos, encoder_status[0].encoder_delta_pos);
    BIND_STATUS(encoder_status0_encoder_delta_pos_last, encoder_status[0].encoder_delta_pos_last);
    BIND_STATUS(encoder_status0_encoder_abs_max_cnt, encoder_status[0].encoder_abs_max_cnt);
    BIND_STATUS(encoder_status0_encoder_abs_cnt, encoder_status[0].encoder_abs_cnt);
    BIND_STATUS(encoder_status0_encoder_abs_cnt_last, encoder_status[0].encoder_abs_cnt_last);
    BIND_STATUS(encoder_status0_encoder_abs_turn, encoder_status[0].encoder_abs_turn);
    BIND_STATUS(encoder_status0_encoder_abs_fractional, encoder_status[0].encoder_abs_fractional);
    BIND_STATUS(encoder_status0_encoder_abs_zero_offset, encoder_status[0].encoder_abs_zero_offset);
    BIND_STATUS(encoder_status0_encoder_abs_home_offset, encoder_status[0].encoder_abs_home_offset);
    BIND_STATUS(encoder_status0_encoder_abs_pos, encoder_status[0].encoder_abs_pos);
    BIND_STATUS(encoder_status0_encoder_abs_pos_last, encoder_status[0].encoder_abs_pos_last);
    BIND_STATUS(encoder_status0_encoder_index_cnt, encoder_status[0].encoder_index_cnt);
    BIND_STATUS(encoder_status0_encoder_pos, encoder_status[0].encoder_pos);
    BIND_STATUS(encoder_status0_encoder_pos_fractional, encoder_status[0].encoder_pos_fractional);
    BIND_STATUS(encoder_status0_encoder_pos_last, encoder_status[0].encoder_pos_last);
    BIND_STATUS(encoder_status0_encoder_speed, encoder_status[0].encoder_speed);
    BIND_STATUS(encoder_status0_encoder_speed_last, encoder_status[0].encoder_speed_last);
    BIND_STATUS(encoder_status0_encoder_acc, encoder_status[0].encoder_acc);
    BIND_STATUS(encoder_status0_encoder_acc_last, encoder_status[0].encoder_acc_last);
    BIND_STATUS(encoder_status0_encoder_pos_second, encoder_status[0].encoder_pos_second);
    BIND_STATUS(encoder_status0_encoder_pos_fractional_second, encoder_status[0].encoder_pos_fractional_second);

    /* 0x2105: read-only Motor_Status_t. */
    BIND_STATUS_ENUM(motor_status0_motor_state, motor_status[0].motor_state);
    BIND_STATUS_ENUM(motor_status0_motor_ctrl_mode, motor_status[0].motor_ctrl_mode);
    BIND_STATUS_ENUM(motor_status0_brake_state, motor_status[0].brake_state);
    BIND_STATUS(motor_status0_brake_cnt, motor_status[0].brake_cnt);
    BIND_STATUS(motor_status0_is_calibrated, motor_status[0].is_calibrated);
    BIND_STATUS(motor_status0_is_enabled, motor_status[0].is_enabled);
    BIND_STATUS(motor_status0_is_error, motor_status[0].is_error);
    BIND_STATUS(motor_status0_is_aligned, motor_status[0].is_aligned);
    BIND_STATUS(motor_status0_is_braked, motor_status[0].is_braked);
    BIND_STATUS(motor_status0_is_homed, motor_status[0].is_homed);
    BIND_STATUS(motor_status0_is_planning, motor_status[0].is_planning);
    BIND_STATUS(motor_status0_is_moving, motor_status[0].is_moving);
    BIND_STATUS(motor_status0_is_arrived, motor_status[0].is_arrived);
    BIND_STATUS(motor_status0_is_blocked, motor_status[0].is_blocked);
    BIND_STATUS(motor_status0_is_follow_err, motor_status[0].is_follow_err);
    BIND_STATUS(motor_status0_err_code, motor_status[0].err_code);
    BIND_STATUS(motor_status0_motor_cnt, motor_status[0].motor_cnt);
    BIND_STATUS(motor_status0_voltage_up_cnt, motor_status[0].voltage_up_cnt);
    BIND_STATUS(motor_status0_voltage_down_cnt, motor_status[0].voltage_down_cnt);
    BIND_STATUS(motor_status0_core_temp_high_cnt, motor_status[0].core_temp_high_cnt);
    BIND_STATUS(motor_status0_core_temp_low_cnt, motor_status[0].core_temp_low_cnt);
    BIND_STATUS(motor_status0_board_temp_high_cnt, motor_status[0].board_temp_high_cnt);
    BIND_STATUS(motor_status0_board_temp_low_cnt, motor_status[0].board_temp_low_cnt);
    BIND_STATUS(motor_status0_phase_over_current_cnt, motor_status[0].phase_over_current_cnt);
    BIND_STATUS(motor_status0_bus_over_current_cnt, motor_status[0].bus_over_current_cnt);
    BIND_STATUS(motor_status0_board_over_current_cnt, motor_status[0].board_over_current_cnt);
    BIND_STATUS(motor_status0_current_unbalance_cnt, motor_status[0].current_unbalance_cnt);
    BIND_STATUS(motor_status0_thermal_limit, motor_status[0].thermal_limit);
    BIND_STATUS(motor_status0_thermal_accumulator, motor_status[0].thermal_accumulator);
    BIND_STATUS(motor_status0_follow_err_window, motor_status[0].follow_err_window);
    BIND_STATUS(motor_status0_follow_err_time, motor_status[0].follow_err_time);
    BIND_STATUS(motor_status0_follow_err_tick, motor_status[0].follow_err_tick);
    BIND_STATUS(motor_status0_arrive_window, motor_status[0].arrive_window);
    BIND_STATUS(motor_status0_arrive_time, motor_status[0].arrive_time);
    BIND_STATUS(motor_status0_arrive_tick, motor_status[0].arrive_tick);
    BIND_STATUS(motor_status0_block_vel, motor_status[0].block_vel);
    BIND_STATUS(motor_status0_block_tor, motor_status[0].block_tor);
    BIND_STATUS(motor_status0_block_time, motor_status[0].block_time);
    BIND_STATUS(motor_status0_block_tick, motor_status[0].block_tick);
    BIND_STATUS(motor_status0_positive_current_limit, motor_status[0].positive_current_limit);
    BIND_STATUS(motor_status0_negative_current_limit, motor_status[0].negative_current_limit);
    BIND_STATUS(motor_status0_positive_tor_limit, motor_status[0].positive_tor_limit);
    BIND_STATUS(motor_status0_negative_tor_limit, motor_status[0].negative_tor_limit);
    BIND_STATUS(motor_status0_positive_vel_limit, motor_status[0].positive_vel_limit);
    BIND_STATUS(motor_status0_negative_vel_limit, motor_status[0].negative_vel_limit);
    BIND_STATUS(motor_status0_current_ref, motor_status[0].current_ref);
    BIND_STATUS(motor_status0_current_ref_f, motor_status[0].current_ref_f);
    BIND_STATUS(motor_status0_current_fb, motor_status[0].current_fb);
    BIND_STATUS(motor_status0_current_fb_f, motor_status[0].current_fb_f);
    BIND_STATUS(motor_status0_current_abs, motor_status[0].current_abs);
    BIND_STATUS(motor_status0_current_err, motor_status[0].current_err);
    BIND_STATUS(motor_status0_current_err_abs, motor_status[0].current_err_abs);
    BIND_STATUS(motor_status0_torque_ref, motor_status[0].torque_ref);
    BIND_STATUS(motor_status0_torque_ref_f, motor_status[0].torque_ref_f);
    BIND_STATUS(motor_status0_torque_ff, motor_status[0].torque_ff);
    BIND_STATUS(motor_status0_torque_fb, motor_status[0].torque_fb);
    BIND_STATUS(motor_status0_torque_fb_f, motor_status[0].torque_fb_f);
    BIND_STATUS(motor_status0_torque_abs, motor_status[0].torque_abs);
    BIND_STATUS(motor_status0_torque_err, motor_status[0].torque_err);
    BIND_STATUS(motor_status0_torque_err_abs, motor_status[0].torque_err_abs);
    BIND_STATUS(motor_status0_acc_ref, motor_status[0].acc_ref);
    BIND_STATUS(motor_status0_acc_fb, motor_status[0].acc_fb);
    BIND_STATUS(motor_status0_acc_abs, motor_status[0].acc_abs);
    BIND_STATUS(motor_status0_acc_err, motor_status[0].acc_err);
    BIND_STATUS(motor_status0_acc_err_abs, motor_status[0].acc_err_abs);
    BIND_STATUS(motor_status0_vel_ref, motor_status[0].vel_ref);
    BIND_STATUS(motor_status0_vel_ref_f, motor_status[0].vel_ref_f);
    BIND_STATUS(motor_status0_vel_ff, motor_status[0].vel_ff);
    BIND_STATUS(motor_status0_vel_fb, motor_status[0].vel_fb);
    BIND_STATUS(motor_status0_vel_abs, motor_status[0].vel_abs);
    BIND_STATUS(motor_status0_vel_err, motor_status[0].vel_err);
    BIND_STATUS(motor_status0_vel_err_abs, motor_status[0].vel_err_abs);
    BIND_STATUS(motor_status0_vel_out, motor_status[0].vel_out);
    BIND_STATUS(motor_status0_vel_out_f, motor_status[0].vel_out_f);
    BIND_STATUS(motor_status0_pos_ref, motor_status[0].pos_ref);
    BIND_STATUS(motor_status0_pos_ref_f, motor_status[0].pos_ref_f);
    BIND_STATUS(motor_status0_pos_fb, motor_status[0].pos_fb);
    BIND_STATUS(motor_status0_pos_abs, motor_status[0].pos_abs);
    BIND_STATUS(motor_status0_pos_err, motor_status[0].pos_err);
    BIND_STATUS(motor_status0_pos_err_abs, motor_status[0].pos_err_abs);
    BIND_STATUS(motor_status0_force_ref, motor_status[0].force_ref);
    BIND_STATUS(motor_status0_force_fb, motor_status[0].force_fb);
    BIND_STATUS(motor_status0_force_fb_f, motor_status[0].force_fb_f);
    BIND_STATUS(motor_status0_force_abs, motor_status[0].force_abs);
    BIND_STATUS(motor_status0_force_err, motor_status[0].force_err);
    BIND_STATUS(motor_status0_force_err_abs, motor_status[0].force_err_abs);
    BIND_STATUS(motor_status0_ext_err_code, motor_status[0].ext_err_code);
    BIND_STATUS(motor_status0_ext_warn_code, motor_status[0].ext_warn_code);

    /* 0x2106: read-only Motor_Observer_t. */
    BIND_STATUS(motor_observer_status0_neso_config_neso_gain, motor_observer[0].neso_config.neso_gain);
    BIND_STATUS(motor_observer_status0_neso_config_neso_k_smooth, motor_observer[0].neso_config.neso_k_smooth);
    BIND_STATUS(motor_observer_status0_neso_config_neso_z1_speed_threshold, motor_observer[0].neso_config.neso_z1_speed_threshold);
    BIND_STATUS(motor_observer_status0_neso_config_neso_z2_speed_threshold, motor_observer[0].neso_config.neso_z2_speed_threshold);
    BIND_STATUS(motor_observer_status0_neso_config_neso_z3_speed_threshold, motor_observer[0].neso_config.neso_z3_speed_threshold);
    BIND_STATUS(motor_observer_status0_neso_status_err, motor_observer[0].neso_status.err);
    BIND_STATUS(motor_observer_status0_neso_status_err_abs, motor_observer[0].neso_status.err_abs);
    BIND_STATUS(motor_observer_status0_neso_status_l1, motor_observer[0].neso_status.l1);
    BIND_STATUS(motor_observer_status0_neso_status_l2, motor_observer[0].neso_status.l2);
    BIND_STATUS(motor_observer_status0_neso_status_l3, motor_observer[0].neso_status.l3);
    BIND_STATUS(motor_observer_status0_neso_status_dz1, motor_observer[0].neso_status.dz1);
    BIND_STATUS(motor_observer_status0_neso_status_dz2, motor_observer[0].neso_status.dz2);
    BIND_STATUS(motor_observer_status0_neso_status_dz3, motor_observer[0].neso_status.dz3);
    BIND_STATUS(motor_observer_status0_neso_status_z1, motor_observer[0].neso_status.z1);
    BIND_STATUS(motor_observer_status0_neso_status_z2, motor_observer[0].neso_status.z2);
    BIND_STATUS(motor_observer_status0_neso_status_z3, motor_observer[0].neso_status.z3);
    BIND_STATUS(motor_observer_status0_observer_outputs_pos_estimate, motor_observer[0].observer_outputs.pos_estimate);
    BIND_STATUS(motor_observer_status0_observer_outputs_vel_estimate, motor_observer[0].observer_outputs.vel_estimate);
    BIND_STATUS(motor_observer_status0_observer_outputs_acc_estimate, motor_observer[0].observer_outputs.acc_estimate);
    BIND_STATUS(motor_observer_status0_observer_outputs_tor_out_estimate, motor_observer[0].observer_outputs.tor_out_estimate);
    BIND_STATUS(motor_observer_status0_observer_outputs_tor_disturb_estimate, motor_observer[0].observer_outputs.tor_disturb_estimate);

    /* 0x2107: read-only Loop_Controller_t. */
    BIND_STATUS(loop_controller_status0_id_pid_dt, loop_controller_status[0].id_pid.dt);
    BIND_STATUS(loop_controller_status0_id_pid_config_kp, loop_controller_status[0].id_pid.config.kp);
    BIND_STATUS(loop_controller_status0_id_pid_config_ki, loop_controller_status[0].id_pid.config.ki);
    BIND_STATUS(loop_controller_status0_id_pid_config_kd, loop_controller_status[0].id_pid.config.kd);
    BIND_STATUS(loop_controller_status0_id_pid_config_kb, loop_controller_status[0].id_pid.config.kb);
    BIND_STATUS(loop_controller_status0_id_pid_config_alpha, loop_controller_status[0].id_pid.config.alpha);
    BIND_STATUS(loop_controller_status0_id_pid_config_beta, loop_controller_status[0].id_pid.config.beta);
    BIND_STATUS(loop_controller_status0_id_pid_config_gamma, loop_controller_status[0].id_pid.config.gamma);
    BIND_STATUS(loop_controller_status0_id_pid_limit_limit_max, loop_controller_status[0].id_pid.limit.limit_max);
    BIND_STATUS(loop_controller_status0_id_pid_limit_limit_min, loop_controller_status[0].id_pid.limit.limit_min);
    BIND_STATUS(loop_controller_status0_id_pid_status_proportional, loop_controller_status[0].id_pid.status.proportional);
    BIND_STATUS(loop_controller_status0_id_pid_status_integral, loop_controller_status[0].id_pid.status.integral);
    BIND_STATUS(loop_controller_status0_id_pid_status_derivative, loop_controller_status[0].id_pid.status.derivative);
    BIND_STATUS(loop_controller_status0_id_pid_status_derivative_f, loop_controller_status[0].id_pid.status.derivative_f);
    BIND_STATUS(loop_controller_status0_id_pid_status_out_temp, loop_controller_status[0].id_pid.status.out_temp);
    BIND_STATUS(loop_controller_status0_id_pid_status_out, loop_controller_status[0].id_pid.status.out);
    BIND_STATUS(loop_controller_status0_id_pid_status_err_k0, loop_controller_status[0].id_pid.status.err_k0);
    BIND_STATUS(loop_controller_status0_id_pid_status_err_k1, loop_controller_status[0].id_pid.status.err_k1);
    BIND_STATUS(loop_controller_status0_id_pid_status_err_k2, loop_controller_status[0].id_pid.status.err_k2);
    BIND_STATUS(loop_controller_status0_id_pid_status_saturation, loop_controller_status[0].id_pid.status.saturation);
    BIND_STATUS(loop_controller_status0_iq_pid_dt, loop_controller_status[0].iq_pid.dt);
    BIND_STATUS(loop_controller_status0_iq_pid_config_kp, loop_controller_status[0].iq_pid.config.kp);
    BIND_STATUS(loop_controller_status0_iq_pid_config_ki, loop_controller_status[0].iq_pid.config.ki);
    BIND_STATUS(loop_controller_status0_iq_pid_config_kd, loop_controller_status[0].iq_pid.config.kd);
    BIND_STATUS(loop_controller_status0_iq_pid_config_kb, loop_controller_status[0].iq_pid.config.kb);
    BIND_STATUS(loop_controller_status0_iq_pid_config_alpha, loop_controller_status[0].iq_pid.config.alpha);
    BIND_STATUS(loop_controller_status0_iq_pid_config_beta, loop_controller_status[0].iq_pid.config.beta);
    BIND_STATUS(loop_controller_status0_iq_pid_config_gamma, loop_controller_status[0].iq_pid.config.gamma);
    BIND_STATUS(loop_controller_status0_iq_pid_limit_limit_max, loop_controller_status[0].iq_pid.limit.limit_max);
    BIND_STATUS(loop_controller_status0_iq_pid_limit_limit_min, loop_controller_status[0].iq_pid.limit.limit_min);
    BIND_STATUS(loop_controller_status0_iq_pid_status_proportional, loop_controller_status[0].iq_pid.status.proportional);
    BIND_STATUS(loop_controller_status0_iq_pid_status_integral, loop_controller_status[0].iq_pid.status.integral);
    BIND_STATUS(loop_controller_status0_iq_pid_status_derivative, loop_controller_status[0].iq_pid.status.derivative);
    BIND_STATUS(loop_controller_status0_iq_pid_status_derivative_f, loop_controller_status[0].iq_pid.status.derivative_f);
    BIND_STATUS(loop_controller_status0_iq_pid_status_out_temp, loop_controller_status[0].iq_pid.status.out_temp);
    BIND_STATUS(loop_controller_status0_iq_pid_status_out, loop_controller_status[0].iq_pid.status.out);
    BIND_STATUS(loop_controller_status0_iq_pid_status_err_k0, loop_controller_status[0].iq_pid.status.err_k0);
    BIND_STATUS(loop_controller_status0_iq_pid_status_err_k1, loop_controller_status[0].iq_pid.status.err_k1);
    BIND_STATUS(loop_controller_status0_iq_pid_status_err_k2, loop_controller_status[0].iq_pid.status.err_k2);
    BIND_STATUS(loop_controller_status0_iq_pid_status_saturation, loop_controller_status[0].iq_pid.status.saturation);
    BIND_STATUS(loop_controller_status0_fw_pid_dt, loop_controller_status[0].fw_pid.dt);
    BIND_STATUS(loop_controller_status0_fw_pid_config_kp, loop_controller_status[0].fw_pid.config.kp);
    BIND_STATUS(loop_controller_status0_fw_pid_config_ki, loop_controller_status[0].fw_pid.config.ki);
    BIND_STATUS(loop_controller_status0_fw_pid_config_kd, loop_controller_status[0].fw_pid.config.kd);
    BIND_STATUS(loop_controller_status0_fw_pid_config_kb, loop_controller_status[0].fw_pid.config.kb);
    BIND_STATUS(loop_controller_status0_fw_pid_config_alpha, loop_controller_status[0].fw_pid.config.alpha);
    BIND_STATUS(loop_controller_status0_fw_pid_config_beta, loop_controller_status[0].fw_pid.config.beta);
    BIND_STATUS(loop_controller_status0_fw_pid_config_gamma, loop_controller_status[0].fw_pid.config.gamma);
    BIND_STATUS(loop_controller_status0_fw_pid_limit_limit_max, loop_controller_status[0].fw_pid.limit.limit_max);
    BIND_STATUS(loop_controller_status0_fw_pid_limit_limit_min, loop_controller_status[0].fw_pid.limit.limit_min);
    BIND_STATUS(loop_controller_status0_fw_pid_status_proportional, loop_controller_status[0].fw_pid.status.proportional);
    BIND_STATUS(loop_controller_status0_fw_pid_status_integral, loop_controller_status[0].fw_pid.status.integral);
    BIND_STATUS(loop_controller_status0_fw_pid_status_derivative, loop_controller_status[0].fw_pid.status.derivative);
    BIND_STATUS(loop_controller_status0_fw_pid_status_derivative_f, loop_controller_status[0].fw_pid.status.derivative_f);
    BIND_STATUS(loop_controller_status0_fw_pid_status_out_temp, loop_controller_status[0].fw_pid.status.out_temp);
    BIND_STATUS(loop_controller_status0_fw_pid_status_out, loop_controller_status[0].fw_pid.status.out);
    BIND_STATUS(loop_controller_status0_fw_pid_status_err_k0, loop_controller_status[0].fw_pid.status.err_k0);
    BIND_STATUS(loop_controller_status0_fw_pid_status_err_k1, loop_controller_status[0].fw_pid.status.err_k1);
    BIND_STATUS(loop_controller_status0_fw_pid_status_err_k2, loop_controller_status[0].fw_pid.status.err_k2);
    BIND_STATUS(loop_controller_status0_fw_pid_status_saturation, loop_controller_status[0].fw_pid.status.saturation);
    BIND_STATUS(loop_controller_status0_speed_pid_dt, loop_controller_status[0].speed_pid.dt);
    BIND_STATUS(loop_controller_status0_speed_pid_config_kp, loop_controller_status[0].speed_pid.config.kp);
    BIND_STATUS(loop_controller_status0_speed_pid_config_ki, loop_controller_status[0].speed_pid.config.ki);
    BIND_STATUS(loop_controller_status0_speed_pid_config_kd, loop_controller_status[0].speed_pid.config.kd);
    BIND_STATUS(loop_controller_status0_speed_pid_config_kb, loop_controller_status[0].speed_pid.config.kb);
    BIND_STATUS(loop_controller_status0_speed_pid_config_alpha, loop_controller_status[0].speed_pid.config.alpha);
    BIND_STATUS(loop_controller_status0_speed_pid_config_beta, loop_controller_status[0].speed_pid.config.beta);
    BIND_STATUS(loop_controller_status0_speed_pid_config_gamma, loop_controller_status[0].speed_pid.config.gamma);
    BIND_STATUS(loop_controller_status0_speed_pid_limit_limit_max, loop_controller_status[0].speed_pid.limit.limit_max);
    BIND_STATUS(loop_controller_status0_speed_pid_limit_limit_min, loop_controller_status[0].speed_pid.limit.limit_min);
    BIND_STATUS(loop_controller_status0_speed_pid_status_proportional, loop_controller_status[0].speed_pid.status.proportional);
    BIND_STATUS(loop_controller_status0_speed_pid_status_integral, loop_controller_status[0].speed_pid.status.integral);
    BIND_STATUS(loop_controller_status0_speed_pid_status_derivative, loop_controller_status[0].speed_pid.status.derivative);
    BIND_STATUS(loop_controller_status0_speed_pid_status_derivative_f, loop_controller_status[0].speed_pid.status.derivative_f);
    BIND_STATUS(loop_controller_status0_speed_pid_status_out_temp, loop_controller_status[0].speed_pid.status.out_temp);
    BIND_STATUS(loop_controller_status0_speed_pid_status_out, loop_controller_status[0].speed_pid.status.out);
    BIND_STATUS(loop_controller_status0_speed_pid_status_err_k0, loop_controller_status[0].speed_pid.status.err_k0);
    BIND_STATUS(loop_controller_status0_speed_pid_status_err_k1, loop_controller_status[0].speed_pid.status.err_k1);
    BIND_STATUS(loop_controller_status0_speed_pid_status_err_k2, loop_controller_status[0].speed_pid.status.err_k2);
    BIND_STATUS(loop_controller_status0_speed_pid_status_saturation, loop_controller_status[0].speed_pid.status.saturation);
    BIND_STATUS(loop_controller_status0_pos_pid_dt, loop_controller_status[0].pos_pid.dt);
    BIND_STATUS(loop_controller_status0_pos_pid_config_kp, loop_controller_status[0].pos_pid.config.kp);
    BIND_STATUS(loop_controller_status0_pos_pid_config_ki, loop_controller_status[0].pos_pid.config.ki);
    BIND_STATUS(loop_controller_status0_pos_pid_config_kd, loop_controller_status[0].pos_pid.config.kd);
    BIND_STATUS(loop_controller_status0_pos_pid_config_kb, loop_controller_status[0].pos_pid.config.kb);
    BIND_STATUS(loop_controller_status0_pos_pid_config_alpha, loop_controller_status[0].pos_pid.config.alpha);
    BIND_STATUS(loop_controller_status0_pos_pid_config_beta, loop_controller_status[0].pos_pid.config.beta);
    BIND_STATUS(loop_controller_status0_pos_pid_config_gamma, loop_controller_status[0].pos_pid.config.gamma);
    BIND_STATUS(loop_controller_status0_pos_pid_limit_limit_max, loop_controller_status[0].pos_pid.limit.limit_max);
    BIND_STATUS(loop_controller_status0_pos_pid_limit_limit_min, loop_controller_status[0].pos_pid.limit.limit_min);
    BIND_STATUS(loop_controller_status0_pos_pid_status_proportional, loop_controller_status[0].pos_pid.status.proportional);
    BIND_STATUS(loop_controller_status0_pos_pid_status_integral, loop_controller_status[0].pos_pid.status.integral);
    BIND_STATUS(loop_controller_status0_pos_pid_status_derivative, loop_controller_status[0].pos_pid.status.derivative);
    BIND_STATUS(loop_controller_status0_pos_pid_status_derivative_f, loop_controller_status[0].pos_pid.status.derivative_f);
    BIND_STATUS(loop_controller_status0_pos_pid_status_out_temp, loop_controller_status[0].pos_pid.status.out_temp);
    BIND_STATUS(loop_controller_status0_pos_pid_status_out, loop_controller_status[0].pos_pid.status.out);
    BIND_STATUS(loop_controller_status0_pos_pid_status_err_k0, loop_controller_status[0].pos_pid.status.err_k0);
    BIND_STATUS(loop_controller_status0_pos_pid_status_err_k1, loop_controller_status[0].pos_pid.status.err_k1);
    BIND_STATUS(loop_controller_status0_pos_pid_status_err_k2, loop_controller_status[0].pos_pid.status.err_k2);
    BIND_STATUS(loop_controller_status0_pos_pid_status_saturation, loop_controller_status[0].pos_pid.status.saturation);
    BIND_STATUS(loop_controller_status0_force_compliance_dt, loop_controller_status[0].force_compliance.dt);
    BIND_STATUS(loop_controller_status0_force_compliance_config_virtual_mass, loop_controller_status[0].force_compliance.config.virtual_mass);
    BIND_STATUS(loop_controller_status0_force_compliance_config_virtual_damping, loop_controller_status[0].force_compliance.config.virtual_damping);
    BIND_STATUS(loop_controller_status0_force_compliance_config_virtual_stiffness, loop_controller_status[0].force_compliance.config.virtual_stiffness);
    BIND_STATUS(loop_controller_status0_force_compliance_limit_max_position, loop_controller_status[0].force_compliance.limit.max_position);
    BIND_STATUS(loop_controller_status0_force_compliance_limit_min_position, loop_controller_status[0].force_compliance.limit.min_position);
    BIND_STATUS(loop_controller_status0_force_compliance_limit_max_velocity, loop_controller_status[0].force_compliance.limit.max_velocity);
    BIND_STATUS(loop_controller_status0_force_compliance_limit_min_velocity, loop_controller_status[0].force_compliance.limit.min_velocity);
    BIND_STATUS(loop_controller_status0_force_compliance_limit_max_acceleration, loop_controller_status[0].force_compliance.limit.max_acceleration);
    BIND_STATUS(loop_controller_status0_force_compliance_limit_min_acceleration, loop_controller_status[0].force_compliance.limit.min_acceleration);
    BIND_STATUS(loop_controller_status0_force_compliance_limit_max_force, loop_controller_status[0].force_compliance.limit.max_force);
    BIND_STATUS(loop_controller_status0_force_compliance_limit_min_force, loop_controller_status[0].force_compliance.limit.min_force);
    BIND_STATUS(loop_controller_status0_force_compliance_limit_max_pos_err, loop_controller_status[0].force_compliance.limit.max_pos_err);
    BIND_STATUS(loop_controller_status0_force_compliance_limit_max_vel_err, loop_controller_status[0].force_compliance.limit.max_vel_err);
    BIND_STATUS(loop_controller_status0_force_compliance_limit_max_acc_err, loop_controller_status[0].force_compliance.limit.max_acc_err);
    BIND_STATUS(loop_controller_status0_force_compliance_status_force_ff, loop_controller_status[0].force_compliance.status.force_ff);
    BIND_STATUS(loop_controller_status0_force_compliance_status_force_err, loop_controller_status[0].force_compliance.status.force_err);
    BIND_STATUS(loop_controller_status0_force_compliance_status_pos_err, loop_controller_status[0].force_compliance.status.pos_err);
    BIND_STATUS(loop_controller_status0_force_compliance_status_vel_err, loop_controller_status[0].force_compliance.status.vel_err);
    BIND_STATUS(loop_controller_status0_force_compliance_status_acc_err, loop_controller_status[0].force_compliance.status.acc_err);
    BIND_STATUS(loop_controller_status0_force_compliance_status_x_pos, loop_controller_status[0].force_compliance.status.x_pos);
    BIND_STATUS(loop_controller_status0_force_compliance_status_base_pos, loop_controller_status[0].force_compliance.status.base_pos);
    BIND_STATUS(loop_controller_status0_force_compliance_status_x_vel, loop_controller_status[0].force_compliance.status.x_vel);
    BIND_STATUS(loop_controller_status0_force_compliance_status_x_acc, loop_controller_status[0].force_compliance.status.x_acc);
    BIND_STATUS(loop_controller_status0_force_compliance_status_force_p, loop_controller_status[0].force_compliance.status.force_p);
    BIND_STATUS(loop_controller_status0_force_compliance_status_force_d, loop_controller_status[0].force_compliance.status.force_d);
    BIND_STATUS(loop_controller_status0_force_compliance_status_force_m, loop_controller_status[0].force_compliance.status.force_m);
    BIND_STATUS(loop_controller_status0_force_compliance_status_force_cmd, loop_controller_status[0].force_compliance.status.force_cmd);

    /* 0x2108: read-only FOC_Status_t. */
    BIND_STATUS(foc_status0_run_state, foc_status[0].run_state);
    BIND_STATUS_ENUM(foc_status0_op_area, foc_status[0].op_area);
    BIND_STATUS(foc_status0_u_ab_Alpha, foc_status[0].u_ab.Alpha);
    BIND_STATUS(foc_status0_u_ab_Beta, foc_status[0].u_ab.Beta);
    BIND_STATUS(foc_status0_u_ab_last_Alpha, foc_status[0].u_ab_last.Alpha);
    BIND_STATUS(foc_status0_u_ab_last_Beta, foc_status[0].u_ab_last.Beta);
    BIND_STATUS(foc_status0_i_ab_Alpha, foc_status[0].i_ab.Alpha);
    BIND_STATUS(foc_status0_i_ab_Beta, foc_status[0].i_ab.Beta);
    BIND_STATUS(foc_status0_i_ab_last_Alpha, foc_status[0].i_ab_last.Alpha);
    BIND_STATUS(foc_status0_i_ab_last_Beta, foc_status[0].i_ab_last.Beta);
    BIND_STATUS(foc_status0_u_dq_D, foc_status[0].u_dq.D);
    BIND_STATUS(foc_status0_u_dq_Q, foc_status[0].u_dq.Q);
    BIND_STATUS(foc_status0_u_dq_last_D, foc_status[0].u_dq_last.D);
    BIND_STATUS(foc_status0_u_dq_last_Q, foc_status[0].u_dq_last.Q);
    BIND_STATUS(foc_status0_i_dq_D, foc_status[0].i_dq.D);
    BIND_STATUS(foc_status0_i_dq_Q, foc_status[0].i_dq.Q);
    BIND_STATUS(foc_status0_i_dq_last_D, foc_status[0].i_dq_last.D);
    BIND_STATUS(foc_status0_i_dq_last_Q, foc_status[0].i_dq_last.Q);
    BIND_STATUS(foc_status0_u_dq_ref_D, foc_status[0].u_dq_ref.D);
    BIND_STATUS(foc_status0_u_dq_ref_Q, foc_status[0].u_dq_ref.Q);
    BIND_STATUS(foc_status0_i_dq_ref_D, foc_status[0].i_dq_ref.D);
    BIND_STATUS(foc_status0_i_dq_ref_Q, foc_status[0].i_dq_ref.Q);
    BIND_STATUS(foc_status0_u_dq_feed_forward_D, foc_status[0].u_dq_feed_forward.D);
    BIND_STATUS(foc_status0_u_dq_feed_forward_Q, foc_status[0].u_dq_feed_forward.Q);
    BIND_STATUS(foc_status0_u_dq_out_raw_D, foc_status[0].u_dq_out_raw.D);
    BIND_STATUS(foc_status0_u_dq_out_raw_Q, foc_status[0].u_dq_out_raw.Q);
    BIND_STATUS(foc_status0_u_dq_out_D, foc_status[0].u_dq_out.D);
    BIND_STATUS(foc_status0_u_dq_out_Q, foc_status[0].u_dq_out.Q);
    BIND_STATUS(foc_status0_u_ab_out_raw_Alpha, foc_status[0].u_ab_out_raw.Alpha);
    BIND_STATUS(foc_status0_u_ab_out_raw_Beta, foc_status[0].u_ab_out_raw.Beta);
    BIND_STATUS(foc_status0_u_ab_out_Alpha, foc_status[0].u_ab_out.Alpha);
    BIND_STATUS(foc_status0_u_ab_out_Beta, foc_status[0].u_ab_out.Beta);
    BIND_STATUS(foc_status0_us_out_raw, foc_status[0].us_out_raw);
    BIND_STATUS(foc_status0_us_out_raw_f, foc_status[0].us_out_raw_f);
    BIND_STATUS(foc_status0_us_out, foc_status[0].us_out);
    BIND_STATUS(foc_status0_us_out_f, foc_status[0].us_out_f);
    BIND_STATUS(foc_status0_us_out_rate_raw, foc_status[0].us_out_rate_raw);
    BIND_STATUS(foc_status0_us_out_rate, foc_status[0].us_out_rate);
    BIND_STATUS(foc_status0_rads_encoder, foc_status[0].rads_encoder);
    BIND_STATUS(foc_status0_rads_encoder_offset, foc_status[0].rads_encoder_offset);
    BIND_STATUS(foc_status0_theta_encoder, foc_status[0].theta_encoder);
    BIND_STATUS(foc_status0_we_encoder, foc_status[0].we_encoder);
    BIND_STATUS(foc_status0_theta_hall, foc_status[0].theta_hall);
    BIND_STATUS(foc_status0_we_hall, foc_status[0].we_hall);
    BIND_STATUS(foc_status0_theta_observer, foc_status[0].theta_observer);
    BIND_STATUS(foc_status0_we_observer, foc_status[0].we_observer);
    BIND_STATUS(foc_status0_theta_hfi, foc_status[0].theta_hfi);
    BIND_STATUS(foc_status0_we_hfi, foc_status[0].we_hfi);
    BIND_STATUS(foc_status0_theta_openloop, foc_status[0].theta_openloop);
    BIND_STATUS(foc_status0_we_openloop, foc_status[0].we_openloop);
    BIND_STATUS(foc_status0_theta, foc_status[0].theta);
    BIND_STATUS(foc_status0_theta_comp, foc_status[0].theta_comp);
    BIND_STATUS(foc_status0_we, foc_status[0].we);
    BIND_STATUS(foc_status0_sin_cos_sin, foc_status[0].sin_cos.sin);
    BIND_STATUS(foc_status0_sin_cos_cos, foc_status[0].sin_cos.cos);
    BIND_STATUS(foc_status0_i_dq_delta_D, foc_status[0].i_dq_delta.D);
    BIND_STATUS(foc_status0_i_dq_delta_Q, foc_status[0].i_dq_delta.Q);
    BIND_STATUS(foc_status0_i_dq_delta_f_D, foc_status[0].i_dq_delta_f.D);
    BIND_STATUS(foc_status0_i_dq_delta_f_Q, foc_status[0].i_dq_delta_f.Q);
    BIND_STATUS(foc_status0_u_dq_rl_D, foc_status[0].u_dq_rl.D);
    BIND_STATUS(foc_status0_u_dq_rl_Q, foc_status[0].u_dq_rl.Q);
    BIND_STATUS(foc_status0_u_dq_cross_D, foc_status[0].u_dq_cross.D);
    BIND_STATUS(foc_status0_u_dq_cross_Q, foc_status[0].u_dq_cross.Q);
    BIND_STATUS(foc_status0_u_bemf, foc_status[0].u_bemf);
    BIND_STATUS(foc_status0_us_max, foc_status[0].us_max);
    BIND_STATUS(foc_status0_uo_max, foc_status[0].uo_max);
    BIND_STATUS(foc_status0_us, foc_status[0].us);
    BIND_STATUS(foc_status0_is, foc_status[0].is);
    BIND_STATUS(foc_status0_us_duty, foc_status[0].us_duty);
    BIND_STATUS(foc_status0_is_duty, foc_status[0].is_duty);
    BIND_STATUS(foc_status0_align_state, foc_status[0].align_state);
    BIND_STATUS(foc_status0_align_tick, foc_status[0].align_tick);
    BIND_STATUS(foc_status0_align_retry_count, foc_status[0].align_retry_count);
    BIND_STATUS(foc_status0_align_initial, foc_status[0].align_initial);
    BIND_STATUS(foc_status0_align_offset_candidate, foc_status[0].align_offset_candidate);
    BIND_STATUS(foc_status0_align_correction, foc_status[0].align_correction);
    BIND_STATUS(foc_status0_align_dir_detected, foc_status[0].align_dir_detected);
    BIND_STATUS(foc_status0_dt_compensation_value, foc_status[0].dt_compensation_value);

    /* 0x2109: read-only HFI_Status_t. */
    BIND_STATUS(hfi_status0_div_cnt, hfi_status[0].div_cnt);
    BIND_STATUS(hfi_status0_sign, hfi_status[0].sign);
    BIND_STATUS(hfi_status0_sign_last, hfi_status[0].sign_last);
    BIND_STATUS(hfi_status0_enable_pole_identify, hfi_status[0].enable_pole_identify);
    BIND_STATUS_ENUM(hfi_status0_pole_identify_status, hfi_status[0].pole_identify_status);
    BIND_STATUS(hfi_status0_pole_identify_sign, hfi_status[0].pole_identify_sign);
    BIND_STATUS(hfi_status0_pole_identify_stable_cnt, hfi_status[0].pole_identify_stable_cnt);
    BIND_STATUS(hfi_status0_pole_identify_pulse_cnt, hfi_status[0].pole_identify_pulse_cnt);
    BIND_STATUS(hfi_status0_pole_identify_interval_cnt, hfi_status[0].pole_identify_interval_cnt);
    BIND_STATUS(hfi_status0_pole_identify_cnt, hfi_status[0].pole_identify_cnt);
    BIND_STATUS(hfi_status0_pole_identify_retry_cnt, hfi_status[0].pole_identify_retry_cnt);
    BIND_STATUS(hfi_status0_udq_excitation_D, hfi_status[0].udq_excitation.D);
    BIND_STATUS(hfi_status0_udq_excitation_Q, hfi_status[0].udq_excitation.Q);
    BIND_STATUS(hfi_status0_uab_base_Alpha, hfi_status[0].uab_base.Alpha);
    BIND_STATUS(hfi_status0_uab_base_Beta, hfi_status[0].uab_base.Beta);
    BIND_STATUS(hfi_status0_uab_response_Alpha, hfi_status[0].uab_response.Alpha);
    BIND_STATUS(hfi_status0_uab_response_Beta, hfi_status[0].uab_response.Beta);
    BIND_STATUS(hfi_status0_iab_base_Alpha, hfi_status[0].iab_base.Alpha);
    BIND_STATUS(hfi_status0_iab_base_Beta, hfi_status[0].iab_base.Beta);
    BIND_STATUS(hfi_status0_iab_response_Alpha, hfi_status[0].iab_response.Alpha);
    BIND_STATUS(hfi_status0_iab_response_Beta, hfi_status[0].iab_response.Beta);
    BIND_STATUS(hfi_status0_iab_response_last_Alpha, hfi_status[0].iab_response_last.Alpha);
    BIND_STATUS(hfi_status0_iab_response_last_Beta, hfi_status[0].iab_response_last.Beta);
    BIND_STATUS(hfi_status0_iab_input_Alpha, hfi_status[0].iab_input.Alpha);
    BIND_STATUS(hfi_status0_iab_input_Beta, hfi_status[0].iab_input.Beta);
    BIND_STATUS(hfi_status0_id_positive_sum, hfi_status[0].id_positive_sum);
    BIND_STATUS(hfi_status0_id_negative_sum, hfi_status[0].id_negative_sum);
    BIND_STATUS(hfi_status0_id_positive, hfi_status[0].id_positive);
    BIND_STATUS(hfi_status0_id_negative, hfi_status[0].id_negative);
    BIND_STATUS(hfi_status0_theta, hfi_status[0].theta);

#undef BIND_STATUS
#undef BIND_STATUS_ENUM

#undef BIND_CONFIG
#undef BIND_ENUM
    return od_freeze(&od_dictionary, first_missing);
}
