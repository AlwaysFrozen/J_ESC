#include "../../motor_control/config/motor_config_policy.h"
#define CHECK(x) do { if (!(x)) return __LINE__; } while (0)
int main(void)
{
    union { uint32_t u; float f; } nan = {.u=0x7fc00000U}, inf = {.u=0x7f800000U};
    CHECK(!motor_policy_speed_trip(0, 6.28f));
    CHECK(!motor_policy_speed_trip(6.28f, 6.28f));
    CHECK(motor_policy_speed_trip(6.29f, 6.28f));
    CHECK(motor_policy_speed_trip(-6.29f, 6.28f));
    CHECK(motor_policy_speed_trip(nan.f, 6.28f));
    CHECK(motor_policy_speed_trip(0, 0));
    CHECK(motor_policy_speed_trip(0, inf.f));
    CHECK(motor_policy_speed_trip(0, -1));
    CHECK(motor_policy_speed_trip(-inf.f, 6.28f));
    CHECK(motor_policy_thermal_valid(1, 3, 0.5f, 10));
    CHECK(!motor_policy_thermal_valid(0, 0, 0, 0));
    CHECK(!motor_policy_thermal_valid(1, 1, 1, 10));
    CHECK(!motor_policy_thermal_valid(1, 3, 0, 10));
    CHECK(!motor_policy_thermal_valid(1, 3, 1, -1));
    CHECK(!motor_policy_thermal_valid(nan.f, 3, 1, 10));
    CHECK(!motor_policy_thermal_valid(1, inf.f, 1, 10));
    return 0;
}
