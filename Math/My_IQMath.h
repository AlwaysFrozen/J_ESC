#ifndef __MY_IQMATH_H__
#define __MY_IQMATH_H__

#include "stdint.h"


#define GLOBAL_Q   15

//Q格式的运算
//1 > 定点加减法：须转换成相同的Q格式才能加减
//2 > 定点乘法：不同Q格式的数据相乘，相当于Q值相加，即Q15数据乘以Q10数据后的结果是Q25格式的数据
//3 > 定点除法：不同Q格式的数据相除，相当于Q值相减
//4 > 定点左移：左移相当于Q值增加
//5 > 定点右移：右移相当于Q减少
//Q格式的应用格式
//实际应用中，浮点运算大都时候都是既有整数部分，也有小数部分的。所以要选择一个适当的定标格式才能更好的处理运算。一般用如下两种方法：
//1 > 使用时使用适中的定标，既可以表示一定的整数复位也可以表示小数复位，如对于2812的32位系统，使用Q15格式，可表示－65536～65535.999969482区间内的数据。
//2 > 全部采用小数，这样因为小数之间相乘永远是小数，永远不会溢出。取一个极限最大值（最好使用2的n次幂），转换成x / Max的小数（如果Max是取的2的 n次幂，就可以使用移位代替除法）。

#define _IQ30(A) (long) ((A) * 1073741824L)
#define _IQ29(A) (long) ((A) * 536870912L)
#define _IQ28(A) (long) ((A) * 268435456L)
#define _IQ27(A) (long) ((A) * 134217728L)
#define _IQ26(A) (long) ((A) * 67108864L)
#define _IQ25(A) (long) ((A) * 33554432L)
#define _IQ24(A) (long) ((A) * 16777216L)
#define _IQ23(A) (long) ((A) * 8388608L)
#define _IQ22(A) (long) ((A) * 4194304L)
#define _IQ21(A) (long) ((A) * 2097152L)
#define _IQ20(A) (long) ((A) * 1048576L)
#define _IQ19(A) (long) ((A) * 524288L)
#define _IQ18(A) (long) ((A) * 262144L)
#define _IQ17(A) (long) ((A) * 131072L)
#define _IQ16(A) (long) ((A) * 65536L)
#define _IQ15(A) (long) ((A) * 32768L)
#define _IQ14(A) (long) ((A) * 16384L)
#define _IQ13(A) (long) ((A) * 8192L)
#define _IQ12(A) (long) ((A) * 4096L)
#define _IQ11(A) (long) ((A) * 2048L)
#define _IQ10(A) (long) ((A) * 1024L)
#define _IQ9(A) (long) ((A) * 512L)
#define _IQ8(A) (long) ((A) * 256L)
#define _IQ7(A) (long) ((A) * 128L)
#define _IQ6(A) (long) ((A) * 64L)
#define _IQ5(A) (long) ((A) * 32L)
#define _IQ4(A) (long) ((A) * 16L)
#define _IQ3(A) (long) ((A) * 8L)
#define _IQ2(A) (long) ((A) * 4L)
#define _IQ1(A) (long) ((A) * 2L)

#define _2P30   (1073741824L)
#define _2P29   (536870912L)
#define _2P28   (268435456L)
#define _2P27   (134217728L)
#define _2P26   (67108864L)
#define _2P25   (33554432L)
#define _2P24   (16777216L)
#define _2P23   (8388608L)
#define _2P22   (4194304L)
#define _2P21   (2097152L)
#define _2P20   (1048576L)
#define _2P19   (524288L)
#define _2P18   (262144L)
#define _2P17   (131072L)
#define _2P16   (65536L)
#define _2P15   (32768L)
#define _2P14   (16384L)
#define _2P13   (8192L)
#define _2P12   (4096L)
#define _2P11   (2048L)
#define _2P10   (1024L)
#define _2P9    (512L)
#define _2P8    (256L)
#define _2P7    (128L)
#define _2P6    (64L)
#define _2P5    (32L)
#define _2P4    (16L)
#define _2P3    (8L)
#define _2P2    (4L)
#define _2P1    (2L)


#if GLOBAL_Q == 30
#define IQ(A)   _IQ30(A)
#define _2PN    _2P30
#elif GLOBAL_Q == 29
#define IQ(A)   _IQ29(A)
#define _2PN    _2P29
#elif GLOBAL_Q == 28
#define IQ(A)   _IQ28(A)
#define _2PN    _2P28
#elif GLOBAL_Q == 27
#define IQ(A)   _IQ27(A)
#define _2PN    _2P27
#elif GLOBAL_Q == 26
#define IQ(A)   _IQ26(A)
#define _2PN    _2P26
#elif GLOBAL_Q == 25
#define IQ(A)   _IQ25(A)
#define _2PN    _2P25
#elif GLOBAL_Q == 24
#define IQ(A)   _IQ24(A)
#define _2PN    _2P24
#elif GLOBAL_Q == 23
#define IQ(A)   _IQ23(A)
#define _2PN    _2P23
#elif GLOBAL_Q == 22
#define IQ(A)   _IQ22(A)
#define _2PN    _2P22
#elif GLOBAL_Q == 21
#define IQ(A)   _IQ21(A)
#define _2PN    _2P21
#elif GLOBAL_Q == 20
#define IQ(A)   _IQ20(A)
#define _2PN    _2P20
#elif GLOBAL_Q == 19
#define IQ(A)   _IQ19(A)
#define _2PN    _2P19
#elif GLOBAL_Q == 18
#define IQ(A)   _IQ18(A)
#define _2PN    _2P18
#elif GLOBAL_Q == 17
#define IQ(A)   _IQ17(A)
#define _2PN    _2P17
#elif GLOBAL_Q == 16
#define IQ(A)   _IQ16(A)
#define _2PN    _2P16
#elif GLOBAL_Q == 15
#define IQ(A)   _IQ15(A)
#define _2PN    _2P15
#elif GLOBAL_Q == 14
#define IQ(A)   _IQ14(A)
#define _2PN    _2P14
#elif GLOBAL_Q == 13
#define IQ(A)   _IQ13(A)
#define _2PN    _2P13
#elif GLOBAL_Q == 12
#define IQ(A)   _IQ12(A)
#define _2PN    _2P12
#elif GLOBAL_Q == 11
#define IQ(A)   _IQ11(A)
#define _2PN    _2P11
#elif GLOBAL_Q == 10
#define IQ(A)   _IQ10(A)
#define _2PN    _2P10
#elif GLOBAL_Q == 9
#define IQ(A)   _IQ9(A)
#define _2PN    _2P9
#elif GLOBAL_Q == 8
#define IQ(A)   _IQ8(A)
#define _2PN    _2P8
#elif GLOBAL_Q == 7
#define IQ(A)   _IQ7(A)
#define _2PN    _2P7
#elif GLOBAL_Q == 6
#define IQ(A)   _IQ6(A)
#define _2PN    _2P6
#elif GLOBAL_Q == 5
#define IQ(A)   _IQ5(A)
#define _2PN    _2P5
#elif GLOBAL_Q == 4
#define IQ(A)   _IQ4(A)
#define _2PN    _2P4
#elif GLOBAL_Q == 3
#define IQ(A)   _IQ3(A)
#define _2PN    _2P3
#elif GLOBAL_Q == 2
#define IQ(A)   _IQ2(A)
#define _2PN    _2P2
#elif GLOBAL_Q == 1
#define IQ(A)   _IQ1(A)
#define _2PN    _2P1
#endif

#define IQ_mpy(A,B)     (((A) * (B)) >> GLOBAL_Q)
#define IQ_div2(A)      ((A)>>1)
#define IQ_mpy2(A)      ((A)<<1)
#define IQ_div(A,B)     (((int64_t)A<<15)/B)
#define IQ_abs(A)       (((A)>=0)?(A):-(A))
#define IQ_min(A,B)     (((A)<=(B))?(A):(B))
#define IQ_max(A,B)     (((A)>=(B))?(A):(B))


#define IQ_constrain(amt,low,high)      ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define IQ_lpf(X,Y,A)                   (Y = (Y + IQ_mpy(IQ(A),(X - Y))))
#define IQ_lpf1(X,Y,A)                  (Y = (Y + IQ_mpy((A),(X - Y))))

#define DEG_TO_RAD(x)   (x / 57.295779f)
#define RAD_TO_DEG(x)   (_IQmpy(IQ(x),IQ(57.295779f)))

// #define IQ_SQRT3        IQ(1.73205080757f)
// #define IQ_2_SQRT3      IQ(1.15470053838f)
// #define IQ_1_SQRT3      IQ(0.57735026919f)
// #define IQ_SQRT3_2      IQ(0.86602540378f)
// #define IQ_SQRT2        IQ(1.41421356237f)

// #define IQ_PI_6         IQ(0.52359877559f)
// #define IQ_PI_3         IQ(1.04719755119f)
// #define IQ_PI_2         IQ(1.57079632679f)
// #define IQ_2PI_3        IQ(2.09439510239f)
// #define IQ_5PI_6        IQ(2.61799387799f)
// #define IQ_PI           IQ(3.14159265359f)
// #define IQ_7PI_6        IQ(3.66519142918f)
// #define IQ_3PI_2        IQ(4.71238898038f)
// #define IQ_11PI_6       IQ(5.75958653158f)
// #define IQ_2PI          IQ(6.28318530718f)

// #define IQ_e            IQ(2.71828182845f)



#define IQ_SQRT3        (56755L)
#define IQ_2_SQRT3      (37837L)
#define IQ_1_SQRT3      (18918L)
#define IQ_SQRT3_2      (28377L)
#define IQ_SQRT2        (46340L)

#define IQ_PI_6         (17157L)
#define IQ_PI_3         (34314L)
#define IQ_PI_2         (51471L)
#define IQ_2PI_3        (68629L)
#define IQ_5PI_6        (85786L)
#define IQ_PI           (102943L)
#define IQ_7PI_6        (120100L)
#define IQ_3PI_2        (154415L)
#define IQ_11PI_6       (188730L)
#define IQ_2PI          (205887L)

#define IQ_e            (89072L)

#define RAD_PI_6        (5461L)
#define RAD_PI_3        (10922L)
#define RAD_PI_2        (16384L)
#define RAD_2PI_3       (21845L)
#define RAD_5PI_6       (27306L)
#define RAD_PI          (32768L)
#define RAD_7PI_6       (38229L)
#define RAD_3PI_2       (49152L)
#define RAD_11PI_6      (60074L)
#define RAD_2PI         (65536L)

void IQSin_Cos_Cale(int32_t IQAngle,int32_t * IQSin, int32_t* IQCos);
int32_t IQAtan_Cale(int32_t Alpha, int32_t Beta);
uint32_t IQSqrt(uint32_t M);
uint32_t sqrt_q15(uint32_t x);



typedef struct My_Math
{
    int32_t out_temp;
    int32_t limit_p;
    int32_t limit_n;
    int32_t rise_ms;
    int32_t fall_ms;
    int32_t ts_ms;
    int32_t delta;
    int32_t delta_limit_p;
    int32_t delta_limit_n;
}IQ_Ramp_t;
void IQ_Ramp_Init(IQ_Ramp_t *ramp, int32_t limit_p, int32_t limit_n, int32_t rise_ms, int32_t fall_ms, int32_t ts_ms);
void IQ_Ramp_Update(IQ_Ramp_t *ramp, int32_t in, int32_t *out);

#endif
