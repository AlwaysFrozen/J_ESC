#include "My_IQMath.h"
#include <math.h>


float IQ_mpy_to_f32(float a, float b)
{
    return (float)(IQ(a) * IQ(b)) / (_2PN * _2PN);
}

float IQ_add_to_f32(float a, float b)
{
    return (float)(IQ(a) + IQ(b)) / _2PN;
}

float IQ_sub_to_f32(float a, float b)
{
    return (float)(IQ(a) - IQ(b)) / _2PN;
}


const int16_t IQSin_Cos_Table[256] = 
{
    0x0000, 0x00C9, 0x0192, 0x025B, 0x0324, 0x03ED, 0x04B6, 0x057F,
    0x0648, 0x0711, 0x07D9, 0x08A2, 0x096A, 0x0A33, 0x0AFB, 0x0BC4,
    0x0C8C, 0x0D54, 0x0E1C, 0x0EE3, 0x0FAB, 0x1072, 0x113A, 0x1201,
    0x12C8, 0x138F, 0x1455, 0x151C, 0x15E2, 0x16A8, 0x176E, 0x1833,
    0x18F9, 0x19BE, 0x1A82, 0x1B47, 0x1C0B, 0x1CCF, 0x1D93, 0x1E57,
    0x1F1A, 0x1FDD, 0x209F, 0x2161, 0x2223, 0x22E5, 0x23A6, 0x2467,
    0x2528, 0x25E8, 0x26A8, 0x2767, 0x2826, 0x28E5, 0x29A3, 0x2A61,
    0x2B1F, 0x2BDC, 0x2C99, 0x2D55, 0x2E11, 0x2ECC, 0x2F87, 0x3041,
    0x30FB, 0x31B5, 0x326E, 0x3326, 0x33DF, 0x3496, 0x354D, 0x3604,
    0x36BA, 0x376F, 0x3824, 0x38D9, 0x398C, 0x3A40, 0x3AF2, 0x3BA5,
    0x3C56, 0x3D07, 0x3DB8, 0x3E68, 0x3F17, 0x3FC5, 0x4073, 0x4121,
    0x41CE, 0x427A, 0x4325, 0x43D0, 0x447A, 0x4524, 0x45CD, 0x4675,
    0x471C, 0x47C3, 0x4869, 0x490F, 0x49B4, 0x4A58, 0x4AFB, 0x4B9D,
    0x4C3F, 0x4CE0, 0x4D81, 0x4E20, 0x4EBF, 0x4F5D, 0x4FFB, 0x5097,
    0x5133, 0x51CE, 0x5268, 0x5302, 0x539B, 0x5432, 0x54C9, 0x5560,
    0x55F5, 0x568A, 0x571D, 0x57B0, 0x5842, 0x58D3, 0x5964, 0x59F3,
    0x5A82, 0x5B0F, 0x5B9C, 0x5C28, 0x5CB3, 0x5D3E, 0x5DC7, 0x5E4F,
    0x5ED7, 0x5F5D, 0x5FE3, 0x6068, 0x60EB, 0x616E, 0x61F0, 0x6271,
    0x62F1, 0x6370, 0x63EE, 0x646C, 0x64E8, 0x6563, 0x65DD, 0x6656,
    0x66CF, 0x6746, 0x67BC, 0x6832, 0x68A6, 0x6919, 0x698B, 0x69FD,
    0x6A6D, 0x6ADC, 0x6B4A, 0x6BB7, 0x6C23, 0x6C8E, 0x6CF8, 0x6D61,
    0x6DC9, 0x6E30, 0x6E96, 0x6EFB, 0x6F5E, 0x6FC1, 0x7022, 0x7083,
    0x70E2, 0x7140, 0x719D, 0x71F9, 0x7254, 0x72AE, 0x7307, 0x735E,
    0x73B5, 0x740A, 0x745F, 0x74B2, 0x7504, 0x7555, 0x75A5, 0x75F3,
    0x7641, 0x768D, 0x76D8, 0x7722, 0x776B, 0x77B3, 0x77FA, 0x783F,
    0x7884, 0x78C7, 0x7909, 0x794A, 0x7989, 0x79C8, 0x7A05, 0x7A41,
    0x7A7C, 0x7AB6, 0x7AEE, 0x7B26, 0x7B5C, 0x7B91, 0x7BC5, 0x7BF8,
    0x7C29, 0x7C59, 0x7C88, 0x7CB6, 0x7CE3, 0x7D0E, 0x7D39, 0x7D62,
    0x7D89, 0x7DB0, 0x7DD5, 0x7DFA, 0x7E1D, 0x7E3E, 0x7E5F, 0x7E7E,
    0x7E9C, 0x7EB9, 0x7ED5, 0x7EEF, 0x7F09, 0x7F21, 0x7F37, 0x7F4D,
    0x7F61, 0x7F74, 0x7F86, 0x7F97, 0x7FA6, 0x7FB4, 0x7FC1, 0x7FCD,
    0x7FD8, 0x7FE1, 0x7FE9, 0x7FF0, 0x7FF5, 0x7FF9, 0x7FFD, 0x7FFE
};

const int32_t IQAtan2_Table[256] = 
{
    0x00000000, 0x000000C9, 0x00000192, 0x0000025B, 0x00000324, 0x000003ED, 0x000004B6, 0x00000580,
    0x00000649, 0x00000712, 0x000008A7, 0x00000971, 0x00000A3B, 0x00000B05, 0x00000BD0, 0x00000C9B,
    0x00000D66, 0x00000E31, 0x00000EFD, 0x00000FC9, 0x00001095, 0x00001162, 0x00001218, 0x0000122F,
    0x000012FC, 0x000013CA, 0x00001498, 0x00001566, 0x00001635, 0x00001705, 0x000017D4, 0x000018A5,
    0x00001975, 0x00001A47, 0x00001B19, 0x00001BEB, 0x00001CBE, 0x00001D91, 0x00001E65, 0x00001F3A,
    0x0000200F, 0x000020E5, 0x000021BC, 0x00002294, 0x0000236C, 0x00002444, 0x0000251E, 0x000025F8,
    0x000026D4, 0x000027B0, 0x0000288C, 0x0000296A, 0x00002A49, 0x00002B28, 0x00002C08, 0x00002CEA,
    0x00002DCC, 0x00002EAF, 0x00002F94, 0x00003079, 0x00003160, 0x00003247, 0x00003330, 0x00003419,
    0x00003509, 0x00003571, 0x000036DE, 0x000037CD, 0x000038BD, 0x000039AE, 0x00003AA0, 0x00003B94,
    0x00003C8A, 0x00003D80, 0x00003E79, 0x00003F72, 0x0000406E, 0x0000416A, 0x00004269, 0x00004369,
    0x0000446A, 0x0000456E, 0x00004673, 0x0000477A, 0x00004882, 0x0000498D, 0x00004A99, 0x00004BA8,
    0x00004CB8, 0x00004DCA, 0x00004EDF, 0x00004FF5, 0x0000510E, 0x00005228, 0x00005345, 0x00005465,
    0x00005586, 0x000056AA, 0x000057D1, 0x000058FA, 0x00005A25, 0x00005B53, 0x00005C84, 0x00005DB8,
    0x00005EEE, 0x00006027, 0x00006163, 0x000062A2, 0x000063E4, 0x00006529, 0x00006671, 0x000067BD,
    0x0000690C, 0x00006A5E, 0x00006BB3, 0x00006D0D, 0x00006E69, 0x00006FCA, 0x0000712E, 0x00007296,
    0x00007403, 0x00007573, 0x000076E7, 0x00007860, 0x000079DD, 0x00007B5F, 0x00007CE5, 0x00007E70,
    0x00008000, 0x00008194, 0x0000832E, 0x000084CD, 0x00008671, 0x0000881A, 0x000089CA, 0x00008B7F,
    0x00008D39, 0x00008EFA, 0x000090C1, 0x0000928F, 0x00009463, 0x0000963D, 0x0000981F, 0x00009A08,
    0x00009BF7, 0x00009DEF, 0x00009FEE, 0x0000A1F5, 0x0000A404, 0x0000A61B, 0x0000A83B, 0x0000AA64,
    0x0000AC96, 0x0000AED1, 0x0000B116, 0x0000B365, 0x0000B5BE, 0x0000B822, 0x0000BA91, 0x0000BD0B,
    0x0000BF90, 0x0000C222, 0x0000C4C0, 0x0000C76A, 0x0000CA22, 0x0000CCE7, 0x0000CF88, 0x0000D29D,
    0x0000D58E, 0x0000D88E, 0x0000DB9F, 0x0000DEC0, 0x0000E1F3, 0x0000E538, 0x0000E88F, 0x0000EBFA,
    0x0000EF78, 0x0000F30B, 0x0000F6B4, 0x0000FA74, 0x0000FE4A, 0x00010239, 0x00010641, 0x00010A64,
    0x00010EA2, 0x000112FC, 0x00011774, 0x00011C0B, 0x00012C03, 0x0001259C, 0x00012A99, 0x00012FBC,
    0x00013504, 0x00013A76, 0x00014012, 0x000145DB, 0x00014BD3, 0x000151FD, 0x0001585A, 0x00015EEE,
    0x000165BC, 0x00016CC6, 0x00017411, 0x00017B9F, 0x00018376, 0x00018B98, 0x0001940A, 0x00019CD2,
    0x0001A5F5, 0x0001AF78, 0x0001B963, 0x0001C3BB, 0x0001CE88, 0x0001D9D2, 0x0001E5A3, 0x0001F205,
    0x0001FF01, 0x00020CA4, 0x00021AFB, 0x00022A15, 0x00023A02, 0x00024AD4, 0x00025C9F, 0x00026F7B,
    0x0002837F, 0x000298CA, 0x0002AF7C, 0x0002C7BA, 0x0002E1AE, 0x0002FD8A, 0x00031B84, 0x00033BDF,
    0x00035EE7, 0x000384F5, 0x0003AE73, 0x0003DBDD, 0x00040DCB, 0x000444F4, 0x00048236, 0x0004C6A6,
    0x0005139B, 0x00056AC9, 0x0005CE63, 0x00064144, 0x0006C740, 0x0007658D, 0x00082374, 0x00090B81,
    0x000A2D7F, 0x000BA246, 0x000D9338, 0x00104AD7, 0x00145E24, 0x001B28CC, 0x0028BDDA, 0x00517C7E
};

#define SIN_RAD         0x0300
#define U0_90           0x0000 
#define U90_180         0x0100
#define U180_270        0x0200
#define U270_360        0x0300
/*
    input range 0~65536 ->  0~2PI
    output range IQ(-1)~IQ(1)
*/
void IQSin_Cos_Cale(int32_t IQAngle,int32_t * IQSin, int32_t* IQCos)
{
    uint16_t hindex;
    hindex = IQAngle; //-32768--- 32767 +32768===0--65535
    hindex >>= 6;                   // 65536/64  ===1024/4=90度=256 0x01-0xFF  0X100  0X1FF  0X200 0X2FF    0X300  0X3FF

    switch (hindex & SIN_RAD) //  0X300  &   0x0000 -ff    0x0100  1ff   0X200    2ff   0x0300   3ff
    {
        case U0_90:
            *IQSin = IQSin_Cos_Table[(uint8_t)(hindex)]; // 0---255  ==0---32766
            *IQCos = IQSin_Cos_Table[(uint8_t)(0xFF - (uint8_t)(hindex))];
            break;

        case U90_180:
            *IQSin = IQSin_Cos_Table[(uint8_t)(0xFF - (uint8_t)(hindex))]; // 255---0 == 0---32766
            *IQCos = -IQSin_Cos_Table[(uint8_t)(hindex)];
            break;

        case U180_270:
            *IQSin = -IQSin_Cos_Table[(uint8_t)(hindex)];
            *IQCos = -IQSin_Cos_Table[(uint8_t)(0xFF - (uint8_t)(hindex))];
            break;

        case U270_360:
            *IQSin = -IQSin_Cos_Table[(uint8_t)(0xFF - (uint8_t)(hindex))];
            *IQCos = IQSin_Cos_Table[(uint8_t)(hindex)];
            break;

        default:
            break;
    }
}

/*
    input range IQ(-1)~IQ(1)
    output range 0~65535 -> 0~2PI
*/
int32_t IQAtan_Cale(int32_t Alpha, int32_t Beta)
{
    int32_t IQTan;
    int32_t IQAngle;
    int16_t i = 0;

    if (Alpha == 0 && Beta == 0)
    {
        return 0;
    }
    else if (Alpha == 0)
    {
        if (Beta > 0)
            return 16383;
        else
            return 49151;
    }
    else if (Beta == 0)
    {
        if (Alpha > 0)
            return 0;
        else
            return 32767;
    }
    else
    {
        IQTan = IQ_div(IQ_abs(Alpha), IQ_abs(Beta));

        if (IQAtan2_Table[i + 128] <= IQTan)
            i += 128;
        if (IQAtan2_Table[i + 64] <= IQTan)
            i += 64;
        if (IQAtan2_Table[i + 32] <= IQTan)
            i += 32;
        if (IQAtan2_Table[i + 16] <= IQTan)
            i += 16;
        if (IQAtan2_Table[i + 8] <= IQTan)
            i += 8;
        if (IQAtan2_Table[i + 4] <= IQTan)
            i += 4;
        if (IQAtan2_Table[i + 2] <= IQTan)
            i += 2;
        if (IQAtan2_Table[i + 1] <= IQTan)
            i += 1;
    }

    if (Alpha > 0)
    {
        if (Beta > 0)
            IQAngle = 255 - i;
        else
            IQAngle = i + 767;
    }
    else
    {
        if (Beta > 0)
            IQAngle = i + 255;
        else
            IQAngle = 767 - i;
    }
    IQAngle = IQAngle << 6;
    return IQAngle;
}

uint32_t IQSqrt(uint32_t M)
{
    uint32_t N, i, tmp, ttp;
    if (M == 0)
        return 0;
    N = 0;

    tmp = (M >> 30);

    M <<= 2;
    if (tmp > 1)
    {
        N++;
        tmp -= N;
    }
    for (i = 15; i > 0; i--)
    {
        N <<= 1;

        tmp <<= 2;
        tmp += (M >> 30);

        ttp = N;
        ttp = (ttp << 1) + 1;

        M <<= 2;
        if (tmp >= ttp)
        {
            tmp -= ttp;
            N++;
        }
    }
    return N;
}

uint32_t sqrt_q15(uint32_t x)
{
    uint32_t xr;  // result register
    uint32_t q2;  // scan-bit register
    uint32_t f;   // flag (one bit)

    xr = 0;                     // clear result
    q2 = 0x40000000L;           // higest possible result bit
    do
    {
        if ((xr + q2) <= x)
        {
            x -= xr + q2;
            f = 1;                  // set flag
        }
        else {
            f = 0;                  // clear flag
        }
        xr >>= 1;
        if (f) {
            xr += q2;               // test flag
        }
    } while (q2 >>= 2);          // shift twice
    if (xr < x) {
        return xr + 1;             // add for rounding
    }
    else {
        return xr;
    }
}

void IQ_Ramp_Init(IQ_Ramp_t *ramp, int32_t limit_p, int32_t limit_n, int32_t rise_ms, int32_t fall_ms, int32_t ts_ms)
{
    ramp->limit_p = limit_p;
    ramp->limit_n = limit_n;
    ramp->rise_ms = rise_ms;
    ramp->fall_ms = fall_ms;
    ramp->ts_ms = ts_ms;
    if(ramp->rise_ms)
    {
        ramp->delta_limit_p = (ramp->limit_p - ramp->limit_n) * ramp->ts_ms / ramp->rise_ms;
    }
    else
    {
        ramp->delta_limit_p = ramp->limit_p - ramp->limit_n;
    }
    if(ramp->fall_ms)
    {
        ramp->delta_limit_n = (ramp->limit_n - ramp->limit_p) * ramp->ts_ms / ramp->fall_ms;
    }
    else
    {
        ramp->delta_limit_n = ramp->limit_n - ramp->limit_p;
    }
}

void IQ_Ramp_Update(IQ_Ramp_t *ramp, int32_t in, int32_t *out)
{
    ramp->out_temp = *out;
    ramp->delta = in - ramp->out_temp;

    ramp->delta = IQ_constrain(ramp->delta,ramp->delta_limit_n,ramp->delta_limit_p);
    ramp->out_temp += ramp->delta;
    ramp->out_temp = IQ_constrain(ramp->out_temp,ramp->limit_n,ramp->limit_p);

    *out = ramp->out_temp;
}
