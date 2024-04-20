#include "PLL.h"
#include "My_Math.h"

// https://zhuanlan.zhihu.com/p/673354158

void PLL_Init(PLL_t *pPLL,float Kp,float Ki,float fc,float dt)
{
    memset(pPLL,0,sizeof(PLL_t));
    pPLL->Kp = Kp;
    pPLL->Ki = Ki;
    pPLL->fc = fc;
    pPLL->dt = dt;
    float t = 1.0f / (_2PI * fc);
    pPLL->alpha = _constrain((dt / (dt + t)),0.0f,1.0f);
}

void PLL_Run(PLL_t *pPLL,float sin_value,float cos_value)
{
    pPLL->err = sin_value * cosf(pPLL->theta) - cos_value * sinf(pPLL->theta);
    pPLL->integral += pPLL->err * pPLL->Ki * pPLL->dt;
    pPLL->out = pPLL->err * pPLL->Kp + pPLL->integral;
    pPLL->hz = pPLL->out / (pPLL->dt * _2PI);
    pPLL->hz_f += (pPLL->hz - pPLL->hz_f) * pPLL->alpha;
    pPLL->theta_unlimit += pPLL->out;
    pPLL->theta += pPLL->out;
    pPLL->theta = Normalize_Angle(pPLL->theta);
}

void Normalize_PLL_Run(PLL_t *pPLL,float sin_value,float cos_value)
{
    float mag = sqrtf(SQ(sin_value) + SQ(cos_value));
    if (mag < 1e-10f)
    {
        mag = 1e-10f;
    }
    sin_value = sin_value / mag;
    cos_value = cos_value / mag;

    pPLL->err = sin_value * cosf(pPLL->theta) - cos_value * sinf(pPLL->theta);
    pPLL->integral += pPLL->err * pPLL->Ki * pPLL->dt;
    pPLL->out = pPLL->err * pPLL->Kp + pPLL->integral;
    pPLL->hz = pPLL->out / (pPLL->dt * _2PI);
    pPLL->hz_f += (pPLL->hz - pPLL->hz_f) * pPLL->alpha;
    pPLL->theta_unlimit += pPLL->out;
    pPLL->theta += pPLL->out;
    pPLL->theta = Normalize_Angle(pPLL->theta);
}
