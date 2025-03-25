#include "PLL.h"
#include "My_Math.h"

// https://zhuanlan.zhihu.com/p/673354158

void PLL_Init(PLL_t *pPLL,float Kp,float Ki,float fc,float dt,IIR_Filter_t *p_iir)
{
    memset(pPLL,0,sizeof(PLL_t));
    pPLL->Kp = Kp;
    pPLL->Ki = Ki;
    pPLL->fc = fc;
    pPLL->dt = dt;

    float t = 1.0f / (_2PI * fc);
    pPLL->alpha = CONSTRAIN((dt / (dt + t)),0.0f,1.0f);

    pPLL->p_iir = p_iir;
}

void PLL_Run(PLL_t *pPLL,float sin_value,float cos_value)
{
    pPLL->err = sin_value * cosf(pPLL->theta) - cos_value * sinf(pPLL->theta);
    pPLL->integral += pPLL->err * pPLL->Ki * pPLL->dt;
    pPLL->wc = pPLL->err * pPLL->Kp + pPLL->integral;
    pPLL->hz = pPLL->wc / (pPLL->dt * _2PI);
    pPLL->theta_unlimit += pPLL->wc;
    pPLL->theta += pPLL->wc;
    pPLL->theta = Normalize_Rad(pPLL->theta);

    if(pPLL->p_iir != NULL)
    {
        pPLL->hz_f = IIR_Filter_Update(pPLL->p_iir,pPLL->hz);
    }
    else 
    {
        pPLL->hz_f += (pPLL->hz - pPLL->hz_f) * pPLL->alpha;
    }
}

void Normalize_PLL_Run(PLL_t *pPLL,float sin_value,float cos_value)
{
    float mag = NORM2_f(sin_value, cos_value);
    if (mag < 1e-10f)
    {
        mag = 1e-10f;
    }
    sin_value = sin_value / mag;
    cos_value = cos_value / mag;

    pPLL->err = sin_value * cosf(pPLL->theta) - cos_value * sinf(pPLL->theta);
    pPLL->integral += pPLL->err * pPLL->Ki * pPLL->dt;
    pPLL->wc = pPLL->err * pPLL->Kp + pPLL->integral;
    pPLL->hz = pPLL->wc / (pPLL->dt * _2PI);
    pPLL->theta_unlimit += pPLL->wc;
    pPLL->theta += pPLL->wc;
    pPLL->theta = Normalize_Rad(pPLL->theta);

    if(pPLL->p_iir != NULL)
    {
        pPLL->hz_f = IIR_Filter_Update(pPLL->p_iir,pPLL->hz);
    }
    else 
    {
        pPLL->hz_f += (pPLL->hz - pPLL->hz_f) * pPLL->alpha;
    }
}
