#include "Sort.h"

void Quick_Sort(int32_t *number, uint32_t first, uint32_t last)
{
    uint32_t i, j, pivot;
    int32_t temp;

    if (first < last)
    {
        pivot = first;
        i = first;
        j = last;
        while (i < j)
        {
            while (number[i] <= number[pivot] && i < last)
                i++;
            while (number[j] > number[pivot] && j > first)
                j--;
            if (i < j)
            {
                temp = number[i];
                number[i] = number[j];
                number[j] = temp;
            }
        }
        temp = number[pivot];
        number[pivot] = number[j];
        number[j] = temp;
        if (j >= 1)
        {
            Quick_Sort(number, first, j - 1);
        }
        Quick_Sort(number, j + 1, last);
    }
}

void Bubble_Sort_U8(uint8_t *arr,uint16_t len)
{
    uint8_t temp;

    for(uint8_t i = 0;i < len - 1;i++)
    {
        for(uint8_t j = 0;j < len - 1- i;j++) 
        {
            if(arr[j] > arr[j + 1]) 
            {
                temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
}

void Bubble_Sort_U16(uint16_t *arr,uint16_t len)
{
    uint16_t temp;

    for(uint16_t i = 0;i < len - 1;i++)
    {
        for(uint16_t j = 0;j < len - 1- i;j++) 
        {
            if(arr[j] > arr[j + 1]) 
            {
                temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
}

void Bubble_Sort_I32(int32_t *arr,uint16_t len)
{
    uint16_t temp;

    for(uint16_t i = 0;i < len - 1;i++)
    {
        for(uint16_t j = 0;j < len - 1- i;j++) 
        {
            if(arr[j] > arr[j + 1]) 
            {
                temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
}

void Bubble_Sort_F32(float *arr,uint16_t len)
{
    float temp;

    for(uint16_t i = 0;i < len - 1;i++)
    {
        for(uint16_t j = 0;j < len - 1- i;j++) 
        {
            if(arr[j] > arr[j + 1]) 
            {
                temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
}

void Index_Sort(float *arr,uint16_t len,uint16_t *index_arr)
{
    float temp,temp_index;

    for(uint16_t i = 0;i < len - 1;i++)
    {
        for(uint16_t j = 0;j < len - 1- i;j++) 
        {
            if(arr[j] > arr[j + 1]) 
            {
                temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;

                temp_index = index_arr[j];
                index_arr[j] = index_arr[j + 1];
                index_arr[j + 1] = temp_index;
            }
        }
    }
}

int32_t Find_Most_Repeated_Element(int32_t arr[], uint32_t len,int32_t *res)
{
    uint32_t slow_p = 0, fast_p = 1;
    uint32_t max_cnt = 0, temp_cnt = 0;

    // Bubble_Sort_I32(arr,len);
    Quick_Sort(arr,0,len - 1);

    while (fast_p < len)
    {
        if (arr[slow_p] == arr[fast_p])
        {
            if (++fast_p == len)
            {
                temp_cnt = fast_p - slow_p;
                if (temp_cnt > max_cnt)
                {
                    max_cnt = temp_cnt;
                    *res = arr[slow_p];
                }
            }
        }
        else
        {
            temp_cnt = fast_p - slow_p;
            if (temp_cnt > max_cnt)
            {
                max_cnt = temp_cnt;
                *res = arr[slow_p];
            }
            slow_p = fast_p++;
        }
    }

    return max_cnt;
}
