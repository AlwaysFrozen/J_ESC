#ifndef __SORT_H__
#define __SORT_H__

#include "stdbool.h"
#include "stdint.h"
#include "Utils.h"

void Quick_Sort(int32_t *number,uint32_t first,uint32_t last);
void Bubble_Sort_U8(uint8_t *arr,uint16_t len);
void Bubble_Sort_U16(uint16_t *arr,uint16_t len);
void Bubble_Sort_I32(int32_t *arr,uint16_t len);
void Bubble_Sort_F32(float *arr,uint16_t len);
void Index_Sort(float *arr,uint16_t len,uint16_t *index_arr);
int32_t Find_Most_Repeated_Element(int32_t arr[], uint32_t len,int32_t *res);

#endif
