#ifndef __UPPERRUN_H__
#define __UPPERRUN_H__
#include "stdint.h"
void uppergoingtask(void const * argument);
void generate_mapping_array(int16_t arr1[], int16_t arr2[], int16_t output[]);
void process_group_special(int16_t mapX, int16_t mapY, int group_id);
#endif // !__UPPERRUN_H__