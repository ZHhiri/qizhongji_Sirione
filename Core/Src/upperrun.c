#include "stdint.h"
#include "upperrun.h"
#include "param.h"
#include "math.h"
#include "upperservo.h"
#include "cmsis_os.h"
#include "tim.h"
#include "mi_motor.h"
uint16_t stateflag = 0;
uint16_t runflag   = 100;
int16_t xinagzi[6] = {2, 4, 5, 6, 3, 1};
int16_t zhiduo[6]  = {1, 6, 3, 4, 5, 0};
int16_t mapping[6] = {0};

int16_t flag = 0;
int16_t group=0;
float x_box;
float x_stack;
float y_stack;
float x2_stack;
float y2_stack;
float yaw_rotate;
float yaw2_rotate;
float z2_place;

float x_box_1   = -8.6;
float x_box_2   = 0 - 0.2;
float x_box_3   = 8.6 + 0.3;
float x_stack_1 = -7.3;
float x_stack_6 = 8.0;
float x_stack_2 = -11.7;
float x_stack_3 = -4.3;
float x_stack_4 = 3.57;
float x_stack_5 = 11.0;

float y_box        = (-9.08 - 1.03)/1.307-0.4; //-9.00000000005
float y_stack_16   = (5.35 - 2.43)/1.307+0.17;
float y_stack_2345 = (6.89 - 2.98)/1.307+0.17;

float z_highest    = -3.8 + 0.3;
float z_high       = -3.8 + 0.3;
float z_high_crawl = -3 + 0.3;
float z_low        = -0.75; //-1.05
float z_low_crawl  = -0.068;
float z_stack      = -2 + 0.3;
float z_stack_2    = -3.4 + 0.3; // 细调
float z_place_2    = -2.0 + 0.3; // 细调
float z_place      = -0.7 + 0.3;

float x_start = 0;
float y_start = 0;
float z_start = 0;
float y_stay  = -7/1.03;

float yaw_0     = 0;
float yaw_now   = -3.1415926535 / 15;
float yaw2_now  = 3.1415926535 - 3.1415926535 / 15;
float yaw_270   = 3.1415926535 / 2;
float yaw_180   = 3.1415926535;
float yaw_180_2 = -3.1415926535;
float yaw_90    = -3.1415926535 / 2;

void generate_mapping_array(int16_t arr1[], int16_t arr2[], int16_t output[])
{
    // 存储每个数字在两个数组中的索引
    int index_list1[6][6] = {{0}};
    int index_list2[6][6] = {{0}};

    // 存储每个数字在两个数组中出现的次数
    int count1[6] = {0};
    int count2[6] = {0};

    // 初始化输出数组为0（表示未匹配）
    for (int i = 0; i < 6; i++) {
        output[i] = 0;
    }

    // 遍历第一个数组（只处理1-6的数字）
    for (int i = 0; i < 6; i++) {
        int num = arr1[i];
        if (num >= 1 && num <= 6) {
            int idx                         = num - 1;
            index_list1[idx][count1[idx]++] = i;
        }
    }

    // 遍历第二个数组（忽略0值，只处理1-6的数字）
    for (int i = 0; i < 6; i++) {
        int num = arr2[i];
        if (num >= 1 && num <= 6) {
            int idx                         = num - 1;
            index_list2[idx][count2[idx]++] = i;
        }
    }

    // 为每个数字生成配对并填充输出数组
    for (int num = 0; num < 6; num++) {
        // 取两个数组中该数字出现次数的最小值
        int min_count = count1[num] < count2[num] ? count1[num] : count2[num];

        // 为每个配对更新输出数组
        for (int j = 0; j < min_count; j++) {
            int arr1_index     = index_list1[num][j];
            int arr2_index     = index_list2[num][j];
            output[arr1_index] = arr2_index + 1; // 索引值加1
        }
    }
}

void process_group_special(int16_t mapX, int16_t mapY, int group_id)
{
    printf("\n===== 处理第%d组 [索引%d和%d] =====\n",
           group_id, (group_id - 1) * 2, (group_id - 1) * 2 + 1);
    printf("映射值: %d, %d\n", mapX, mapY);

    // 特殊组合判断

    if (mapX == 0 && mapY == 1) {
        printf("[0,1] ");
        switch (group_id) {
            case 1:
                x_box       = x_box_1;
                x_stack     = x_stack_1;
                x2_stack    = x_stack_1;
                y_stack     = y_stack_16;
                y2_stack    = y_stack_16;
                yaw_rotate  = yaw_90;
                yaw2_rotate = yaw_270;
                z2_place    = z_place_2;
                printf(" (第一组处理)");
                break;
            case 2:
                x_box       = x_box_2;
                x_stack     = x_stack_1;
                x2_stack    = x_stack_1;
                y_stack     = y_stack_16;
                y2_stack    = y_stack_16;
                yaw_rotate  = yaw_90;
                yaw2_rotate = yaw_270;
                z2_place    = z_place_2;
                printf(" (第二组处理)");
                break;
            case 3:
                x_box       = x_box_3;
                x_stack     = x_stack_1;
                x2_stack    = x_stack_1;
                y_stack     = y_stack_16;
                y2_stack    = y_stack_16;
                yaw_rotate  = yaw_90;
                yaw2_rotate = yaw_270;
                z2_place    = z_place_2;
                printf(" (第三组处理)");
        }
    } else if (mapX == 0 && mapY == 2) {
        printf("[0,2]");
        switch (group_id) {
            case 1:
                x_box       = x_box_1;
                x_stack     = x_stack_2;
                x2_stack    = x_stack_2;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_0;
                yaw2_rotate = yaw_180;
                z2_place    = z_place_2;
                printf(" (第一组处理)");
                break;
            case 2:
                x_box       = x_box_2;
                x_stack     = x_stack_2;
                x2_stack    = x_stack_2;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_0;
                yaw2_rotate = yaw_180;
                z2_place    = z_place_2;
                printf(" (第二组处理)");
                break;
            case 3:
                x_box       = x_box_3;
                x_stack     = x_stack_2;
                x2_stack    = x_stack_2;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_0;
                yaw2_rotate = yaw_180;
                z2_place    = z_place_2;
                printf(" (第三组处理)");
        }
    } else if (mapX == 0 && mapY == 3) {
        printf("[0,3]");
        switch (group_id) {
            case 1:
                x_box       = x_box_1;
                x_stack     = x_stack_3;
                x2_stack    = x_stack_3;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_0;
                yaw2_rotate = yaw_180;
                z2_place    = z_place_2;
                printf(" (第一组处理)");
                break;
            case 2:
                x_box       = x_box_2;
                x_stack     = x_stack_3;
                x2_stack    = x_stack_3;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_0;
                yaw2_rotate = yaw_180;
                z2_place    = z_place_2;
                printf(" (第二组处理)");
                break;
            case 3:
                x_box       = x_box_3;
                x_stack     = x_stack_3;
                x2_stack    = x_stack_3;
                y_stack     = y_stack_16;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_0;
                yaw2_rotate = yaw_180;
                z2_place    = z_place_2;
                printf(" (第三组处理)");
        }
    } else if (mapX == 0 && mapY == 4) {
        printf("[0,2]");
        switch (group_id) {
            case 1:
                x_box       = x_box_1;
                x_stack     = x_stack_4;
                x2_stack    = x_stack_4;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_0;
                yaw2_rotate = yaw_180;
                z2_place    = z_place_2;
                printf(" (第一组处理)");
                break;
            case 2:
                x_box       = x_box_2;
                x_stack     = x_stack_4;
                x2_stack    = x_stack_4;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_0;
                yaw2_rotate = yaw_180;
                z2_place    = z_place_2;
                printf(" (第二组处理)");
                break;
            case 3:
                x_box       = x_box_3;
                x_stack     = x_stack_4;
                x2_stack    = x_stack_4;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_0;
                yaw2_rotate = yaw_180;
                z2_place    = z_place_2;
                printf(" (第三组处理)");
        }
    } else if (mapX == 0 && mapY == 5) {
        printf("[0,5]");
        switch (group_id) {
            case 1:
                x_box       = x_box_1;
                x_stack     = x_stack_5;
                x2_stack    = x_stack_5;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_0;
                yaw2_rotate = yaw_180;
                z2_place    = z_place_2;
                printf(" (第一组处理)");
                break;
            case 2:
                x_box       = x_box_2;
                x_stack     = x_stack_5;
                x2_stack    = x_stack_5;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_0;
                yaw2_rotate = yaw_180;
                z2_place    = z_place_2;
                printf(" (第二组处理)");
                break;
            case 3:
                x_box       = x_box_3;
                x_stack     = x_stack_5;
                x2_stack    = x_stack_5;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_0;
                yaw2_rotate = yaw_180;
                z2_place    = z_place_2;
                printf(" (第三组处理)");
        }
    } else if (mapX == 0 && mapY == 6) {

        switch (group_id) {
            case 1:
                x_box       = x_box_1;
                x_stack     = x_stack_6;
                x2_stack    = x_stack_6;
                y_stack     = y_stack_16;
                y2_stack    = y_stack_16;
                yaw_rotate  = yaw_270;
                yaw2_rotate = yaw_90;
                z2_place    = z_place_2;
                printf(" (第一组处理)");
                break;
            case 2:
                x_box       = x_box_2;
                x_stack     = x_stack_6;
                x2_stack    = x_stack_6;
                y_stack     = y_stack_16;
                y2_stack    = y_stack_16;
                yaw_rotate  = yaw_270;
                yaw2_rotate = yaw_90;
                z2_place    = z_place_2;
                printf(" (第二组处理)");
                break;
            case 3:
                x_box       = x_box_3;
                x_stack     = x_stack_6;
                x2_stack    = x_stack_6;
                y_stack     = y_stack_16;
                y2_stack    = y_stack_16;
                yaw_rotate  = yaw_270;
                yaw2_rotate = yaw_90;
                z2_place    = z_place_2;
                printf(" (第三组处理)");
        }
    } else if (mapX == 1 && mapY == 0) {
        printf("[1,0] ");
        switch (group_id) {
            case 1:
                x_box       = x_box_1;
                x_stack     = x_stack_1;
                x2_stack    = x_stack_1;
                y_stack     = y_stack_16;
                y2_stack    = y_stack_16;
                yaw_rotate  = yaw_270;
                yaw2_rotate = yaw_90;
                z2_place    = z_place_2;
                printf(" (第一组处理)");
                break;
            case 2:
                x_box       = x_box_2;
                x_stack     = x_stack_1;
                x2_stack    = x_stack_1;
                y_stack     = y_stack_16;
                y2_stack    = y_stack_16;
                yaw_rotate  = yaw_270;
                yaw2_rotate = yaw_90;
                z2_place    = z_place_2;
                printf(" (第二组处理)");
                break;
            case 3:
                x_box       = x_box_3;
                x_stack     = x_stack_1;
                x2_stack    = x_stack_1;
                y_stack     = y_stack_16;
                y2_stack    = y_stack_16;
                yaw_rotate  = yaw_270;
                yaw2_rotate = yaw_90;
                z2_place    = z_place_2;
                printf(" (第三组处理)");
        }
    } else if (mapX == 1 && mapY == 2) {
        printf("[1,2] ");
        switch (group_id) {
            case 1:
                x_box       = x_box_1;
                x_stack     = x_stack_1;
                x2_stack    = x_stack_2;
                y_stack     = y_stack_16;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_270;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第一组处理)");
                break;
            case 2:
                x_box       = x_box_2;
                x_stack     = x_stack_1;
                x2_stack    = x_stack_2;
                y_stack     = y_stack_16;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_270;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第二组处理)");
                break;
            case 3:
                x_box       = x_box_3;
                x_stack     = x_stack_1;
                x2_stack    = x_stack_2;
                y_stack     = y_stack_16;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_270;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第三组处理)");
                break;
        }
    } else if (mapX == 1 && mapY == 3) {
        printf("[1,3] ");
        switch (group_id) {
            case 1:
                x_box       = x_box_1;
                x_stack     = x_stack_1;
                x2_stack    = x_stack_3;
                y_stack     = y_stack_16;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_270;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第一组处理)");
                break;
            case 2:
                x_box       = x_box_2;
                x_stack     = x_stack_1;
                x2_stack    = x_stack_3;
                y_stack     = y_stack_16;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_270;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第二组处理)");
                break;
            case 3:
                x_box       = x_box_3;
                x_stack     = x_stack_1;
                x2_stack    = x_stack_3;
                y_stack     = y_stack_16;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_270;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第三组处理)");
                break;
        }
    } else if (mapX == 1 && mapY == 4) {
        printf("[1,4] ");
        switch (group_id) {
            case 1:
                x_box       = x_box_1;
                x_stack     = x_stack_1;
                x2_stack    = x_stack_4;
                y_stack     = y_stack_16;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_270;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第一组处理)");
                break;
            case 2:
                x_box       = x_box_2;
                x_stack     = x_stack_1;
                x2_stack    = x_stack_4;
                y_stack     = y_stack_16;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_270;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第二组处理)");
                break;
            case 3:
                x_box       = x_box_3;
                x_stack     = x_stack_1;
                x2_stack    = x_stack_4;
                y_stack     = y_stack_16;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_270;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第三组处理)");
                break;
        }
    } else if (mapX == 1 && mapY == 5) {
        printf("[1,5]");
        switch (group_id) {
            case 1:
                x_box       = x_box_1;
                x_stack     = x_stack_1;
                x2_stack    = x_stack_5;
                y_stack     = y_stack_16;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_270;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第一组处理)");
                break;
            case 2:
                x_box       = x_box_2;
                x_stack     = x_stack_1;
                x2_stack    = x_stack_5;
                y_stack     = y_stack_16;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_270;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第二组处理)");
                break;
            case 3:
                x_box       = x_box_3;
                x_stack     = x_stack_1;
                x2_stack    = x_stack_5;
                y_stack     = y_stack_16;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_270;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第三组处理)");
                break;
        }
    } else if (mapX == 1 && mapY == 6) {
        printf("[1,6]");
        switch (group_id) {
            case 1:
                x_box       = x_box_1;
                x_stack     = x_stack_1;
                x2_stack    = x_stack_6;
                y_stack     = y_stack_16;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_270;
                yaw2_rotate = yaw_270;
                z2_place    = z_place;
                printf(" (第一组处理)");
                break;
            case 2:
                x_box       = x_box_2;
                x_stack     = x_stack_1;
                x2_stack    = x_stack_6;
                y_stack     = y_stack_16;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_270;
                yaw2_rotate = yaw_270;
                z2_place    = z_place;
                printf(" (第二组处理)");
                break;
            case 3:
                x_box       = x_box_3;
                x_stack     = x_stack_1;
                x2_stack    = x_stack_6;
                y_stack     = y_stack_16;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_270;
                yaw2_rotate = yaw_270;
                z2_place    = z_place;
                printf(" (第三组处理)");
                break;
        }
    } else if (mapX == 2 && mapY == 0) {
        printf("[2,0]");
        switch (group_id) {
            case 1:
                x_box       = x_box_1;
                x_stack     = x_stack_2;
                x2_stack    = x_stack_2;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place_2;
                printf(" (第一组处理)");
                break;
            case 2:
                x_box       = x_box_2;
                x_stack     = x_stack_2;
                x2_stack    = x_stack_2;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place_2;
                printf(" (第二组处理)");
                break;
            case 3:
                x_box       = x_box_3;
                x_stack     = x_stack_2;
                x2_stack    = x_stack_2;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place_2;
                printf(" (第三组处理)");
        }
    } else if (mapX == 2 && mapY == 1) {
        printf("[2,1] ");
        switch (group_id) {
            case 1:
                x_box       = x_box_1;
                x_stack     = x_stack_2;
                x2_stack    = x_stack_1;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_16;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_90;
                z2_place    = z_place;
                printf(" (第一组处理)");
                break;
            case 2:
                x_box       = x_box_2;
                x_stack     = x_stack_2;
                x2_stack    = x_stack_1;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_16;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_90;
                z2_place    = z_place;
                printf(" (第二组处理)");
                break;
            case 3:
                x_box       = x_box_3;
                x_stack     = x_stack_2;
                x2_stack    = x_stack_1;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_16;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_90;
                z2_place    = z_place;
                printf(" (第三组处理)");
                break;
        }
    } else if (mapX == 2 && mapY == 3) {
        printf("[2,3]");
        switch (group_id) {
            case 1:
                x_box       = x_box_1;
                x_stack     = x_stack_2;
                x2_stack    = x_stack_3;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第一组处理)");
                break;
            case 2:
                x_box       = x_box_2;
                x_stack     = x_stack_2;
                x2_stack    = x_stack_3;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第二组处理)");
                break;
            case 3:
                x_box       = x_box_3;
                x_stack     = x_stack_2;
                x2_stack    = x_stack_3;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第三组处理)");
                break;
        }
    } else if (mapX == 2 && mapY == 4) {
        printf("[2,4]");
        switch (group_id) {
            case 1:
                x_box       = x_box_1;
                x_stack     = x_stack_2;
                x2_stack    = x_stack_4;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第一组处理)");
                break;
            case 2:
                x_box       = x_box_2;
                x_stack     = x_stack_2;
                x2_stack    = x_stack_4;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第二组处理)");
                break;
            case 3:
                x_box       = x_box_3;
                x_stack     = x_stack_2;
                x2_stack    = x_stack_4;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第三组处理)");
                break;
        }
    } else if (mapX == 2 && mapY == 5) {
        printf("[2,5]");
        switch (group_id) {
            case 1:
                x_box       = x_box_1;
                x_stack     = x_stack_2;
                x2_stack    = x_stack_5;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第一组处理)");
                break;
            case 2:
                x_box       = x_box_2;
                x_stack     = x_stack_2;
                x2_stack    = x_stack_5;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第二组处理)");
                break;
            case 3:
                x_box       = x_box_3;
                x_stack     = x_stack_2;
                x2_stack    = x_stack_5;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第三组处理)");
                break;
        }
    } else if (mapX == 2 && mapY == 6) {
        printf("[2,6]");
        switch (group_id) {
            case 1:
                x_box       = x_box_1;
                x_stack     = x_stack_2;
                x2_stack    = x_stack_6;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_16;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_270;
                z2_place    = z_place;
                printf(" (第一组处理)");
                break;
            case 2:
                x_box       = x_box_2;
                x_stack     = x_stack_2;
                x2_stack    = x_stack_6;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_16;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_270;
                z2_place    = z_place;
                printf(" (第二组处理)");
                break;
            case 3:
                x_box       = x_box_3;
                x_stack     = x_stack_2;
                x2_stack    = x_stack_6;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_16;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_270;
                z2_place    = z_place;
                printf(" (第三组处理)");
                break;
        }
    } else if (mapX == 3 && mapY == 0) {
        printf("[3,0]");
        switch (group_id) {
            case 1:
                x_box       = x_box_1;
                x_stack     = x_stack_3;
                x2_stack    = x_stack_3;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place_2;
                printf(" (第一组处理)");
                break;
            case 2:
                x_box       = x_box_2;
                x_stack     = x_stack_3;
                x2_stack    = x_stack_3;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place_2;
                printf(" (第二组处理)");
                break;
            case 3:
                x_box       = x_box_3;
                x_stack     = x_stack_3;
                x2_stack    = x_stack_3;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place_2;
                printf(" (第三组处理)");
        }
    } else if (mapX == 3 && mapY == 1) {
        printf("[3,1]");
        switch (group_id) {
            case 1:
                x_box       = x_box_1;
                x_stack     = x_stack_3;
                x2_stack    = x_stack_1;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_16;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_90;
                z2_place    = z_place;
                printf(" (第一组处理)");
                break;
            case 2:
                x_box       = x_box_2;
                x_stack     = x_stack_3;
                x2_stack    = x_stack_1;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_16;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_90;
                z2_place    = z_place;
                printf(" (第二组处理)");
                break;
            case 3:
                x_box       = x_box_3;
                x_stack     = x_stack_3;
                x2_stack    = x_stack_1;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_16;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_90;
                z2_place    = z_place;
                printf(" (第三组处理)");
                break;
        }
    } else if (mapX == 3 && mapY == 2) {
        printf("[3,2]");
        switch (group_id) {
            case 1:
                x_box       = x_box_1;
                x_stack     = x_stack_3;
                x2_stack    = x_stack_2;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第一组处理)");
                break;
            case 2:
                x_box       = x_box_2;
                x_stack     = x_stack_3;
                x2_stack    = x_stack_2;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第二组处理)");
                break;
            case 3:
                x_box       = x_box_3;
                x_stack     = x_stack_3;
                x2_stack    = x_stack_2;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第三组处理)");
                break;
        }
    }

    else if (mapX == 3 && mapY == 4) {
        printf("[3,4]");
        switch (group_id) {
            case 1:
                x_box       = x_box_1;
                x_stack     = x_stack_3;
                x2_stack    = x_stack_4;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第一组处理)");
                break;
            case 2:
                x_box       = x_box_2;
                x_stack     = x_stack_3;
                x2_stack    = x_stack_4;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第二组处理)");
                break;
            case 3:
                x_box       = x_box_3;
                x_stack     = x_stack_3;
                x2_stack    = x_stack_4;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第三组处理)");
                break;
        }
    } else if (mapX == 3 && mapY == 5) {
        printf("[3,5]");
        switch (group_id) {
            case 1:
                x_box       = x_box_1;
                x_stack     = x_stack_3;
                x2_stack    = x_stack_5;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第一组处理)");
                break;
            case 2:
                x_box       = x_box_2;
                x_stack     = x_stack_3;
                x2_stack    = x_stack_5;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第二组处理)");
                break;
            case 3:
                x_box       = x_box_3;
                x_stack     = x_stack_3;
                x2_stack    = x_stack_5;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第三组处理)");
                break;
        }
    } else if (mapX == 3 && mapY == 6) {
        printf("[3,6]");
        switch (group_id) {
            case 1:
                x_box       = x_box_1;
                x_stack     = x_stack_3;
                x2_stack    = x_stack_6;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_16;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_270;
                z2_place    = z_place;
                printf(" (第一组处理)");
                break;
            case 2:
                x_box       = x_box_2;
                x_stack     = x_stack_3;
                x2_stack    = x_stack_6;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_16;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_270;
                z2_place    = z_place;
                printf(" (第二组处理)");
                break;
            case 3:
                x_box       = x_box_3;
                x_stack     = x_stack_3;
                x2_stack    = x_stack_6;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_16;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_270;
                z2_place    = z_place;
                printf(" (第三组处理)");
                break;
        }
    } else if (mapX == 4 && mapY == 0) {
        printf("[4,0]");
        switch (group_id) {
            case 1:
                x_box       = x_box_1;
                x_stack     = x_stack_4;
                x2_stack    = x_stack_4;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place_2;
                printf(" (第一组处理)");
                break;
            case 2:
                x_box       = x_box_2;
                x_stack     = x_stack_4;
                x2_stack    = x_stack_4;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place_2;
                printf(" (第二组处理)");
                break;
            case 3:
                x_box       = x_box_3;
                x_stack     = x_stack_4;
                x2_stack    = x_stack_4;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place_2;
                printf(" (第三组处理)");
        }
    } else if (mapX == 4 && mapY == 1) {
        printf("[4,1]");
        switch (group_id) {
            case 1:
                x_box       = x_box_1;
                x_stack     = x_stack_4;
                x2_stack    = x_stack_1;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_16;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_90;
                z2_place    = z_place;
                printf(" (第一组处理)");
                break;
            case 2:
                x_box       = x_box_2;
                x_stack     = x_stack_4;
                x2_stack    = x_stack_1;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_16;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_90;
                z2_place    = z_place;
                printf(" (第二组处理)");
                break;
            case 3:
                x_box       = x_box_3;
                x_stack     = x_stack_4;
                x2_stack    = x_stack_1;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_16;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_90;
                z2_place    = z_place;
                printf(" (第三组处理)");
                break;
        }
    } else if (mapX == 4 && mapY == 2) {
        printf("[4,2]");
        switch (group_id) {
            case 1:
                x_box       = x_box_1;
                x_stack     = x_stack_4;
                x2_stack    = x_stack_2;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第一组处理)");
                break;
            case 2:
                x_box       = x_box_2;
                x_stack     = x_stack_4;
                x2_stack    = x_stack_2;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第二组处理)");
                break;
            case 3:
                x_box       = x_box_3;
                x_stack     = x_stack_4;
                x2_stack    = x_stack_2;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第三组处理)");
                break;
        }
    } else if (mapX == 4 && mapY == 3) {
        printf("[4,3]");
        switch (group_id) {
            case 1:
                x_box       = x_box_1;
                x_stack     = x_stack_4;
                x2_stack    = x_stack_3;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第一组处理)");
                break;
            case 2:
                x_box       = x_box_2;
                x_stack     = x_stack_4;
                x2_stack    = x_stack_3;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第二组处理)");
                break;
            case 3:
                x_box       = x_box_3;
                x_stack     = x_stack_4;
                x2_stack    = x_stack_3;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第三组处理)");
                break;
        }
    } else if (mapX == 4 && mapY == 5) {
        printf("[4,5]");
        switch (group_id) {
            case 1:
                x_box       = x_box_1;
                x_stack     = x_stack_4;
                x2_stack    = x_stack_5;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第一组处理)");
                break;
            case 2:
                x_box       = x_box_2;
                x_stack     = x_stack_4;
                x2_stack    = x_stack_5;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第二组处理)");
                break;
            case 3:
                x_box       = x_box_3;
                x_stack     = x_stack_4;
                x2_stack    = x_stack_5;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第三组处理)");
                break;
        }
    } else if (mapX == 4 && mapY == 6) {
        printf("[4,6]");
        switch (group_id) {
            case 1:
                x_box       = x_box_1;
                x_stack     = x_stack_4;
                x2_stack    = x_stack_6;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_16;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_270;
                z2_place    = z_place;
                printf(" (第一组处理)");
                break;
            case 2:
                x_box       = x_box_2;
                x_stack     = x_stack_4;
                x2_stack    = x_stack_6;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_16;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_270;
                z2_place    = z_place;
                printf(" (第二组处理)");
                break;
            case 3:
                x_box       = x_box_3;
                x_stack     = x_stack_4;
                x2_stack    = x_stack_6;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_16;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_270;
                z2_place    = z_place;
                printf(" (第三组处理)");
                break;
        }
    } else if (mapX == 5 && mapY == 0) {
        printf("[5,0]");
        switch (group_id) {
            case 1:
                x_box       = x_box_1;
                x_stack     = x_stack_5;
                x2_stack    = x_stack_5;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_16;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place_2;
                printf(" (第一组处理)");
                break;
            case 2:
                x_box       = x_box_2;
                x_stack     = x_stack_5;
                x2_stack    = x_stack_5;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place_2;
                printf(" (第二组处理)");
                break;
            case 3:
                x_box       = x_box_3;
                x_stack     = x_stack_5;
                x2_stack    = x_stack_5;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place_2;
                printf(" (第三组处理)");
        }
    } else if (mapX == 5 && mapY == 1) {
        printf("[5,1]");
        switch (group_id) {
            case 1:
                x_box       = x_box_1;
                x_stack     = x_stack_5;
                x2_stack    = x_stack_1;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_16;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_90;
                z2_place    = z_place;
                printf(" (第一组处理)");
                break;
            case 2:
                x_box       = x_box_2;
                x_stack     = x_stack_5;
                x2_stack    = x_stack_1;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_16;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_90;
                z2_place    = z_place;
                printf(" (第二组处理)");
                break;
            case 3:
                x_box       = x_box_3;
                x_stack     = x_stack_5;
                x2_stack    = x_stack_1;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_16;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_90;
                z2_place    = z_place;
                printf(" (第三组处理)");
                break;
        }
    } else if (mapX == 5 && mapY == 2) {
        printf("[5,2]");
        switch (group_id) {
            case 1:
                x_box       = x_box_1;
                x_stack     = x_stack_5;
                x2_stack    = x_stack_2;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第一组处理)");
                break;
            case 2:
                x_box       = x_box_2;
                x_stack     = x_stack_5;
                x2_stack    = x_stack_2;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第二组处理)");
                break;
            case 3:
                x_box       = x_box_3;
                x_stack     = x_stack_5;
                x2_stack    = x_stack_2;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第三组处理)");
                break;
        }
    } else if (mapX == 5 && mapY == 3) {
        printf("[5,3]");
        switch (group_id) {
            case 1:
                x_box       = x_box_1;
                x_stack     = x_stack_5;
                x2_stack    = x_stack_3;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第一组处理)");
                break;
            case 2:
                x_box       = x_box_2;
                x_stack     = x_stack_5;
                x2_stack    = x_stack_3;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第二组处理)");
                break;
            case 3:
                x_box       = x_box_3;
                x_stack     = x_stack_5;
                x2_stack    = x_stack_3;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第三组处理)");
                break;
        }
    } else if (mapX == 5 && mapY == 4) {
        printf("[5,4]");
        switch (group_id) {
            case 1:
                x_box       = x_box_1;
                x_stack     = x_stack_5;
                x2_stack    = x_stack_4;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第一组处理)");
                break;
            case 2:
                x_box       = x_box_2;
                x_stack     = x_stack_5;
                x2_stack    = x_stack_4;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第二组处理)");
                break;
            case 3:
                x_box       = x_box_3;
                x_stack     = x_stack_5;
                x2_stack    = x_stack_4;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第三组处理)");
                break;
        }
    } else if (mapX == 5 && mapY == 6) {
        printf("[5,6]");
        switch (group_id) {
            case 1:
                x_box       = x_box_1;
                x_stack     = x_stack_5;
                x2_stack    = x_stack_6;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_16;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_270;
                z2_place    = z_place;
                printf(" (第一组处理)");
                break;
            case 2:
                x_box       = x_box_2;
                x_stack     = x_stack_5;
                x2_stack    = x_stack_6;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_16;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_270;
                z2_place    = z_place;
                printf(" (第二组处理)");
                break;
            case 3:
                x_box       = x_box_3;
                x_stack     = x_stack_5;
                x2_stack    = x_stack_6;
                y_stack     = y_stack_2345;
                y2_stack    = y_stack_16;
                yaw_rotate  = yaw_180;
                yaw2_rotate = yaw_270;
                z2_place    = z_place;
                printf(" (第三组处理)");
                break;
        }
    } else if (mapX == 6 && mapY == 0) {
        printf("[6,0]");
        switch (group_id) {
            case 1:
                x_box       = x_box_1;
                x_stack     = x_stack_6;
                x2_stack    = x_stack_6;
                y_stack     = y_stack_16;
                y2_stack    = y_stack_16;
                yaw_rotate  = yaw_90;
                yaw2_rotate = yaw_270;
                z2_place    = z_place_2;
                printf(" (第一组处理)");
                break;
            case 2:
                x_box       = x_box_2;
                x_stack     = x_stack_6;
                x2_stack    = x_stack_6;
                y_stack     = y_stack_16;
                y2_stack    = y_stack_16;
                yaw_rotate  = yaw_90;
                yaw2_rotate = yaw_270;
                z2_place    = z_place_2;
                printf(" (第二组处理)");
                break;
            case 3:
                x_box       = x_box_3;
                x_stack     = x_stack_6;
                x2_stack    = x_stack_6;
                y_stack     = y_stack_16;
                y2_stack    = y_stack_16;
                yaw_rotate  = yaw_90;
                yaw2_rotate = yaw_270;
                z2_place    = z_place_2;
                printf(" (第三组处理)");
        }
    } else if (mapX == 6 && mapY == 1) {
        printf("[6,1]");
        switch (group_id) {
            case 1:
                x_box       = x_box_1;
                x_stack     = x_stack_6;
                x2_stack    = x_stack_1;
                y_stack     = y_stack_16;
                y2_stack    = y_stack_16;
                yaw_rotate  = yaw_90;
                yaw2_rotate = yaw_90;
                z2_place    = z_place;
                printf(" (第一组处理)");
                break;
            case 2:
                x_box       = x_box_2;
                x_stack     = x_stack_6;
                x2_stack    = x_stack_1;
                y_stack     = y_stack_16;
                y2_stack    = y_stack_16;
                yaw_rotate  = yaw_90;
                yaw2_rotate = yaw_90;
                z2_place    = z_place;
                printf(" (第二组处理)");
                break;
            case 3:
                x_box       = x_box_3;
                x_stack     = x_stack_6;
                x2_stack    = x_stack_1;
                y_stack     = y_stack_16;
                y2_stack    = y_stack_16;
                yaw_rotate  = yaw_90;
                yaw2_rotate = yaw_90;
                z2_place    = z_place;
                printf(" (第三组处理)");
                break;
        }
    } else if (mapX == 6 && mapY == 2) {
        printf("[6,2]");
        switch (group_id) {
            case 1:
                x_box       = x_box_1;
                x_stack     = x_stack_6;
                x2_stack    = x_stack_2;
                y_stack     = y_stack_16;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_90;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第一组处理)");
                break;
            case 2:
                x_box       = x_box_2;
                x_stack     = x_stack_6;
                x2_stack    = x_stack_2;
                y_stack     = y_stack_16;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_90;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第二组处理)");
                break;
            case 3:
                x_box       = x_box_3;
                x_stack     = x_stack_6;
                x2_stack    = x_stack_2;
                y_stack     = y_stack_16;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_90;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第三组处理)");
                break;
        }
    } else if (mapX == 6 && mapY == 3) {
        printf("[6,3]");
        switch (group_id) {
            case 1:
                x_box       = x_box_1;
                x_stack     = x_stack_6;
                x2_stack    = x_stack_3;
                y_stack     = y_stack_16;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_90;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第一组处理)");
                break;
            case 2:
                x_box       = x_box_2;
                x_stack     = x_stack_6;
                x2_stack    = x_stack_3;
                y_stack     = y_stack_16;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_90;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第二组处理)");
                break;
            case 3:
                x_box       = x_box_3;
                x_stack     = x_stack_6;
                x2_stack    = x_stack_3;
                y_stack     = y_stack_16;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_90;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第三组处理)");
                break;
        }
    } else if (mapX == 6 && mapY == 4) {
        printf("[6,4]");
        switch (group_id) {
            case 1:
                x_box       = x_box_1;
                x_stack     = x_stack_6;
                x2_stack    = x_stack_4;
                y_stack     = y_stack_16;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_90;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第一组处理)");
                break;
            case 2:
                x_box       = x_box_2;
                x_stack     = x_stack_6;
                x2_stack    = x_stack_4;
                y_stack     = y_stack_16;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_90;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第二组处理)");
                break;
            case 3:
                x_box       = x_box_3;
                x_stack     = x_stack_6;
                x2_stack    = x_stack_4;
                y_stack     = y_stack_16;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_90;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第三组处理)");
                break;
        }
    } else if (mapX == 6 && mapY == 5) {
        printf("[6,5]");
        switch (group_id) {
            case 1:
                x_box       = x_box_1;
                x_stack     = x_stack_6;
                x2_stack    = x_stack_5;
                y_stack     = y_stack_16;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_90;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第一组处理)");
                break;
            case 2:
                x_box       = x_box_2;
                x_stack     = x_stack_6;
                x2_stack    = x_stack_5;
                y_stack     = y_stack_16;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_90;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第二组处理)");
                break;
            case 3:
                x_box       = x_box_3;
                x_stack     = x_stack_6;
                x2_stack    = x_stack_5;
                y_stack     = y_stack_16;
                y2_stack    = y_stack_2345;
                yaw_rotate  = yaw_90;
                yaw2_rotate = yaw_0;
                z2_place    = z_place;
                printf(" (第三组处理)");
                break;
        }
    } else {
        printf("err");
    }
    printf("\n");
}


void uppergoingtask(void const *argument)
{
    /* USER CODE BEGIN uppergoingtask */
    /* Infinite loop */

    /*
       货架
       y----->
       1 2 3
    x
    |
    |
    |
    |
    |
    |
    6         1
      5 4 3 2
       纸垛
    z轴是0为起始位置

    基本逻辑：
      1、每次取不同横坐标x1、x2、x3的货架的上下两个箱子（先取上面的、后退、旋转下降、取下面的）
      2、每次取两个箱子后回到起始区（回程中回转180度）
      3、从起始区向纸垛出发先放后取的箱子
      4、放箱子
    */

    generate_mapping_array(xinagzi, zhiduo, mapping);
   
    // for (uint16_t i = 0; i < 3000; i++) {
    //     motor_controlmode(&mi_motor[0], 0, yaw_now, 0, 1.5, 0.5);
    //     osDelay(1);
    // }
    // for (uint16_t i = 0; i < 3000; i++) {
    //     motor_controlmode(&mi_motor[0], 0, yaw_0, 0, 2, 0.8);
    //     osDelay(1);
    // }
    // for (uint16_t i = 0; i < 3000; i++) {
    //     motor_controlmode(&mi_motor[0], 0, yaw_180, 0, 2, 0.8);
    //     osDelay(1);
    // }
    // for (uint16_t i = 0; i < 3000; i++) {
    //     motor_controlmode(&mi_motor[0], 0, 0, 0, 2, 0.8);
    //     osDelay(1);
    // }
    // osDelay(3000);
    //  mygantry.gantrypos.z = z_high;
    // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1100); // Open
    // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1100); // Open
    //  osDelay(2000);

//   mygantry.gantrypos.y =y_stack_2345 ;

//     for ( ; ; )
//     {
//         /* code */
//     }
    
    // for (;;) {
    // mygantry.gantrypos.x = x_box_1;
    // float diff_x = fabs(mygantry.gantrypos.x * 8191 - hDJI[3].AxisData.AxisAngle_inDegree);
    // if (diff_x < 80) {
    //     runflag = 110;
    // }
    // if(runflag == 110)
    // {
    //     mygantry.gantrypos.x = 0;
    // }
    // osDelay(50);
    // }
    osDelay(100);

    for (;;) {

        // 先设定初始位置：（0,0,0）
        for (group = 0; group < 3;) {

            int idx1 = group * 2;
            int idx2 = group * 2 + 1;
            process_group_special(mapping[idx1], mapping[idx2], group + 1);
            if (runflag == 100) {

                float diff_z = fabs(mygantry.gantrypos.z * 8191 - hDJI[4].AxisData.AxisAngle_inDegree);
                float diff_y = fabs(mygantry.gantrypos.y * 8191 - hDJI[1].AxisData.AxisAngle_inDegree);
                float diff_x = fabs(mygantry.gantrypos.x * 8191 - hDJI[3].AxisData.AxisAngle_inDegree);

                // float diff_yaw = fabs(mygantry.gantrypos.yaw * 8191 - hDJI[0].AxisData.AxisAngle_inDegree);

                if (diff_x < 90 && diff_y < 90 && diff_z < 90) // diff<4°/360°*8191
                {
                    //osDelay(500);
                    runflag = 0;
                }
            }

            // 进入抓取环节
            if (stateflag == 0) {
                // 前往(x_box1,0,z_highest,0)
                if (runflag == 0) {
                    
                    mygantry.gantrypos.z = z_highest;
                    mygantry.gantrypos.x = x_box;
					
					  for (uint16_t i = 0; i < 500; i++) {
                        motor_controlmode(&mi_motor[0], 0, yaw_0, 0, 1.5, 0.6);
                        osDelay(1);
                    }
                    
                    //float diff_z         = fabs(mygantry.gantrypos.z * 8191 - hDJI[4].AxisData.AxisAngle_inDegree);
                    float diff_x         = fabs(mygantry.gantrypos.x * 8191 - hDJI[3].AxisData.AxisAngle_inDegree);
                    if ( diff_x < 400) {
                        runflag = 1;
                    }
                }

                if (runflag == 1) {
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1100); // Open
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1100); // Open

                    osDelay(500); // 等待Servo
                    runflag = 2;   // 进入下一个环节
                    WheelCorrect_StartTick = WheelCorrect_NowTick;
                    Wheel_StartPos[1]      = hDJI[1].posPID.fdb;
                    Wheel_StartPos[2]      = hDJI[2].posPID.fdb;
                }

                // 前往(x_box1,y_box,z_highest,0)
                if (runflag == 2) {

                    mygantry.gantrypos.y = y_box;
                    osDelay(200);                            // 等待Servo                   
                    mygantry.gantrypos.z = z_low;
                    float diff_z         = fabs(mygantry.gantrypos.z * 8191 - hDJI[4].AxisData.AxisAngle_inDegree);
                    float diff_y = fabs(mygantry.gantrypos.y * 8191 - hDJI[1].AxisData.AxisAngle_inDegree);
                    if (diff_z < 500 &&diff_y < 90) {
                        runflag = 3;
                    }
                }

                // 下降到抓取高度
                if (runflag == 3) {
                    mygantry.gantrypos.z = z_low_crawl;
                    float diff_z         = fabs(mygantry.gantrypos.z * 8191 - hDJI[4].AxisData.AxisAngle_inDegree);
                    if (diff_z < 500) {
                        runflag = 4;
                    }
                }

                if (runflag == 4) {
                     osDelay(100);                                      // 等待Servo
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 800); // Close
                    osDelay(500);
                    pid_reset(&mygantry.Motor_Z->posPID, 6.0, 0.3, 0.00001); // 重置z轴位置PID
                    runflag = 5;
                }

                // 进行抓取动作并进行下一个抓取动作的准备动作
                if (runflag == 5) {
                    //osDelay(500);
                    mygantry.gantrypos.z = z_low; // 抓取完毕后抬高
                    float diff_z         = fabs(mygantry.gantrypos.z * 8191 - hDJI[4].AxisData.AxisAngle_inDegree);
                    if (diff_z < 500) {
                        runflag = 6;
                        WheelCorrect_StartTick = WheelCorrect_NowTick;
                        Wheel_StartPos[1]      = hDJI[1].posPID.fdb;
                        Wheel_StartPos[2]      = hDJI[2].posPID.fdb;
                    }
                }

                if (runflag == 6) {
                    mygantry.gantrypos.y = y_stay;
                    // mygantry.gantrypos.yaw = yaw_180;
                    float diff_y = fabs(mygantry.gantrypos.y * 8191 - hDJI[1].AxisData.AxisAngle_inDegree);
                    if (diff_y < 90) {
                        runflag = 7;
                    }
                }

                if (runflag == 7) {

                    // mygantry.gantrypos.yaw = yaw_180;
                    mygantry.gantrypos.z = z_highest;
                    for (uint16_t i = 0; i < 2500; i++) {
                        motor_controlmode(&mi_motor[0], 0, yaw_180, 0, 1.5, 0.5);
                        osDelay(1);
                    }
                    
                    float diff_z = fabs(mygantry.gantrypos.z * 8191 - hDJI[4].AxisData.AxisAngle_inDegree);
                    if (diff_z < 500) {
                        runflag = 8;
                        WheelCorrect_StartTick = WheelCorrect_NowTick;
                        Wheel_StartPos[1]      = hDJI[1].posPID.fdb;
                        Wheel_StartPos[2]      = hDJI[2].posPID.fdb;
                    }
                }

                if (runflag == 8) {                    
                    mygantry.gantrypos.y = y_box;
                    // mygantry.gantrypos.x = x_box_1;
                    mygantry.gantrypos.x = x_box;
                    float diff_y         = fabs(mygantry.gantrypos.y * 8191 - hDJI[1].AxisData.AxisAngle_inDegree);
                    float diff_x         = fabs(mygantry.gantrypos.x * 8191 - hDJI[3].AxisData.AxisAngle_inDegree);

                    if (diff_x < 400 && diff_y < 90) {
                        runflag = 9;
                    }
                }

                if (runflag == 9) {
                    mygantry.gantrypos.z = z_high_crawl;
                    float diff_z         = fabs(mygantry.gantrypos.z * 8191 - hDJI[4].AxisData.AxisAngle_inDegree);
                    if (diff_z < 500) {
                        runflag = 10;
                    }
                }

                if (runflag == 10) {
                    
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 800); // Close
                    osDelay(500);
                    runflag = 11;
                }

                if (runflag == 11) {
                    
                    mygantry.gantrypos.z = z_high;
                    float diff_z         = fabs(mygantry.gantrypos.z * 8191 - hDJI[4].AxisData.AxisAngle_inDegree);
                    if (diff_z < 500) {
                        runflag = 12;
                         WheelCorrect_StartTick = WheelCorrect_NowTick;
                        Wheel_StartPos[1]      = hDJI[1].posPID.fdb;
                        Wheel_StartPos[2]      = hDJI[2].posPID.fdb;
                    }
                }
                //
                if (runflag == 12) {
                    mygantry.gantrypos.y = y_stack;
                    mygantry.gantrypos.z = z_high ;
                    osDelay(400);
                    mygantry.gantrypos.x = x_stack;
                    motor_controlmode(&mi_motor[0], 0, yaw_rotate, 0, 1.6, 0.83);
                    
                    // mygantry.gantrypos.yaw = yaw_rotate;

                    float diff_y = fabs(mygantry.gantrypos.y * 8191 - hDJI[1].AxisData.AxisAngle_inDegree);
                    float diff_x = fabs(mygantry.gantrypos.x * 8191 - hDJI[3].AxisData.AxisAngle_inDegree);
                    float diff_z = fabs(mygantry.gantrypos.z * 8191 - hDJI[4].AxisData.AxisAngle_inDegree);
                    // float diff_yaw = fabs(mygantry.gantrypos.yaw * 8191 - hDJI[0].AxisData.AxisAngle_inDegree);
                    if (diff_x < 400 && diff_y < 90 && diff_z < 500) {
                        runflag = 14;
                        WheelCorrect_StartTick = WheelCorrect_NowTick;
                        Wheel_StartPos[1]      = hDJI[1].posPID.fdb;
                        Wheel_StartPos[2]      = hDJI[2].posPID.fdb;
                    }
                }

                // if (runflag == 13) {
                //     for (uint16_t i = 0; i < 3000; i++) {
                //         motor_controlmode(&mi_motor[0], 0, yaw_rotate, 0, 2, 0.8);
                //         osDelay(1);
                //     }
                //     mygantry.gantrypos.y = y_stack;
                //     float diff_y         = fabs(mygantry.gantrypos.y * 8191 - hDJI[1].AxisData.AxisAngle_inDegree);
                //     if (diff_y < 90) {
                //         runflag = 14;
                //     }
                // }

                if (runflag == 14) {
                    mygantry.gantrypos.z = z_place;
                    float diff_z         = fabs(mygantry.gantrypos.z * 8191 - hDJI[4].AxisData.AxisAngle_inDegree);
                    if (diff_z < 500) { 
                        osDelay(300);
                        if (mapping[idx1]==0)
                        {
                            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1100); // Open
                            /* code */
                        }
                        else
                        {
                            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1100); // Open  
                        }
                        osDelay(500);
                        runflag = 15;
                        WheelCorrect_StartTick = WheelCorrect_NowTick;
                        Wheel_StartPos[1]      = hDJI[1].posPID.fdb;
                        Wheel_StartPos[2]      = hDJI[2].posPID.fdb;
                    }
                }

                if (runflag == 15) {
                    
                    
                    mygantry.gantrypos.z = z_high;
					mygantry.gantrypos.y = 0.0;
					 for (uint16_t i = 0; i < 1500; i++) {
                        motor_controlmode(&mi_motor[0], 0, yaw2_rotate, 0, 1.6, 0.83);
                        osDelay(1);
                    }                 
                    float diff_z = fabs(mygantry.gantrypos.z * 8191 - hDJI[4].AxisData.AxisAngle_inDegree);

                    // float diff_yaw         = fabs(mygantry.gantrypos.yaw * 8191 - hDJI[0].AxisData.AxisAngle_inDegree);
                    if ( diff_z < 500) {
                        runflag = 16;
                         WheelCorrect_StartTick = WheelCorrect_NowTick;
                        Wheel_StartPos[1]      = hDJI[1].posPID.fdb;
                        Wheel_StartPos[2]      = hDJI[2].posPID.fdb;
                    }
                }

                if (runflag == 16) {

                   
                    if (flag == 0) {
                        WheelCorrect_StartTick = WheelCorrect_NowTick;
                        Wheel_StartPos[1]      = hDJI[1].posPID.fdb;
                        Wheel_StartPos[2]      = hDJI[2].posPID.fdb;
                        flag                   = 1;
                        /* code */
                    }
                    mygantry.gantrypos.x = x2_stack;
                    mygantry.gantrypos.y = y2_stack;
                    float diff_x         = fabs(mygantry.gantrypos.x * 8191 - hDJI[3].AxisData.AxisAngle_inDegree);
                    float diff_y         = fabs(mygantry.gantrypos.y * 8191 - hDJI[1].AxisData.AxisAngle_inDegree);
                    
                    if (diff_x < 400 &&diff_y < 90) {
                        runflag = 17;
                    }
                }

                if (runflag == 17) {

                    mygantry.gantrypos.z = z2_place;
                    float diff_z         = fabs(mygantry.gantrypos.z * 8191 - hDJI[4].AxisData.AxisAngle_inDegree);

                    if (diff_z < 500) {
                        runflag = 18;
                          if (mapping[idx1]==0)
                        {
                            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1100); // Open
                            /* code */
                        }
                        else
                        {
                            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1100); // Open  
                        }
                    osDelay(500);
					WheelCorrect_StartTick = WheelCorrect_NowTick;
                        Wheel_StartPos[1]      = hDJI[1].posPID.fdb;
                        Wheel_StartPos[2]      = hDJI[2].posPID.fdb;
                    }
                }

                if (runflag == 18) {
                 
                    // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1100); // Open
                    mygantry.gantrypos.z = z_highest;
                    osDelay(500);
                    mygantry.gantrypos.y = 0.0;
                    float diff_z = fabs(mygantry.gantrypos.z * 8191 - hDJI[4].AxisData.AxisAngle_inDegree);
                    
                    if (diff_z < 500)
                    {
                        runflag = 19;
                        runflag = 100;
                        group++;
                        WheelCorrect_StartTick = WheelCorrect_NowTick;
                        Wheel_StartPos[1]      = hDJI[1].posPID.fdb;
                        Wheel_StartPos[2]      = hDJI[2].posPID.fdb;
                    }
                }

                if (group == 3) {
                    mygantry.gantrypos.y = y_box;
                    // mygantry.gantrypos.x = x_box_1;
                    mygantry.gantrypos.x = x_box;
                    /* code */
                    float diff_x = fabs(mygantry.gantrypos.x * 8191 - hDJI[3].AxisData.AxisAngle_inDegree);
                    float diff_y = fabs(mygantry.gantrypos.y * 8191 - hDJI[1].AxisData.AxisAngle_inDegree);

                    if (diff_x < 400 && diff_y < 90) {
                        runflag = 20;
                    }
                }
            }
        }
        osDelay(50);
    }
    /* USER CODE END uppergoingtask */
}
