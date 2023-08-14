#ifndef __SDCARDTASK_H
#define __SDCARDTASK_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "ff.h"
#include <stdio.h>
#include <string.h>
#include "bsp_spi_sdcard.h"

#include "counter.h"

// #define SDLOG()
enum SDWRITE
{
    SD_START,
    SD_WARNING,
    SD_INFO,
    SD_ERROR
};

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
    FAILED = 0,
    PASSED = !FAILED
} TestStatus;

/* Private define ------------------------------------------------------------*/
#define BLOCK_SIZE 512      /* Block Size in Bytes */
#define NUMBER_OF_BLOCKS 10 /* For Multi Blocks operation (Read/Write) */
#define MULTI_BUFFER_SIZE (BLOCK_SIZE * NUMBER_OF_BLOCKS)

typedef struct SDStatus
{
    uint8_t SD_init_result;     // SD����ʼ��
    uint8_t SD_FS_Write_result; // д���
    uint8_t SD_FS_Read_result;  // �����
    uint8_t SD_FS_Open_result;  // ���ļ����
    uint8_t SD_FS_Mount_result; // �����ļ�ϵͳ���
    uint8_t SDCard_task_init;   // SD������ɹ���ʼ�����

    SD_Error Status;
    FATFS fs;
    FIL fnew;
    FRESULT res_sd; /* �ļ�������� */
    UINT fnum;      /* �ļ��ɹ���д���� */

    int32_t count;

    uint32_t last_cnt;
    float delta_t;
} SDStatus;

void SDCard_task(void *pvParameters);
void SDLOG(enum SDWRITE write_type, const char *str);

#endif
