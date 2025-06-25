#include "sd_card_manager.h"
#include "fatfs.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include "task2.h"

static FATFS sdFatFs;
static uint8_t sd_initialized = 0;

uint8_t SD_Init(void)
{
    FRESULT fresult;

    if (sd_initialized)
    {
        return 1;
    }

    if (BSP_SD_Init() != MSD_OK)
    {
        return 0;
    }

    fresult = f_mount(&sdFatFs, "", 1);
    if (fresult != FR_OK)
    {
        return 0;
    }

    sd_initialized = 1;
    return 1;
}

uint8_t SD_WriteFile(const char *filename, const char *content, uint8_t append)
{
    FIL file;
    FRESULT fresult;
    UINT bytesWritten;
    BYTE mode;

    if (!sd_initialized)
    {
        if (!SD_Init())
        {
            return 0;
        }
    }

    mode = append ? (FA_OPEN_APPEND | FA_WRITE) : (FA_CREATE_ALWAYS | FA_WRITE);

    fresult = f_open(&file, filename, mode);
    if (fresult != FR_OK)
    {
        return 0;
    }

    fresult = f_write(&file, content, strlen(content), &bytesWritten);

    f_close(&file);

    return (fresult == FR_OK && bytesWritten == strlen(content)) ? 1 : 0;
}

uint8_t SD_CreateTeamFile(void)
{
    char content[50];
    sprintf(content, "Nhom: %d", GROUP_NUMBER);
    return SD_WriteFile(FILE_NAME, content, 0);
}

int SD_ReadFile(const char *filename, char *buffer, uint32_t buffer_size)
{
    FIL file;
    FRESULT fresult;
    UINT bytesRead = 0;

    if (!sd_initialized)
    {
        if (!SD_Init())
        {
            return -1;
        }
    }

    fresult = f_open(&file, filename, FA_READ);
    if (fresult != FR_OK)
    {
        return -1;
    }

    UINT size_to_read = f_size(&file);
    if (size_to_read >= buffer_size)
    {
        size_to_read = buffer_size - 1;
    }

    fresult = f_read(&file, buffer, size_to_read, &bytesRead);

    f_close(&file);

    if (fresult != FR_OK)
    {
        return -1;
    }

    buffer[bytesRead] = '\0';

    return bytesRead;
}

int SD_ReadTeamFile(char *buffer, uint32_t buffer_size)
{
    return SD_ReadFile(FILE_NAME, buffer, buffer_size);
}
