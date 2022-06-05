/**
 * @file nvs_application.c
 * @author
 * @date
 * @brief
 */

//=============================================================================
// [Inclusions] ===============================================================
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "nvs_flash.h"

#include "nvs_application.h"

//=============================================================================

//=============================================================================
// [Private Defines] ==========================================================
#define PARTITION_NAME  "config_param"

//=============================================================================

//=============================================================================
// [Local Typedef] ============================================================

//=============================================================================

//=============================================================================
// [External Data Definitions] ================================================

// Const ---------------------------------------------
//----------------------------------------------------

// Vars ----------------------------------------------
//----------------------------------------------------

// Task Handlers -------------------------------------
//----------------------------------------------------

// Queue Handlers ------------------------------------
//----------------------------------------------------

// Event Group Handlers ------------------------------
//----------------------------------------------------

// Semaphore Handlers --------------------------------
//----------------------------------------------------

//=============================================================================

//=============================================================================
// [Local Data Declarations] ==================================================

// Const ---------------------------------------------
//----------------------------------------------------

// Vars ----------------------------------------------
//----------------------------------------------------

// Task Handlers -------------------------------------
//----------------------------------------------------

// Queue Handlers ------------------------------------
//----------------------------------------------------

// Event Group Handlers ------------------------------
//----------------------------------------------------

// Semaphore Handlers --------------------------------
static SemaphoreHandle_t nvs_mutex = NULL;

//----------------------------------------------------

//=============================================================================

//=============================================================================
// [Local Function Declarations] ==============================================

//=============================================================================

//=============================================================================
// [FreeRTOS Task Definitions] ================================================

//=============================================================================

//=============================================================================
// [Local Function Definitions] ===============================================

//=============================================================================

//=============================================================================
// [External Function Definition] =============================================
void nvs_init(void)
{
    if (nvs_flash_init() != ESP_OK)
    {
        nvs_flash_erase();
        nvs_flash_init();
    }

    nvs_mutex = xSemaphoreCreateMutex();
    xSemaphoreGive(nvs_mutex);
}

bool nvs_write_u32(const char * key, uint32_t value)
{
    bool ret_val = false;
    nvs_handle_t handle = 0;

    //Enter critical
    if (pdTRUE == xSemaphoreTake(nvs_mutex, portMAX_DELAY))
    {
        if (ESP_OK == nvs_open(PARTITION_NAME, NVS_READWRITE, &handle))
        {
            if (ESP_OK == nvs_set_u32(handle, key, value))
            {
                ret_val = true;
            }

            nvs_commit(handle);
            nvs_close(handle);
        }

        //Exit critical
        xSemaphoreGive(nvs_mutex);
    }

    return ret_val;
}

bool nvs_read_u32(const char * key, uint32_t * value)
{
    nvs_handle_t handle = 0;
    bool ret_val = false;

    //Enter critical
    if (pdTRUE == xSemaphoreTake(nvs_mutex, portMAX_DELAY))
    {
        if (ESP_OK == nvs_open(PARTITION_NAME, NVS_READONLY, &handle))
        {
            if (ESP_OK == nvs_get_u32(handle, key, value))
            {
                ret_val = true;
            }

            nvs_close(handle);
        }

        //Exit critical
        xSemaphoreGive(nvs_mutex);
    }

    return ret_val;
}

bool nvs_write_u16(const char * key, uint16_t value)
{
    bool ret_val = false;
    nvs_handle_t handle = 0;

    //Enter critical
    if (pdTRUE == xSemaphoreTake(nvs_mutex, portMAX_DELAY))
    {
        if (ESP_OK == nvs_open(PARTITION_NAME, NVS_READWRITE, &handle))
        {
            if (ESP_OK == nvs_set_u16(handle, key, value))
            {
                ret_val = true;
            }

            nvs_commit(handle);
            nvs_close(handle);
        }

        //Exit critical
        xSemaphoreGive(nvs_mutex);
    }

    return ret_val;
}

bool nvs_read_u16(const char * key, uint16_t * value)
{
    nvs_handle_t handle = 0;
    bool ret_val = false;

    //Enter critical
    if (pdTRUE == xSemaphoreTake(nvs_mutex, portMAX_DELAY))
    {
        if (ESP_OK == nvs_open(PARTITION_NAME, NVS_READONLY, &handle))
        {
            if (ESP_OK == nvs_get_u16(handle, key, value))
            {
                ret_val = true;
            }

            nvs_close(handle);
        }

        //Exit critical
        xSemaphoreGive(nvs_mutex);
    }

    return ret_val;
}


bool nvs_write_i16(const char * key, int16_t value)
{
    nvs_handle_t handle = 0;
    bool ret_val = false;

    //Enter critical
    if (pdTRUE == xSemaphoreTake(nvs_mutex, portMAX_DELAY))
    {
        if (ESP_OK == nvs_open(PARTITION_NAME, NVS_READWRITE, &handle))
        {
            if (ESP_OK == nvs_set_i16(handle, key, value))
            {
                ret_val = true;
            }

            nvs_close(handle);
        }

        //Exit critical
        xSemaphoreGive(nvs_mutex);
    }

    return ret_val;
}

bool nvs_read_i16(const char * key, int16_t * value)
{
    nvs_handle_t handle = 0;
    bool ret_val = false;

    //Enter critical
    if (pdTRUE == xSemaphoreTake(nvs_mutex, portMAX_DELAY))
    {
        if (ESP_OK == nvs_open(PARTITION_NAME, NVS_READONLY, &handle))
        {
            if (ESP_OK == nvs_get_i16(handle, key, value))
            {
                ret_val = true;
            }

            nvs_close(handle);
        }

        //Exit critical
        xSemaphoreGive(nvs_mutex);
    }

    return ret_val;
}

bool nvs_write_u8(const char * key, uint8_t value)
{
    bool ret_val = false;
    nvs_handle_t handle = 0;

    //Enter critical
    if (pdTRUE == xSemaphoreTake(nvs_mutex, portMAX_DELAY))
    {
        if (ESP_OK == nvs_open(PARTITION_NAME, NVS_READWRITE, &handle))
        {
            if (ESP_OK == nvs_set_u8(handle, key, value))
            {
                ret_val = true;
            }

            nvs_commit(handle);
            nvs_close(handle);
        }

        //Exit critical
        xSemaphoreGive(nvs_mutex);
    }

    return ret_val;
}

bool nvs_read_u8(const char * key, uint8_t * value)
{
    nvs_handle_t handle = 0;
    bool ret_val = false;

    //Enter critical
    if (pdTRUE == xSemaphoreTake(nvs_mutex, portMAX_DELAY))
    {
        if (ESP_OK == nvs_open(PARTITION_NAME, NVS_READONLY, &handle))
        {
            if (ESP_OK == nvs_get_u8(handle, key, value))
            {
                ret_val = true;
            }

            nvs_close(handle);
        }

        //Exit critical
        xSemaphoreGive(nvs_mutex);
    }

    return ret_val;
}

bool nvs_write_i8(const char * key, int8_t value)
{
    bool ret_val = false;
    nvs_handle_t handle = 0;

    //Enter critical
    if (pdTRUE == xSemaphoreTake(nvs_mutex, portMAX_DELAY))
    {
        if (ESP_OK == nvs_open(PARTITION_NAME, NVS_READWRITE, &handle))
        {
            if (ESP_OK == nvs_set_i8(handle, key, value))
            {
                ret_val = true;
            }

            nvs_commit(handle);
            nvs_close(handle);
        }

        //Exit critical
        xSemaphoreGive(nvs_mutex);
    }

    return ret_val;
}

bool nvs_read_i8(const char * key, int8_t * value)
{
    nvs_handle_t handle = 0;
    bool ret_val = false;

    //Enter critical
    if (pdTRUE == xSemaphoreTake(nvs_mutex, portMAX_DELAY))
    {
        if (ESP_OK == nvs_open(PARTITION_NAME, NVS_READONLY, &handle))
        {
            if (ESP_OK == nvs_get_i8(handle, key, value))
            {
                ret_val = true;
            }

            nvs_close(handle);
        }

        //Exit critical
        xSemaphoreGive(nvs_mutex);
    }

    return ret_val;
}

bool nvs_write_bool(const char * key, bool value)
{
    bool ret_val = false;
    nvs_handle_t handle = 0;

    //Enter critical
    if (pdTRUE == xSemaphoreTake(nvs_mutex, portMAX_DELAY))
    {
        if (ESP_OK == nvs_open(PARTITION_NAME, NVS_READWRITE, &handle))
        {
            if (ESP_OK == nvs_set_u8(handle, key, (uint8_t)value))
            {
                ret_val = true;
            }

            nvs_commit(handle);
            nvs_close(handle);
        }

        //Exit critical
        xSemaphoreGive(nvs_mutex);
    }

    return ret_val;
}

bool nvs_read_bool(const char * key, bool * value)
{
    nvs_handle_t handle = 0;
    bool ret_val = false;

    //Enter critical
    if (pdTRUE == xSemaphoreTake(nvs_mutex, portMAX_DELAY))
    {
        if (ESP_OK == nvs_open(PARTITION_NAME, NVS_READONLY, &handle))
        {
            if (ESP_OK == nvs_get_u8(handle, key, (uint8_t *)value))
            {
                ret_val = true;
            }

            nvs_close(handle);
        }

        //Exit critical
        xSemaphoreGive(nvs_mutex);
    }

    return ret_val;
}

bool nvs_write_blob(const char * key, uint8_t * value, size_t len)
{
    nvs_handle_t handle = 0;
    bool ret_val = false;

    //Enter critical
    if (pdTRUE == xSemaphoreTake(nvs_mutex, portMAX_DELAY))
    {
        if (ESP_OK == nvs_open(PARTITION_NAME, NVS_READWRITE, &handle))
        {
            if (ESP_OK == nvs_set_blob(handle, key, value, len))
            {
                ret_val = true;
            }

            nvs_commit(handle);
            nvs_close(handle);
        }

        //Exit critical
        xSemaphoreGive(nvs_mutex);
    }

    return ret_val;
}

bool nvs_read_blob(const char * key, uint8_t * value, size_t * len)
{
    nvs_handle_t handle = 0;
    bool ret_val = false;

    //Enter critical
    if (pdTRUE == xSemaphoreTake(nvs_mutex, portMAX_DELAY))
    {
        if (ESP_OK == nvs_open(PARTITION_NAME, NVS_READONLY, &handle))
        {
            if (ESP_OK == nvs_get_blob(handle, key, value, len))
            {
                ret_val = true;
            }

            nvs_close(handle);
        }

        //Exit critical
        xSemaphoreGive(nvs_mutex);
    }

    return ret_val;
}

bool nvs_write_string(const char * key, char * value)
{
    nvs_handle_t handle = 0;
    bool ret_val = false;

    //Enter critical
    if (pdTRUE == xSemaphoreTake(nvs_mutex, portMAX_DELAY))
    {
        if (ESP_OK == nvs_open(PARTITION_NAME, NVS_READWRITE, &handle))
        {
            if (ESP_OK == nvs_set_str(handle, key, value))
            {
                ret_val = true;
            }

            nvs_commit(handle);
            nvs_close(handle);
        }

        //Exit critical
        xSemaphoreGive(nvs_mutex);
    }

    return ret_val;
}

bool nvs_read_string(const char * key, char * value)
{
    nvs_handle_t handle = 0;
    bool ret_val = false;
    size_t required_size = 0;

    //Enter critical
    if (pdTRUE == xSemaphoreTake(nvs_mutex, portMAX_DELAY))
    {
        if (ESP_OK == nvs_open(PARTITION_NAME, NVS_READONLY, &handle))
        {
            if (ESP_OK == nvs_get_str(handle, key, NULL, &required_size))
            {
                if (ESP_OK == nvs_get_str(handle, key, value, &required_size))
                {
                    ret_val = true;
                }
            }

            nvs_close(handle);
        }

        //Exit critical
        xSemaphoreGive(nvs_mutex);
    }

    return ret_val;
}


//=============================================================================

//====================== [End Document] =======================================
