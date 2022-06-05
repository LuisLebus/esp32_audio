/**
 * @file device.c
 * @author
 * @date
 * @brief
 */

//=============================================================================
// [Inclusions] ===============================================================
#include <string.h>

#include "esp_system.h"
#include "esp_ota_ops.h"

#include "device.h"
#include "nvs_application.h"
#include "nvs_keys.h"

//=============================================================================

//=============================================================================
// [Private Defines] ==========================================================
#define DEVICE_FAMILY       "sensors"
#define DEVICE_MODEL        "T700"

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
static device_config_t device_config = {0};
static device_state_t device_state = {0};

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

void device_init(void)
{
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    sprintf(device_config.id, "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    nvs_read_string(NVS_KEY_DEV_SN, device_config.sn);

    nvs_read_u32(NVS_KEY_DEV_RESETS, &device_state.resets);
    device_state.resets++;
    nvs_write_u32(NVS_KEY_DEV_RESETS, device_state.resets);


    esp_app_desc_t running_app_info = {0};
    const esp_partition_t* running = esp_ota_get_running_partition();
    if (ESP_OK == esp_ota_get_partition_description(running, &running_app_info))
    {
        strncpy(device_config.version, running_app_info.version, sizeof(device_config.version));
    }

    strncpy(device_config.family, DEVICE_FAMILY, sizeof(device_config.family));
    strncpy(device_config.model, DEVICE_MODEL, sizeof(device_config.model));
}
// End void device_init(void)


void device_reset(void)
{
    esp_restart();
}
// End void device_reset(void)


void device_set_sn(char * sn)
{
    if (true == nvs_write_string(NVS_KEY_DEV_SN, sn))
    {
        strncpy(device_config.sn, sn, sizeof(device_config.sn));
    }
}
// End void device_set_sn(char * value)


void device_set_resets(uint32_t resets)
{
    if (true == nvs_write_u32(NVS_KEY_DEV_RESETS, resets))
    {
        device_state.resets = resets;
    }
}
// End void device_set_resets(uint32_t resets)


device_config_t device_get_config(void)
{
    return device_config;
}
// End device_config_t device_get_config(void)


device_state_t device_get_state(void)
{
    return device_state;
}
// End device_state_t device_get_state(void)

//=============================================================================

//====================== [End Document] =======================================
