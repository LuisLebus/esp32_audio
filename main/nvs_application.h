/**
 * @file nvs_application.h
 * @author
 * @date
 * @brief
 */

#ifndef _NVS_APPLICATION_H_
#define _NVS_APPLICATION_H_

//=============================================================================
// [Inclusions] ===============================================================
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

//=============================================================================

//=============================================================================
// [External Defines] =========================================================

//=============================================================================

//=============================================================================
// [External Typedef] =========================================================

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
// [External Function Declarations] ===========================================

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
void nvs_init(void);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
bool nvs_write_u32(const char * key, uint32_t value);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
bool nvs_read_u32(const char * key, uint32_t * value);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
bool nvs_write_u16(const char * key, uint16_t value);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
bool nvs_read_u16(const char * key, uint16_t * value);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
bool nvs_write_i16(const char * key, int16_t value);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
bool nvs_read_i16(const char * key, int16_t * value);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
bool nvs_write_u8(const char * key, uint8_t value);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
bool nvs_read_u8(const char * key, uint8_t * value);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
bool nvs_write_i8(const char * key, int8_t value);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
bool nvs_read_i8(const char * key, int8_t * value);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
bool nvs_write_bool(const char * key, bool value);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
bool nvs_read_bool(const char * key, bool * value);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
bool nvs_write_blob(const char * key, uint8_t * value, size_t len);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
bool nvs_read_blob(const char * key, uint8_t * value, size_t * len);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
bool nvs_write_string(const char * key, char * value);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
bool nvs_read_string(const char * key, char * value);

//=============================================================================

#endif /* _NVS_APPLICATION_H_ */
