/**
 * @file dac_audio.c
 * @author
 * @date
 * @brief
 */

//=============================================================================
// [Inclusions] ===============================================================
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "driver/timer.h"
#include "driver/dac.h"
#include "esp_log.h"

#include "dac_audio.h"

#include "alarma.h"
#include "alert.h"
#include "countdown.h"
#include "game1.h"
#include "game2.h"
#include "telefono.h"
#include "configurado.h"

//=============================================================================

//=============================================================================
// [Private Defines] ==========================================================
#define DAC_AUDIO_BUFFER_SIZE   4096            // Size of buffer to store data to send to DAC. 3 bytes minimum!
                                                // 4000 bytes (Default) should allow for very slow main loops
                                                // that repeat only about every 100Hz, If your main loop is much faster
                                                // than this and the VAST majority will be you can safely reduce this size
                                                // without any issues down to an absolute minimum of 3 bytes
                                                // If you experience sound slowing down then you need to increase
                                                // Buffer memory. Which can be done in your code when you create the
                                                // DacAudio object
                                                // If your main loop is very very slow but you want to keep buffer
                                                // memory low then consider calling the DAC_Audio_Class FillBuffer
                                                // routine more than once in you main loop. Although to be affective
                                                // they should not be called in quick succession but farther apart in
                                                // time on your main loop.
                                                // Use the routine MaxBufferUsage in your main loop to output the max
                                                // buffer memory (via serial monitor) that your loop is using, you can
                                                // then set you buffer memory very precisely
#define DAC_AUDIO_BYTES_PER_SEC (8000.0)        // The rate at which bytes are sent to the DAC, note that the max
                                                // sample rate however is 44100Khz, this higher sampling rate allows
                                                // for samples to be increased in pitch. If a 44100 rate then the max
                                                // would be 3 x the pitch.


#define DAC_AUDIO_TIMER_DIVIDER     80
#define DAC_AUDIO_VALUE_TIMEOUT     (uint64_t)(1000000 * (1 / DAC_AUDIO_BYTES_PER_SEC))

#define DAC_AUDIO_SILENCE           0x7F

#define DAC_AUDIO_CH                DAC_CHANNEL_1

#define DAC_AUDIO_DATA_CHUNK_ID     0x61746164
#define DAC_AUDIO_FMT_CHUNK_ID      0x20746D66

// Convert 4 byte little-endian to a long.
#define LONGWORD(bfr, ofs)          ( (bfr[ofs + 3] << 24) | (bfr[ofs + 2] << 16) | (bfr[ofs + 1] << 8) | bfr[ofs + 0] )

#define DAC_AUDIO_DELAY_TASK        100

#define DAC_AUDIO_BIT               BIT0

//=============================================================================

//=============================================================================
// [Local Typedef] ============================================================

typedef struct
{
    bool playing;
    uint32_t data_size;
    uint32_t data_start;
    uint32_t data_idx;
    const uint8_t * data;
} dac_audio_config_t;


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

static dac_audio_config_t dac_audio_config[DAC_AUDIO_WAV_MAX_NUM] = {
        {
                .playing = false,
                .data = alarma,
        },
        {
                .playing = false,
                .data = alert,
        },
        {
                .playing = false,
                .data = countdown,
        },
        {
                .playing = false,
                .data = game1,
        },
        {
                .playing = false,
                .data = game2,
        },
        {
                .playing = false,
                .data = telefono,
        },
        {
                .playing = false,
                .data = configurado,
        }
};


static volatile uint32_t next_play_pos = 0;                      // Next position in buffer of next byte to play
static volatile uint32_t next_fill_pos = 0;                      // Next fill position in buffer to fill
static volatile uint8_t buffer[DAC_AUDIO_BUFFER_SIZE];           // The buffer to store the data that will be sent to the
static volatile uint8_t last_dac_value = DAC_AUDIO_SILENCE;      // Last value sent to DAC, if next is same we don't write to DAC again

//----------------------------------------------------

// Task Handlers -------------------------------------
//----------------------------------------------------

// Queue Handlers ------------------------------------
//----------------------------------------------------

// Event Group Handlers ------------------------------
//----------------------------------------------------

// Semaphore Handlers --------------------------------
static EventGroupHandle_t dac_audio_ev_group = NULL;


//----------------------------------------------------

//=============================================================================

//=============================================================================
// [Local Function Declarations] ==============================================

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
static void dac_audio_fill_buffer(void);

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
static uint8_t dac_audio_wav_next_byte(dac_audio_wav_t dac_audio_wav);

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
static void dac_audio_wav_init(dac_audio_wav_t dac_audio_wav);

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
static bool IRAM_ATTR dac_audio_on_timer(void * arg);

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
static void dac_audio_process_task(void * pvParameters);

//=============================================================================

//=============================================================================
// [FreeRTOS Task Definitions] ================================================

static void dac_audio_process_task(void * pvParameters)
{
    while(1)
    {
        dac_audio_fill_buffer();

        vTaskDelay(pdMS_TO_TICKS(DAC_AUDIO_DELAY_TASK));
    }
}
// End static void buzzer_process_task(void * pvParameters)

//=============================================================================

//=============================================================================
// [Local Function Definitions] ===============================================

static uint8_t dac_audio_wav_next_byte(dac_audio_wav_t dac_audio_wav)
{
    uint8_t return_value = dac_audio_config[dac_audio_wav].data[dac_audio_config[dac_audio_wav].data_idx];

    dac_audio_config[dac_audio_wav].data_idx++;
    if (dac_audio_config[dac_audio_wav].data_idx >= dac_audio_config[dac_audio_wav].data_size)
    {
        dac_audio_config[dac_audio_wav].playing = false;        // mark as completed
    }

    return return_value;
}


static void dac_audio_wav_init(dac_audio_wav_t dac_audio_wav)
{
    uint32_t ofs = 12;
    uint32_t siz = 0;

    /* Process the chunks.  "fmt " is format, "data" is the samples, ignore all else. */
    siz = LONGWORD(dac_audio_config[dac_audio_wav].data, 4);

    dac_audio_config[dac_audio_wav].data_start = 0;

    while (ofs < siz)
    {
        if (LONGWORD(dac_audio_config[dac_audio_wav].data, ofs) == DAC_AUDIO_DATA_CHUNK_ID)
        {
            dac_audio_config[dac_audio_wav].data_size = LONGWORD(dac_audio_config[dac_audio_wav].data, ofs + 4);
            dac_audio_config[dac_audio_wav].data_start = ofs + 8;
        }

        ofs += LONGWORD(dac_audio_config[dac_audio_wav].data, ofs + 4) + 8;
    }
}


static void dac_audio_fill_buffer(void)
{
    uint32_t bytes_available = 0;                                               // # of bytes that we can put into the buffer.

    bytes_available = DAC_AUDIO_BUFFER_SIZE - (next_fill_pos - next_play_pos);  // Work out how much buffer we can fill
    if (bytes_available >= DAC_AUDIO_BUFFER_SIZE)                               // This indicates the next fill pos was behind play pos,
    {
        bytes_available -= DAC_AUDIO_BUFFER_SIZE;                               //  bring into correct range
    }

    for (uint8_t i = 0; i < DAC_AUDIO_WAV_MAX_NUM; i++)
    {
        if (true == dac_audio_config[i].playing)
        {
            while (bytes_available > 0)                                         // while space in buffer
            {
                bytes_available--;

                buffer[next_fill_pos] = dac_audio_wav_next_byte(i);

                // Move the fill position to next pos
                next_fill_pos++;                                                // move to next buffer position
                if(next_fill_pos >= DAC_AUDIO_BUFFER_SIZE)                      // Set to start of buffer if end reached
                {
                    next_fill_pos = 0;
                }
            }

            break;
        }
    }
}


static bool IRAM_ATTR dac_audio_on_timer(void* arg)
{
    //Sound playing code, plays whatever's in the buffer continuously. Big change from previous versions
    if (last_dac_value != buffer[next_play_pos])        // Send value to DAC only if changed since last value else no need
    {
        //value to DAC has changed, send to actual hardware, else we just leave setting as is as it's not changed
        dac_output_voltage(DAC_AUDIO_CH, buffer[next_play_pos]);
        last_dac_value = buffer[next_play_pos];
    }

    buffer[next_play_pos] = DAC_AUDIO_SILENCE;          // Set this buffer byte to silence
    next_play_pos++;                                    // Move play pos to next byte in buffer
    if (next_play_pos >= DAC_AUDIO_BUFFER_SIZE)         // If gone past end of buffer,
    {
        next_play_pos = 0;                              // Set back to beginning
    }

    return true;
}

//=============================================================================

//=============================================================================
// [External Function Definition] =============================================

void dac_audio_init(void)
{
    dac_output_enable(DAC_AUDIO_CH);

    for (uint32_t i = 0; i < DAC_AUDIO_BUFFER_SIZE; i++)    // Set all bytes in buffer to 0x7F (DAC silence)
    {
        buffer[i] = DAC_AUDIO_SILENCE;
    }

    last_dac_value = DAC_AUDIO_SILENCE;                     // set to mid point
    dac_output_voltage(DAC_AUDIO_CH, last_dac_value);

    dac_audio_ev_group = xEventGroupCreate();

    timer_config_t timer_config = {0};
    timer_config.divider = DAC_AUDIO_TIMER_DIVIDER;
    timer_config.counter_dir = TIMER_COUNT_UP;
    timer_config.counter_en = TIMER_PAUSE;
    timer_config.alarm_en = TIMER_ALARM_EN;
    timer_config.auto_reload = TIMER_AUTORELOAD_EN;

    timer_init(TIMER_GROUP_0, TIMER_1, &timer_config);

    timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_1, DAC_AUDIO_VALUE_TIMEOUT);

    timer_enable_intr(TIMER_GROUP_0, TIMER_1);
    timer_isr_callback_add(TIMER_GROUP_0, TIMER_1, dac_audio_on_timer, NULL, 0);

    timer_start(TIMER_GROUP_0, TIMER_1);

    xTaskCreate(dac_audio_process_task, "dac_proc_ts", configMINIMAL_STACK_SIZE * 3, NULL, tskIDLE_PRIORITY + 1, NULL);
}
// End void dac_audio_init(void)


void dac_audio_play(dac_audio_wav_t dac_audio_wav)
{
    dac_audio_stop();

    if ((0 == dac_audio_config[dac_audio_wav].data_start) || (0 == dac_audio_config[dac_audio_wav].data_size))
    {
        dac_audio_wav_init(dac_audio_wav);
    }

    dac_audio_config[dac_audio_wav].data_idx = dac_audio_config[dac_audio_wav].data_start;

    dac_audio_config[dac_audio_wav].playing = true;
}
// End void dac_audio_play(dac_audio_wav_t dac_audio_wav)


void dac_audio_stop(void)
{
    for (uint8_t i = 0; i < DAC_AUDIO_WAV_MAX_NUM; i++)
    {
        dac_audio_config[i].playing = false;
    }
}
// End void void dac_audio_stop(void)

//=============================================================================

//====================== [End Document] =======================================
