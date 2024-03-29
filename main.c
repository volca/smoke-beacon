/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

/** @file
 *
 * @defgroup ble_sdk_app_beacon_main main.c
 * @{
 * @ingroup ble_sdk_app_beacon
 * @brief Beacon Transmitter Sample Application main file.
 *
 * This file contains the source code for an Beacon transmitter sample application.
 */

#include <stdbool.h>
#include <stdint.h>
#include "ble_advdata.h"
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "bsp.h"
#include "app_timer.h"
#include "app_button.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define BTN_LONG_PUSH_TIMEOUT           APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER)
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation que  ues. */
#define CENTRAL_LINK_COUNT              0                                 /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           0                                 /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)    /**< Delay from a GPIOTE event u  ntil a button is reported as pushed (in number of timer ticks). */
#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                 /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define APP_CFG_NON_CONN_ADV_TIMEOUT    0                                 /**< Time for which the device must be advertising in non-connectable mode (in seconds). 0 disables timeout. */
#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(400, UNIT_0_625_MS) /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

#define APP_BEACON_INFO_LENGTH          0x17                              /**< Total length of information advertised by the Beacon. */
#define APP_ADV_DATA_LENGTH             0x15                              /**< Length of manufacturer specific data in the advertisement. */
#define APP_DEVICE_TYPE                 0x02                              /**< 0x02 refers to Beacon. */
#define APP_MEASURED_RSSI               0xC3                              /**< The Beacon's measured RSSI at 1 meter distance in dBm. */
#define APP_COMPANY_IDENTIFIER          0x004C                            /**< Company identifier for Apple. as per www.bluetooth.org. */
#define APP_MAJOR_VALUE                 0x01, 0x02                        /**< Major value used to identify Beacons. */
#define APP_MINOR_VALUE                 0x03, 0x04                        /**< Minor value used to identify Beacons. */
#define APP_BEACON_UUID                 0xFD, 0xA5, 0x06, 0x93, \
                                        0xA4, 0xE2,0x4F, 0xB1, \
                                        0xAF, 0xCF, 0xC6, 0xEB, \
                                        0x07, 0x64, 0x78, 0x25            /**< Proprietary UUID for Beacon. */

#define DEAD_BEEF                       0xDEADBEEF                        /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define APP_TIMER_PRESCALER             0                                 /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                 /**< Size of timer operation queues. */

#define ADV_TIMEOUT_TIME                APP_TIMER_TICKS(15000, APP_TIMER_PRESCALER)  

#define SERVICE_UUID                    0x181c
#define SERVICE_DATA_LEN                10

#define DEVICE_NAME                     "absmk"

APP_TIMER_DEF(m_adv_tmr);
APP_TIMER_DEF(m_btn_tmr);

static ble_gap_addr_t m_gap_addr = {
    .addr_type = BLE_GAP_ADDR_TYPE_PUBLIC
};

static ble_gap_adv_params_t m_adv_params;                                 /**< Parameters to be passed to the stack when starting advertising. */

static uint8_t m_service_data[SERVICE_DATA_LEN];

// We'll put our sensor data inside an advertisement service.
static ble_advdata_service_data_t m_advdata_service_data = {
    .service_uuid = SERVICE_UUID,
    .data = {
        .p_data = m_service_data,
        .size = SERVICE_DATA_LEN,
    }
};

// Holds the service data to be broadcasted. The contents of this struct
// will be encoded into gap_adv_data.
// Warning: do not update this while advertising.
static ble_advdata_t m_adv_data = {
    .name_type = BLE_ADVDATA_FULL_NAME,
    .flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED,
    .p_service_data_array = &m_advdata_service_data,
    .service_data_count = 1,
};

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

static void nrf51_addr_get(ble_gap_addr_t *gap_addr) {
    uint32_t addr[2];
    addr[0] = NRF_FICR->DEVICEADDR[0];
    addr[1] = NRF_FICR->DEVICEADDR[1];
    addr[1] |= 0xc000;
    memcpy(gap_addr->addr, (uint8_t *)addr, 6);
    /*
    gap_addr->addr[0] = (NRF_FICR->DEVICEADDR[1] >> 8) & 0xFF;
    gap_addr->addr[1] = (NRF_FICR->DEVICEADDR[1]) & 0xFF;
    gap_addr->addr[2] = (NRF_FICR->DEVICEADDR[0] >> 24) & 0xFF;
    gap_addr->addr[3] = (NRF_FICR->DEVICEADDR[0] >> 16) & 0xFF;
    gap_addr->addr[4] = (NRF_FICR->DEVICEADDR[0] >>  8) & 0xFF;
    gap_addr->addr[5] = (NRF_FICR->DEVICEADDR[0] >>  0) & 0xFF;
    */
}

static void set_service_data_bthome_protocol(uint8_t detected) 
{
    // 1. Smoke
    // uint18_t.
    m_service_data[0] = 2;
    // Type - smoke.
    m_service_data[1] = 0x29;
    // Value. 1 = detected
    m_service_data[2] = detected;
    // TODO
    // 2. Mac
    // uint18_t.
    m_service_data[3] = 0x86;
    // Value.
    // BLE_GAP_ADDR_TYPE_PUBLIC
    nrf51_addr_get(&m_gap_addr);
    m_service_data[4] = m_gap_addr.addr[0];
    m_service_data[5] = m_gap_addr.addr[1];
    m_service_data[6] = m_gap_addr.addr[2];
    m_service_data[7] = m_gap_addr.addr[3];
    m_service_data[8] = m_gap_addr.addr[4];
    m_service_data[9] = m_gap_addr.addr[5];
}

static void gap_params_init() {
    uint32_t                err_code;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    set_service_data_bthome_protocol(0);
    err_code = ble_advdata_set(&m_adv_data, NULL);
    APP_ERROR_CHECK(err_code);

    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
    m_adv_params.p_peer_addr = NULL;                             // Undirected advertisement.
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = NON_CONNECTABLE_ADV_INTERVAL;
    m_adv_params.timeout     = APP_CFG_NON_CONN_ADV_TIMEOUT;
}

/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_start(&m_adv_params);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
}

static void advertising_swap(uint8_t idx) {
    uint32_t      err_code;
    set_service_data_bthome_protocol(idx);
    err_code = ble_advdata_set(&m_adv_data, NULL);
    NRF_LOG_INFO("Button state:%d err:%d\r\n", idx, err_code);
}

/**@brief Function for doing power management.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    switch (pin_no)
    {
        case BUTTON_2:
            if (button_action == APP_BUTTON_PUSH) {
                app_timer_stop(m_btn_tmr);
                uint32_t err_code = app_timer_start(
                    m_btn_tmr,
                    BTN_LONG_PUSH_TIMEOUT,
                    NULL
                );
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            break;
    }
}

static void on_btn_long_push() {
    bsp_indication_set(BSP_INDICATE_IDLE);
    advertising_swap(1);
    uint32_t err_code = app_timer_stop(m_adv_tmr);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(m_adv_tmr, ADV_TIMEOUT_TIME, NULL);
    APP_ERROR_CHECK(err_code);
}

static void btn_timeout_handler(void * p_context) {
    UNUSED_PARAMETER(p_context);
    on_btn_long_push();
}

static void adv_timeout_handler(void * p_context) {
    UNUSED_PARAMETER(p_context);
    advertising_swap(0);
    NRF_LOG_INFO("adv timeout");
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    // Initialize timer module, making it use the scheduler
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers.
    uint32_t err_code = app_timer_create(&m_adv_tmr, APP_TIMER_MODE_SINGLE_SHOT, adv_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_btn_tmr, APP_TIMER_MODE_SINGLE_SHOT, btn_timeout_handler);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;
    // Initialize.
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
    err_code = bsp_init(BSP_INIT_LED, APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), NULL);
    APP_ERROR_CHECK(err_code);

    timers_init();
    //The array must be static because a pointer to it will be saved in the button handler module.
    static app_button_cfg_t buttons[] =
    {
        {BUTTON_2, false, BUTTON_PULL, button_event_handler}
    };
    err_code = app_button_init(buttons, sizeof(buttons) / sizeof(buttons[0]),
            BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);
    app_button_enable();

    ble_stack_init();
    gap_params_init();
    advertising_init();

    // Start execution.
    NRF_LOG_INFO("BLE Beacon started\r\n");
    advertising_start();

    // Enter main loop.
    for (;; )
    {
        if (NRF_LOG_PROCESS() == false)
        {
            power_manage();
        }
    }
}


/**
 * @}
 */
