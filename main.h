#ifndef MAIN_H
#define MAIN_H

#include "app_bt_utils.h"
#include "wiced_bt_stack.h"
#include "cycfg_bt_settings.h"
#include "cycfg_gap.h"
#include "cycfg_gatt_db.h"

typedef void (*pfn_free_buffer_t)(uint8_t *);

/* Callback function for Bluetooth stack management type events */
static wiced_bt_dev_status_t app_bt_management_callback             (wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);

/* GATT Event Callback and Handler Functions */
static wiced_bt_gatt_status_t app_bt_gatt_event_callback            (wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data);

static wiced_bt_gatt_status_t app_bt_connect_event_handler          (wiced_bt_gatt_connection_status_t *p_conn_status);
static wiced_bt_gatt_status_t app_bt_server_event_handler           (wiced_bt_gatt_event_data_t *p_data);

static wiced_bt_gatt_status_t app_bt_write_handler                  (wiced_bt_gatt_event_data_t *p_data);
static wiced_bt_gatt_status_t app_bt_gatt_req_read_handler          (uint16_t conn_id, wiced_bt_gatt_opcode_t opcode,
                                                                    wiced_bt_gatt_read_t *p_read_req, uint16_t len_requested);
static wiced_bt_gatt_status_t app_bt_gatt_req_read_multi_handler    (uint16_t conn_id, wiced_bt_gatt_opcode_t opcode,
                                                                    wiced_bt_gatt_read_multiple_req_t *p_read_req, uint16_t len_requested);
static wiced_bt_gatt_status_t app_bt_gatt_req_read_by_type_handler  (uint16_t conn_id, wiced_bt_gatt_opcode_t opcode,
                                                                    wiced_bt_gatt_read_by_type_t *p_read_req, uint16_t len_requested);

/* Helper functions to find GATT database handles and allocate/free buffers for GATT operations */
static gatt_db_lookup_table_t 	*app_bt_find_by_handle(uint16_t handle);
static uint8_t 					*app_bt_alloc_buffer(uint16_t len);
static void 					app_bt_free_buffer(uint8_t *p_data);

/* App functions */
void button_interrupt_handler(void *callback_arg, cyhal_gpio_event_t event);

#endif /* MAIN_H */