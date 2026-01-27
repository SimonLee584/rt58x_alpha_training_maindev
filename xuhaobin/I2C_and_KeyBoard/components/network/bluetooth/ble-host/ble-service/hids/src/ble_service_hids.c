/*
 * Copyright (c) 2022-2025 Rafael Microelectronics Inc. All rights reserved.
 * 
 * SPDX-License-Identifier: LicenseRef-RafaelMicro-Proprietary-1.0
 *
 */

/************************************************************************
 *
 * File Name  : ble_service_hids.c
 * Description: This file contains the definitions and functions of BLE HIDS
 *
 *
 ************************************************************************/
#include "ble_service_hids.h"
#include "ble_profile.h"

/** ble_svcs_hids_handler
 * @note This callback receives the HIDS events.
 * Each of these events can be associated with parameters.
*/

void ble_svcs_hids_handler(ble_evt_att_param_t *p_param);


/**************************************************************************
 * HIDS UUID Definitions
 **************************************************************************/

/** HIDS UUID.
 * @note 16-bits UUID
 * @note UUID: 1812
*/

const uint16_t attr_uuid_hids_primary_service[] =
{
    0x1812,
};

/** HIDS characteristic INFORMATION UUID.
 * @note 16-bits UUID
 * @note UUID: 2A4A
*/

const uint16_t attr_uuid_hids_charc_information[] =
{
    0x2A4A,
};

/** HIDS characteristic CONTROL_POINT UUID.
 * @note 16-bits UUID
 * @note UUID: 2A4C
*/

const uint16_t attr_uuid_hids_charc_control_point[] =
{
    0x2A4C,
};

/** HIDS characteristic PROTOCOL_MODE UUID.
 * @note 16-bits UUID
 * @note UUID: 2A4E
*/

const uint16_t attr_uuid_hids_charc_protocol_mode[] =
{
    0x2A4E,
};

/** HIDS characteristic REPORT_MAP UUID.
 * @note 16-bits UUID
 * @note UUID: 2A4B
*/

const uint16_t attr_uuid_hids_charc_report_map[] =
{
    0x2A4B,
};

/** HIDS characteristic BOOT_KEYBOARD_INPUT_REPORT UUID.
 * @note 16-bits UUID
 * @note UUID: 2A22
*/

const uint16_t attr_uuid_hids_charc_boot_keyboard_input_report[] =
{
    0x2A22,
};

/** HIDS characteristic BOOT_KEYBOARD_OUTPUT_REPORT UUID.
 * @note 16-bits UUID
 * @note UUID: 2A32
*/

const uint16_t attr_uuid_hids_charc_boot_keyboard_output_report[] =
{
    0x2A32,
};

/** HIDS characteristic KEYBOARD_INPUT_REPORT UUID.
 * @note 16-bits UUID
 * @note UUID: 2A4D
*/

const uint16_t attr_uuid_hids_charc_keyboard_input_report[] =
{
    0x2A4D,
};

/** HIDS characteristic KEYBOARD_OUTPUT_REPORT UUID.
 * @note 16-bits UUID
 * @note UUID: 2A4D
*/

const uint16_t attr_uuid_hids_charc_keyboard_output_report[] =
{
    0x2A4D,
};

/** HIDS characteristic BOOT_MOUSE_INPUT_REPORT UUID.
 * @note 16-bits UUID
 * @note UUID: 2A33
*/

/**************************************************************************
 * HIDS Service Value Definitions
 **************************************************************************/
/** HID characteristic HID Information value.
 * @note Return the "Read data" when central send "Read Request".
*/
const uint8_t hids_information[] =
{
    0x11, 0x01,     //bcdHID: 0x0213
    0x40,           //bCountryCode: 0x40
    0x01,           //Flags: 0x_1 (providing wake-up signal to a HID host)
};

/** HID characteristic HID report map value.
 * @note Return the "Read data" when central send "Read Request".
*/
const uint8_t hids_report_map[] =    //Device Class Definition for Human Interface Devices (HID) Version 1.11, 6.2.2 Report Descriptor
{
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x06,        // Usage (Keyboard)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x01,        //   Report ID (1)

    // Modifier keys (左Ctrl, Shift, Alt, GUI, 右Ctrl, Shift, Alt, GUI)
    0x05, 0x07,        //   Usage Page (Key Codes)
    0x19, 0xE0,        //   Usage Minimum (Keyboard LeftControl)
    0x29, 0xE7,        //   Usage Maximum (Keyboard Right GUI)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x08,        //   Report Count (8)
    0x81, 0x02,        //   Input (Data, Variable, Absolute) - Modifier byte
    
    // Reserved byte (must be 0)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x01,        //   Input (Constant) - Reserved byte
    
    // LED report (Num Lock, Caps Lock, Scroll Lock, Compose, Kana)
    0x05, 0x08,        //   Usage Page (LEDs)
    0x19, 0x01,        //   Usage Minimum (Num Lock)
    0x29, 0x05,        //   Usage Maximum (Kana)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x05,        //   Report Count (5)
    0x91, 0x02,        //   Output (Data, Variable, Absolute) - LED report
    0x75, 0x03,        //   Report Size (3)
    0x95, 0x01,        //   Report Count (1)
    0x91, 0x01,        //   Output (Constant) - LED padding
    
    // Key arrays (最多支持6个按键同时按下)
    0x05, 0x07,        //   Usage Page (Key Codes)
    0x19, 0x00,        //   Usage Minimum (0)
    0x29, 0x65,        //   Usage Maximum (Keyboard Application)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x65,        //   Logical Maximum (101)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x06,        //   Report Count (6)
    0x81, 0x00,        //   Input (Data, Array) - Key arrays
    
    0xC0               // End Collection
};

/** HID characteristic HID protocol mode value.
 * @note Return the "Read data" when central send "Read Request".
*/
const uint8_t hids_protocol_mode[] =
{
    0x01,
};

/** KEYBOARD_INPUT_REPORT report reference definition.
 * @note Return the "description value" when central send "Read Request".
*/
const uint8_t hids_keyboard_input_report_reference[] =
{
    0x01,    //Report ID
    0x01     //Report Type: Input (0x01)
};

/** KEYBOARD_OUTPUT_REPORT report reference definition.
 * @note Return the "description value" when central send "Read Request".
*/
const uint8_t hids_keyboard_output_report_reference[] =
{
    0x01,    //Report ID : 01
    0x02     //Report Type: Output (0x02)
};

/**************************************************************************
 * HIDS Service/ Characteristic Definitions
 **************************************************************************/

const ble_att_param_t att_hids_primary_service =
{
    (void *)attr_uuid_type_primary_service,
    (void *)attr_uuid_hids_primary_service,
    sizeof(attr_uuid_hids_primary_service),
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    attr_null_access,                       //This function should be set to attr_null_access when att_len or p_uuid_value is a null value.
};

const ble_att_param_t att_hids_characteristic_information =
{
    (void *)attr_uuid_type_characteristic,
    (void *)attr_uuid_hids_charc_information,
    sizeof(attr_uuid_hids_charc_information),
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    attr_null_access,                       //This function should be set to attr_null_access when att_len or p_uuid_value is a null value.
};

const ble_att_param_t att_hids_information =
{
    (void *)attr_uuid_hids_charc_information,
    (void *)0,
    0,
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    ble_svcs_hids_handler,       //registered callback function
};

const ble_att_param_t att_hids_characteristic_control_point =
{
    (void *)attr_uuid_type_characteristic,
    (void *)attr_uuid_hids_charc_control_point,
    sizeof(attr_uuid_hids_charc_control_point),
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    attr_null_access,                       //This function should be set to attr_null_access when att_len or p_uuid_value is a null value.
};

const ble_att_param_t att_hids_control_point =
{
    (void *)attr_uuid_hids_charc_control_point,
    (void *)0,
    0,
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        //GATT_DECLARATIONS_PROPERTIES_READ |
        GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    ble_svcs_hids_handler,       //registered callback function
};

const ble_att_param_t att_hids_characteristic_protocol_mode =
{
    (void *)attr_uuid_type_characteristic,
    (void *)attr_uuid_hids_charc_protocol_mode,
    sizeof(attr_uuid_hids_charc_protocol_mode),
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    attr_null_access,                       //This function should be set to attr_null_access when att_len or p_uuid_value is a null value.
};

const ble_att_param_t att_hids_protocol_mode =
{
    (void *)attr_uuid_hids_charc_protocol_mode,
    (void *)0,
    0,
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    ble_svcs_hids_handler,       //registered callback function
};

const ble_att_param_t att_hids_characteristic_report_map =
{
    (void *)attr_uuid_type_characteristic,
    (void *)attr_uuid_hids_charc_report_map,
    sizeof(attr_uuid_hids_charc_report_map),
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    attr_null_access,                       //This function should be set to attr_null_access when att_len or p_uuid_value is a null value.
};

const ble_att_param_t att_hids_report_map =
{
    (void *)attr_uuid_hids_charc_report_map,
    (void *)0,
    0,
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    ble_svcs_hids_handler,       //registered callback function
};

const ble_att_param_t att_hids_characteristic_boot_keyboard_input_report =
{
    (void *)attr_uuid_type_characteristic,
    (void *)attr_uuid_hids_charc_boot_keyboard_input_report,
    sizeof(attr_uuid_hids_charc_boot_keyboard_input_report),
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    attr_null_access,                       //This function should be set to attr_null_access when att_len or p_uuid_value is a null value.
};

const ble_att_param_t att_hids_boot_keyboard_input_report =
{
    (void *)attr_uuid_hids_charc_boot_keyboard_input_report,
    (void *)0,
    0,
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        GATT_DECLARATIONS_PROPERTIES_WRITE |
        GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    ble_svcs_hids_handler,       //registered callback function
};

const ble_att_param_t att_hids_boot_keyboard_input_report_client_charc_configuration =
{
    (void *)attr_uuid_type_client_charc_configuration,
    (void *)0,
    0,
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    ble_svcs_hids_handler,       //registered callback function
};

const ble_att_param_t att_hids_characteristic_boot_keyboard_output_report =
{
    (void *)attr_uuid_type_characteristic,
    (void *)attr_uuid_hids_charc_boot_keyboard_output_report,
    sizeof(attr_uuid_hids_charc_boot_keyboard_output_report),
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    attr_null_access,                       //This function should be set to attr_null_access when att_len or p_uuid_value is a null value.
};

const ble_att_param_t att_hids_boot_keyboard_output_report =
{
    (void *)attr_uuid_hids_charc_boot_keyboard_output_report,
    (void *)0,
    0,
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    ble_svcs_hids_handler,       //registered callback function
};

const ble_att_param_t att_hids_characteristic_keyboard_input_report =
{
    (void *)attr_uuid_type_characteristic,
    (void *)attr_uuid_hids_charc_keyboard_input_report,
    sizeof(attr_uuid_hids_charc_keyboard_input_report),
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    attr_null_access,                       //This function should be set to attr_null_access when att_len or p_uuid_value is a null value.
};

const ble_att_param_t att_hids_keyboard_input_report =
{
    (void *)attr_uuid_hids_charc_keyboard_input_report,
    (void *)0,
    0,
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        GATT_DECLARATIONS_PROPERTIES_WRITE |
        GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    ble_svcs_hids_handler,       //registered callback function
};

const ble_att_param_t att_hids_keyboard_input_report_client_charc_configuration =
{
    (void *)attr_uuid_type_client_charc_configuration,
    (void *)0,
    0,
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    ble_svcs_hids_handler,       //registered callback function
};

const ble_att_param_t att_hids_keyboard_input_report_report_reference =
{
    (void *)attr_uuid_type_report_reference,
    (void *)hids_keyboard_input_report_reference,
    sizeof(hids_keyboard_input_report_reference),
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    ble_svcs_hids_handler,       //registered callback function
};

const ble_att_param_t att_hids_characteristic_keyboard_output_report =
{
    (void *)attr_uuid_type_characteristic,
    (void *)attr_uuid_hids_charc_keyboard_output_report,
    sizeof(attr_uuid_hids_charc_keyboard_output_report),
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    attr_null_access,                       //This function should be set to attr_null_access when att_len or p_uuid_value is a null value.
};

const ble_att_param_t att_hids_keyboard_output_report =
{
    (void *)attr_uuid_hids_charc_keyboard_output_report,
    (void *)0,
    0,
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    ble_svcs_hids_handler,       //registered callback function
};

const ble_att_param_t att_hids_keyboard_output_report_report_reference =
{
    (void *)attr_uuid_type_report_reference,
    (void *)hids_keyboard_output_report_reference,
    sizeof(hids_keyboard_output_report_reference),
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    ble_svcs_hids_handler,       //registered callback function
};

/**************************************************************************
 * BLE Service << HIDS >> Local Variable
 **************************************************************************/
#ifndef MAX_NUM_CONN_HIDS
// check MAX_NUM_CONN_HIDS if defined or set to default 1.
#define MAX_NUM_CONN_HIDS       1
#endif


// Service basic information
ble_svcs_common_info_t           hids_basic_info[MAX_NUM_CONN_HIDS];

// HIDS information
ble_svcs_hids_info_t             *hids_info[MAX_NUM_CONN_HIDS];

// HIDS callback function
ble_svcs_evt_hids_handler_t      hids_callback[MAX_NUM_CONN_HIDS];

// HIDS registered total count
uint8_t                          hids_count = 0;


/**************************************************************************
 * BLE Service << HIDS >> Public Function
 **************************************************************************/
/** HIDS Initialization
*/
ble_err_t ble_svcs_hids_init(uint8_t host_id, ble_gatt_role_t role, ble_svcs_hids_info_t *p_info, ble_svcs_evt_hids_handler_t callback)
{
    ble_err_t status;
    uint8_t config_index;

    if (p_info == NULL)
    {
        return BLE_ERR_INVALID_PARAMETER;
    }

    // init service client basic information and get "config_index" & "hids_count"
    status = ble_svcs_common_init(host_id, role, MAX_NUM_CONN_HIDS, hids_basic_info, &config_index, &hids_count);
    if (status != BLE_ERR_OK)
    {
        return status;
    }

    // Set service role
    p_info->role = role;

    // Set HIDS data
    hids_info[config_index] = p_info;

    // Register HIDS callback function
    hids_callback[config_index] = callback;

    // Get handles at initialization if role is set to BLE_GATT_ROLE_SERVER
    if ((role & BLE_GATT_ROLE_SERVER) != 0)
    {
        status = ble_svcs_hids_handles_get(host_id, BLE_GATT_ROLE_SERVER, hids_info[config_index]);
        if (status != BLE_ERR_OK)
        {
            return status;
        }
    }

    return BLE_ERR_OK;
}


/** Get HIDS Handle Numbers
*/
ble_err_t ble_svcs_hids_handles_get(uint8_t host_id, ble_gatt_role_t role, ble_svcs_hids_info_t *p_info)
{
    ble_err_t status;
    ble_gatt_handle_table_param_t ble_gatt_handle_table_param;

    status = BLE_ERR_OK;
    do
    {
        ble_gatt_handle_table_param.host_id = host_id;
        ble_gatt_handle_table_param.gatt_role = p_info->role;
        ble_gatt_handle_table_param.p_element = (ble_att_param_t *)&att_hids_primary_service;

        if (role == BLE_GATT_ROLE_SERVER)
        {
            ble_gatt_handle_table_param.p_handle_num_addr = (void *)&p_info->server_info.handles;
        }
        else if (role == BLE_GATT_ROLE_CLIENT)
        {
            ble_gatt_handle_table_param.p_handle_num_addr = (void *)&p_info->client_info.handles;
        }
        else
        {
            printf("Error role setting.\n");
            status = BLE_ERR_INVALID_PARAMETER;
            break;
        }
        status = ble_svcs_handles_mapping_get(&ble_gatt_handle_table_param);
    } while (0);

    return status;
}

/**************************************************************************
 * BLE Service << HIDS >> General Callback Function
 **************************************************************************/
// post the event to the callback function
static void hids_evt_post(ble_evt_att_param_t *p_param, ble_svcs_evt_hids_handler_t *p_callback)
{
    // check callback is null or not
    if (*p_callback == NULL)
    {
        return;
    }
    // post event to user
    (*p_callback)(p_param);
}


// handle HIDS client GATT event
static void handle_hids_client(uint8_t index, ble_evt_att_param_t *p_param)
{
    switch (p_param->opcode)
    {
    case OPCODE_ATT_READ_RESPONSE:
        if (p_param->handle_num == hids_info[index]->client_info.handles.hdl_information)
        {
            // received read response from server
            p_param->event = BLESERVICE_HIDS_INFORMATION_READ_RSP_EVENT;
            hids_evt_post(p_param, &hids_callback[index]);
        }
        else if (p_param->handle_num == hids_info[index]->client_info.handles.hdl_protocol_mode)
        {
            // received read response from server
            p_param->event = BLESERVICE_HIDS_PROTOCOL_MODE_READ_RSP_EVENT;
            hids_evt_post(p_param, &hids_callback[index]);
        }
        else if (p_param->handle_num == hids_info[index]->client_info.handles.hdl_report_map)
        {
            // received read response from server
            p_param->event = BLESERVICE_HIDS_REPORT_MAP_READ_RSP_EVENT;
            hids_evt_post(p_param, &hids_callback[index]);
        }
        else if (p_param->handle_num == hids_info[index]->client_info.handles.hdl_boot_keyboard_input_report)
        {
            // received read response from server
            p_param->event = BLESERVICE_HIDS_BOOT_KEYBOARD_INPUT_REPORT_READ_RSP_EVENT;
            hids_evt_post(p_param, &hids_callback[index]);
        }
        else if (p_param->handle_num == hids_info[index]->client_info.handles.hdl_boot_keyboard_input_report_cccd)
        {
            // received cccd read response from server
            p_param->event = BLESERVICE_HIDS_BOOT_KEYBOARD_INPUT_REPORT_CCCD_READ_RSP_EVENT;
            hids_evt_post(p_param, &hids_callback[index]);
        }
        else if (p_param->handle_num == hids_info[index]->client_info.handles.hdl_boot_keyboard_output_report)
        {
            // received read response from server
            p_param->event = BLESERVICE_HIDS_BOOT_KEYBOARD_OUTPUT_REPORT_READ_RSP_EVENT;
            hids_evt_post(p_param, &hids_callback[index]);
        }
        else if (p_param->handle_num == hids_info[index]->client_info.handles.hdl_keyboard_input_report)
        {
            // received read response from server
            p_param->event = BLESERVICE_HIDS_KEYBOARD_INPUT_REPORT_READ_RSP_EVENT;
            hids_evt_post(p_param, &hids_callback[index]);
        }
        else if (p_param->handle_num == hids_info[index]->client_info.handles.hdl_keyboard_input_report_cccd)
        {
            // received cccd read response from server
            p_param->event = BLESERVICE_HIDS_KEYBOARD_INPUT_REPORT_CCCD_READ_RSP_EVENT;
            hids_evt_post(p_param, &hids_callback[index]);
        }
        else if (p_param->handle_num == hids_info[index]->client_info.handles.hdl_keyboard_input_report_report_reference)
        {
            // received report reference read response from server
            p_param->event = BLESERVICE_HIDS_KEYBOARD_INPUT_REPORT_REPORT_READ_RSP_EVENT;
            hids_evt_post(p_param, &hids_callback[index]);
        }
        else if (p_param->handle_num == hids_info[index]->client_info.handles.hdl_keyboard_output_report)
        {
            // received read response from server
            p_param->event = BLESERVICE_HIDS_KEYBOARD_OUTPUT_REPORT_READ_RSP_EVENT;
            hids_evt_post(p_param, &hids_callback[index]);
        }
        else if (p_param->handle_num == hids_info[index]->client_info.handles.hdl_keyboard_output_report_report_reference)
        {
            // received report reference read response from server
            p_param->event = BLESERVICE_HIDS_KEYBOARD_OUTPUT_REPORT_REPORT_READ_RSP_EVENT;
            hids_evt_post(p_param, &hids_callback[index]);
        }
        
        break;
    case OPCODE_ATT_WRITE_RESPONSE:
    case OPCODE_ATT_RESTORE_BOND_DATA_COMMAND:
        if (p_param->handle_num == hids_info[index]->client_info.handles.hdl_boot_keyboard_input_report)
        {
            // received write response from server
            p_param->event = BLESERVICE_HIDS_BOOT_KEYBOARD_INPUT_REPORT_WRITE_RSP_EVENT;
            hids_evt_post(p_param, &hids_callback[index]);
        }
        else if (p_param->handle_num == hids_info[index]->client_info.handles.hdl_boot_keyboard_input_report_cccd)
        {
            // received cccd write response from server
            p_param->event = BLESERVICE_HIDS_BOOT_KEYBOARD_INPUT_REPORT_CCCD_WRITE_RSP_EVENT;
            hids_evt_post(p_param, &hids_callback[index]);
        }
        else if (p_param->handle_num == hids_info[index]->client_info.handles.hdl_boot_keyboard_output_report)
        {
            // received write response from server
            p_param->event = BLESERVICE_HIDS_BOOT_KEYBOARD_OUTPUT_REPORT_WRITE_RSP_EVENT;
            hids_evt_post(p_param, &hids_callback[index]);
        }
        else if (p_param->handle_num == hids_info[index]->client_info.handles.hdl_keyboard_input_report)
        {
            // received write response from server
            p_param->event = BLESERVICE_HIDS_KEYBOARD_INPUT_REPORT_WRITE_RSP_EVENT;
            hids_evt_post(p_param, &hids_callback[index]);
        }
        else if (p_param->handle_num == hids_info[index]->client_info.handles.hdl_keyboard_input_report_cccd)
        {
            // received cccd write response from server
            p_param->event = BLESERVICE_HIDS_KEYBOARD_INPUT_REPORT_CCCD_WRITE_RSP_EVENT;
            hids_evt_post(p_param, &hids_callback[index]);
        }
        else if (p_param->handle_num == hids_info[index]->client_info.handles.hdl_keyboard_output_report)
        {
            // received write response from server
            p_param->event = BLESERVICE_HIDS_KEYBOARD_OUTPUT_REPORT_WRITE_RSP_EVENT;
            hids_evt_post(p_param, &hids_callback[index]);
        }
        break;

    case OPCODE_ATT_HANDLE_VALUE_NOTIFICATION:
        if (p_param->handle_num == hids_info[index]->client_info.handles.hdl_boot_keyboard_input_report)
        {
            // received notify from server
            p_param->event = BLESERVICE_HIDS_BOOT_KEYBOARD_INPUT_REPORT_NOTIFY_EVENT;
            hids_evt_post(p_param, &hids_callback[index]);
        }
        else if (p_param->handle_num == hids_info[index]->client_info.handles.hdl_keyboard_input_report)
        {
            // received notify from server
            p_param->event = BLESERVICE_HIDS_KEYBOARD_INPUT_REPORT_NOTIFY_EVENT;
            hids_evt_post(p_param, &hids_callback[index]);
        }
        
        break;
    case OPCODE_ATT_HANDLE_VALUE_INDICATION:
        break;

    default:
        break;
    }
}


// handle HIDS server GATT event
static void handle_hids_server(uint8_t index, ble_evt_att_param_t *p_param)
{
    switch (p_param->opcode)
    {
    case OPCODE_ATT_READ_REQUEST:
    case OPCODE_ATT_READ_BY_TYPE_REQUEST:
        if (p_param->handle_num == hids_info[index]->server_info.handles.hdl_information)
        {
            // received read from client
            ble_svcs_auto_handle_read_req(p_param, (uint8_t *)hids_information, sizeof(hids_information));
        }
        else if (p_param->handle_num == hids_info[index]->server_info.handles.hdl_protocol_mode)
        {
            // received read from client
            ble_svcs_auto_handle_read_req(p_param, (uint8_t *)hids_protocol_mode, sizeof(hids_protocol_mode));
        }
        else if (p_param->handle_num == hids_info[index]->server_info.handles.hdl_report_map)
        {
            // received read from client
            ble_svcs_auto_handle_read_req(p_param, (uint8_t *)hids_report_map, sizeof(hids_report_map));
        }
        else if (p_param->handle_num == hids_info[index]->server_info.handles.hdl_boot_keyboard_input_report)
        {
            // received read from client
            p_param->event = BLESERVICE_HIDS_BOOT_KEYBOARD_INPUT_REPORT_READ_EVENT;
            hids_evt_post(p_param, &hids_callback[index]);
        }
        else if (p_param->handle_num == hids_info[index]->server_info.handles.hdl_boot_keyboard_input_report_cccd)
        {
            // received cccd read from client
            ble_svcs_auto_handle_cccd_read_req(p_param, hids_info[index]->server_info.data.boot_keyboard_input_report_cccd);
        }
        else if (p_param->handle_num == hids_info[index]->server_info.handles.hdl_boot_keyboard_output_report)
        {
            // received read from client
            p_param->event = BLESERVICE_HIDS_BOOT_KEYBOARD_OUTPUT_REPORT_READ_EVENT;
            hids_evt_post(p_param, &hids_callback[index]);
        }
        else if (p_param->handle_num == hids_info[index]->server_info.handles.hdl_keyboard_input_report)
        {
            // received read from client
            p_param->event = BLESERVICE_HIDS_KEYBOARD_INPUT_REPORT_READ_EVENT;
            hids_evt_post(p_param, &hids_callback[index]);
        }
        else if (p_param->handle_num == hids_info[index]->server_info.handles.hdl_keyboard_input_report_cccd)
        {
            // received cccd read from client
            ble_svcs_auto_handle_cccd_read_req(p_param, hids_info[index]->server_info.data.keyboard_input_report_cccd);
        }
        else if (p_param->handle_num == hids_info[index]->server_info.handles.hdl_keyboard_output_report)
        {
            // received read from client
            p_param->event = BLESERVICE_HIDS_KEYBOARD_OUTPUT_REPORT_READ_EVENT;
            hids_evt_post(p_param, &hids_callback[index]);
        }
        break;

    case OPCODE_ATT_READ_BLOB_REQUEST:
        if (p_param->handle_num == hids_info[index]->server_info.handles.hdl_report_map)
        {
            // received read from client
            ble_svcs_auto_handle_read_blob_req(p_param, (uint8_t *)hids_report_map, sizeof(hids_report_map));
        }
        break;

    case OPCODE_ATT_WRITE_REQUEST:
    case OPCODE_ATT_RESTORE_BOND_DATA_COMMAND:
        if (p_param->handle_num == hids_info[index]->server_info.handles.hdl_boot_keyboard_input_report)
        {
            // received write from client
            p_param->event = BLESERVICE_HIDS_BOOT_KEYBOARD_INPUT_REPORT_WRITE_EVENT;
            hids_evt_post(p_param, &hids_callback[index]);
        }
        else if (p_param->handle_num == hids_info[index]->server_info.handles.hdl_boot_keyboard_input_report_cccd)
        {
            // received cccd write from client
            ble_svcs_handle_cccd_write_req(p_param->data, p_param->length, &hids_info[index]->server_info.data.boot_keyboard_input_report_cccd);
            p_param->event = BLESERVICE_HIDS_BOOT_KEYBOARD_INPUT_REPORT_CCCD_WRITE_EVENT;
            hids_evt_post(p_param, &hids_callback[index]);
        }
        else if (p_param->handle_num == hids_info[index]->server_info.handles.hdl_boot_keyboard_output_report)
        {
            // received write from client
            p_param->event = BLESERVICE_HIDS_BOOT_KEYBOARD_OUTPUT_REPORT_WRITE_EVENT;
            hids_evt_post(p_param, &hids_callback[index]);
        }
        else if (p_param->handle_num == hids_info[index]->server_info.handles.hdl_keyboard_input_report)
        {
            // received write from client
            p_param->event = BLESERVICE_HIDS_KEYBOARD_INPUT_REPORT_WRITE_EVENT;
            hids_evt_post(p_param, &hids_callback[index]);
        }
        else if (p_param->handle_num == hids_info[index]->server_info.handles.hdl_keyboard_input_report_cccd)
        {
            // received cccd write from client
            ble_svcs_handle_cccd_write_req(p_param->data, p_param->length, &hids_info[index]->server_info.data.keyboard_input_report_cccd);
            p_param->event = BLESERVICE_HIDS_KEYBOARD_INPUT_REPORT_CCCD_WRITE_EVENT;
            hids_evt_post(p_param, &hids_callback[index]);
        }
        else if (p_param->handle_num == hids_info[index]->server_info.handles.hdl_keyboard_output_report)
        {
            // received write from client
            p_param->event = BLESERVICE_HIDS_KEYBOARD_OUTPUT_REPORT_WRITE_EVENT;
            hids_evt_post(p_param, &hids_callback[index]);
        }
        break;

    case OPCODE_ATT_WRITE_COMMAND:
        if (p_param->handle_num == hids_info[index]->server_info.handles.hdl_control_point)
        {
            // received write without response from client
            p_param->event = BLESERVICE_HIDS_CONTROL_POINT_WRITE_WITHOUT_RSP_EVENT;
            hids_evt_post(p_param, &hids_callback[index]);
        }
        else if (p_param->handle_num == hids_info[index]->server_info.handles.hdl_protocol_mode)
        {
            // received write without response from client
            p_param->event = BLESERVICE_HIDS_PROTOCOL_MODE_WRITE_WITHOUT_RSP_EVENT;
            hids_evt_post(p_param, &hids_callback[index]);
        }
        else if (p_param->handle_num == hids_info[index]->server_info.handles.hdl_boot_keyboard_output_report)
        {
            // received write without response from client
            p_param->event = BLESERVICE_HIDS_BOOT_KEYBOARD_OUTPUT_REPORT_WRITE_WITHOUT_RSP_EVENT;
            hids_evt_post(p_param, &hids_callback[index]);
        }
        else if (p_param->handle_num == hids_info[index]->server_info.handles.hdl_keyboard_output_report)
        {
            // received write without response from client
            p_param->event = BLESERVICE_HIDS_KEYBOARD_OUTPUT_REPORT_WRITE_WITHOUT_RSP_EVENT;
            hids_evt_post(p_param, &hids_callback[index]);
        }
        break;

    case OPCODE_ATT_HANDLE_VALUE_CONFIRMATION:
        break;

    default:
        break;
    }
}



/** ble_svcs_hids_handler
 * @note This callback receives the HIDS events.
 * Each of these events can be associated with parameters.
*/

void ble_svcs_hids_handler(ble_evt_att_param_t *p_param)
{
    uint8_t index;

    if (ble_svcs_common_info_index_query(p_param->host_id, p_param->gatt_role, MAX_NUM_CONN_HIDS, hids_basic_info, &index) != BLE_ERR_OK)
    {
        // Host id has not registered so there is no callback function -> do nothing
        return;
    }

    if (p_param->gatt_role == BLE_GATT_ROLE_CLIENT)
    {
        // handle HIDS client GATT event
        handle_hids_client(index, p_param);
    }

    if (p_param->gatt_role == BLE_GATT_ROLE_SERVER)
    {
        // handle HIDS server GATT event
        handle_hids_server(index, p_param);
    }
}
