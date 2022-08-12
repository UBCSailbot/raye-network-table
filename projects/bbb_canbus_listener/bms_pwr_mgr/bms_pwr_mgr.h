/**
*
*  Copyright 2022 UBC Sailbot
*
*  @file  bms_pwr_mgr.h
*
*  @author Henry Huang (hhenry01)
*
*/

#ifndef _BMS_PWR_MGR_H_
#define _BMS_PWR_MGR_H_

#include <cmath>
#include <stdint.h>
#include <linux/can.h>

///// CONSTANTS /////
constexpr float max_voltage                   = 18.7;
constexpr float min_voltage                   = 16.7;
constexpr float v_range                       = max_voltage - min_voltage;
constexpr float default_threshold             = 0.8 * v_range + min_voltage;
constexpr float min_threshold                 = 0.1 * v_range + min_voltage;
constexpr float max_threshold                 = 0.8 * v_range + min_voltage;
constexpr float life_support_threshold        = 0.01 * v_range + min_voltage;
constexpr float decrease_threshold_amount     = 0.1 * v_range;  // How much to decrease the threshold by
constexpr float increase_threshold_amount     = 0.1 * v_range;  // How much to increase the threshold by
constexpr float surplus_to_increase_threshold = 0.25 * v_range; // Surplus needed by the charging side to increase
constexpr float surplus_to_decrease_threshold = 0.05 * v_range; // Surplus needed by the charging side to decrease
                                                                // Decrease if <= threshold, otherwise just swap
constexpr uint64_t can_port_discharge_stbd_charge_cmd = 0x3F0738;
constexpr uint64_t can_port_charge_stbd_discharge_cmd = 0x3F3807;

///// DATATYPES /////
typedef enum : uint8_t {
    regular_operation,
    critical
} battery_state_e;

typedef enum : uint8_t {
    port = 0,
    stbd = 1
} port_stbd_e;

///// FUNCTIONS /////
namespace BmsPwrMgr {
/**
 * @brief Function that gets invoked whenever a new BMS status frame is received
 *
 * @param bms_frame_id  One of: BMS1_FRAME1_ID, BMS2_FRAME1_ID, BMS3_FRAME1_ID,
                                BMS4_FRAME1_ID, BMS5_FRAME1_ID, BMS6_FRAME1_ID
 * @param voltage       Voltage reading given as part of the BMS bms_frame_id command
 * @param frame         Reference to can_frame structure that will be written to CAN to command bms charge/discharge
 * @param socket        File descriptor to write the can frame to
 */
void onNewVoltageReading(const canid_t &bms_frame_id, const float &voltage, struct can_frame &frame, int socket);
/**
 * @brief Configure the private fields of the BMS Power Manager
 *
 * @note  Unless testing, must only be used once - if at all
 *
 * @param state         What will the state of the manager be
 * @param threshold     Voltage to set the threshold to
 * @param charge_side   Is port or stbd charging
 * @param port_volt     Update port BMS voltage
 * @param stbd_volt     Update stbd BMS voltage
 */
void configure(battery_state_e state, float threshold, port_stbd_e charge_side, float port_volt, float stbd_volt);

// Getter methods to provide read only access to global variables
battery_state_e getCurrState(void);
float           getCurrThreshold(void);
port_stbd_e     getCurrChargeSide(void);
float           getCurrPortVolt(void);
float           getCurrStbdVolt(void);
}  // namespace BmsMgr

#endif // _BMS_PWR_MGR_H_
