/**
*
*  Copyright 2022 UBC Sailbot
*
*  @file  bbb_canbus_listener.cpp
*  @brief Facilitates communication between CANbus and Network Table
*
*  Polls sensor data from the CANbus and publishes 
*  it to the network table. Writes actuation angles
*  outputted from the boat controller to the CANbus
*
*  @author Henry Huang (hhenry01)
*
*/

#include <stdint.h>
#include <assert.h>
#include <linux/can.h>
#include <unistd.h>

#include <algorithm>
#include <iostream>
#include <array>

#include "bms_pwr_mgr.h"  // NOLINT(build/include)
#include "uccm-sensors/frame_parser.h"

///// PRIVATE CONSTANTS AND DATATYPES /////

constexpr std::array<std::uint8_t, 3> port_bms             = {BMS1_FRAME1_ID, BMS2_FRAME1_ID, BMS3_FRAME1_ID};
constexpr std::array<std::uint8_t, 3> stbd_bms             = {BMS4_FRAME1_ID, BMS5_FRAME1_ID, BMS6_FRAME1_ID};
constexpr std::array<std::array<std::uint8_t, 3>, 2> bmses = {port_bms, stbd_bms};

///// PRIVATE GLOBAL VARIABLES /////

namespace {
battery_state_e curr_state       = regular_operation;
float           curr_threshold   = default_threshold;
port_stbd_e     curr_charge_side = port;  // TODO(hhenry01): verify if OK to set port as default charging side
float           curr_port_volt   = max_voltage;
float           curr_stbd_volt   = max_voltage;
}

///// FUNCTIONS /////

static inline void SwapChargeDischargeFrame(struct can_frame &frame, const int socket) {  // NOLINT(runtime/references)
    uint64_t cmd = curr_charge_side == port ? can_port_discharge_stbd_charge_cmd : can_port_charge_stbd_discharge_cmd;
    for (int i = 0; i < CAN_MAX_DLEN; i++) {
        frame.data[i] = (cmd >> (8 * i)) & 0xFF;
    }
    curr_charge_side = static_cast<port_stbd_e>((~curr_charge_side) & 1);
    write(socket, &frame, sizeof(struct can_frame));
}

static inline void GetCurrentStatus(const uint8_t &bms_frame_id,
    float *&curr_voltage, float &opposite_bms_side_volt, port_stbd_e &this_bms_side) {  // NOLINT(runtime/references)
    if (std::any_of(std::begin(port_bms), std::end(port_bms),
        [bms_frame_id](uint8_t i) { return i == bms_frame_id; })) {  // If frame ID is in port
        curr_voltage = &curr_port_volt;
        opposite_bms_side_volt = curr_stbd_volt;
        this_bms_side = port;
    } else {  // ID is in stbd
        curr_voltage = &curr_stbd_volt;
        opposite_bms_side_volt = curr_port_volt;
        this_bms_side = stbd;
    }
}

void BmsPwrMgr::onNewVoltageReading(const canid_t &bms_frame_id, const float &voltage,
                         struct can_frame &frame, const int socket) {  // NOLINT(runtime/references)
    float *curr_voltage = nullptr;
    float opposite_bms_side_volt;
    port_stbd_e this_bms_side;
    GetCurrentStatus(bms_frame_id, curr_voltage, opposite_bms_side_volt, this_bms_side);
    assert(curr_voltage == &curr_port_volt || curr_voltage == &curr_stbd_volt);
    *curr_voltage = voltage;
    switch (curr_state) {
        case regular_operation: {
            if (curr_charge_side == this_bms_side) {  // Given voltage of side that is currently charging
                if (voltage >= (curr_threshold + surplus_to_increase_threshold)) {
                    if (curr_threshold < max_threshold) {
                        curr_threshold += increase_threshold_amount;
                        // TODO(hhenry01): Ask ELEC if better to swap or not swap in this scenario
                        SwapChargeDischargeFrame(frame, socket);  // Since this side has a higher charge, swap
                    }
                }
            } else {  // Given voltage of side that is currently discharging
                if (voltage <= curr_threshold) {
                    SwapChargeDischargeFrame(frame, socket);
                    if (opposite_bms_side_volt <= curr_threshold + surplus_to_decrease_threshold) {
                        if (curr_threshold > min_threshold) {
                            curr_threshold -= decrease_threshold_amount;
                        } else {
                            curr_state = critical;
                        }
                    }
                }
            }
            break;
        }
        case critical: {
            if (curr_charge_side == this_bms_side) {
                if (voltage >= (curr_threshold + surplus_to_increase_threshold)) {
                    SwapChargeDischargeFrame(frame, socket);
                    curr_state = regular_operation;
                }
            } else {
                if (voltage <= life_support_threshold) {
                    SwapChargeDischargeFrame(frame, socket);
                }
            }
            break;
        }
    }
}

void BmsPwrMgr::configure(battery_state_e state, float threshold, port_stbd_e charge_side, float port_volt,
                          float stbd_volt) {
    curr_state       = state;
    curr_threshold   = threshold;
    curr_charge_side = charge_side;
    curr_port_volt   = port_volt;
    curr_stbd_volt   = stbd_volt;
}

battery_state_e BmsPwrMgr::getCurrState(void) {
    return curr_state;
}

float BmsPwrMgr::getCurrThreshold(void) {
    return curr_threshold;
}

port_stbd_e BmsPwrMgr::getCurrChargeSide(void) {
    return curr_charge_side;
}

float BmsPwrMgr::getCurrPortVolt(void) {
    return curr_port_volt;
}

float BmsPwrMgr::getCurrStbdVolt(void) {
    return curr_stbd_volt;
}
