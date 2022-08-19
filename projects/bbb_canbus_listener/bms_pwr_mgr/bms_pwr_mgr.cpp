/**
*
*  Copyright 2022 UBC Sailbot
*
*  @file  bms_pwr_mgr.cpp
*  @brief Manages the battery load distribution
*
*  Tracks the voltage level Raye's port and
*  starboard batteries and commands them to
*  charge or discharge for better load distribution
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
#include <thread>
#include <mutex>
#include <chrono>

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
port_stbd_e     curr_charge_side = port;
float           curr_port_volt   = max_voltage;
float           curr_stbd_volt   = max_voltage;

struct can_frame write_frame = {
    .can_id  = BMS_CMD_FRAME_ID,
    .can_dlc = CAN_DLC,
};
std::thread write_cmd_thread;
std::mutex write_frame_mtx;
bool terminate_flag = false;
}  // namespace

///// FUNCTIONS /////

static inline void swapChargeDischargeFrame(void) {
    uint64_t cmd = curr_charge_side == port ? can_port_discharge_stbd_charge_cmd : can_port_charge_stbd_discharge_cmd;
    write_frame_mtx.lock();
    for (int i = 0; i < CAN_MAX_DLEN; i++) {
        write_frame.data[i] = (cmd >> (8 * i)) & 0xFF;
    }
    write_frame_mtx.unlock();
    curr_charge_side = static_cast<port_stbd_e>((~curr_charge_side) & 1);
}

static inline void getCurrentStatus(const uint8_t &bms_frame_id,
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

void write_cmd(int socket) {
    while (!terminate_flag) {
        write_frame_mtx.lock();
        write(socket, &write_frame, sizeof(struct can_frame));
        write_frame_mtx.unlock();
        std::this_thread::sleep_for(std::chrono::seconds(can_write_interval_seconds));
    }
}

void BmsPwrMgr::init(int fd) {
    // Set the data so the write thread doesn't just write nothing
    // Because we swap to the opposite of the default charge/discharge sides, we flip it first to maintain the defaults
    curr_charge_side = static_cast<port_stbd_e>((~curr_charge_side) & 1);
    swapChargeDischargeFrame();
    write_cmd_thread = std::thread(write_cmd, fd);
}

void BmsPwrMgr::terminate(void) {
    if (!terminate_flag) {
        terminate_flag = true;
        write_cmd_thread.join();
        terminate_flag = false;
    }
}

void BmsPwrMgr::onNewVoltageReading(const canid_t &bms_frame_id, const float &voltage) {
    float *curr_voltage = nullptr;
    float opposite_bms_side_volt;
    port_stbd_e this_bms_side;
    getCurrentStatus(bms_frame_id, curr_voltage, opposite_bms_side_volt, this_bms_side);
    assert(curr_voltage == &curr_port_volt || curr_voltage == &curr_stbd_volt);
    *curr_voltage = voltage;
    switch (curr_state) {
        case regular_operation: {
            if (curr_charge_side == this_bms_side) {  // Given voltage of side that is currently charging
                if (voltage >= (curr_threshold + surplus_to_increase_threshold)) {
                    if (curr_threshold < max_threshold) {
                        curr_threshold += increase_threshold_amount;
                        swapChargeDischargeFrame();  // Since this side has a higher charge, swap
                    }
                }
            } else {  // Given voltage of side that is currently discharging
                if (voltage <= curr_threshold) {
                    swapChargeDischargeFrame();
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
                    swapChargeDischargeFrame();
                    curr_state = regular_operation;
                }
            } else {
                if (voltage <= life_support_threshold) {
                    swapChargeDischargeFrame();
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
