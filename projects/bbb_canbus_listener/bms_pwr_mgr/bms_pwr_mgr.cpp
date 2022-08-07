#include <algorithm>
#include <iostream>
#include <array>

#include <stdint.h>
#include <assert.h>
#include <linux/can.h>

#include "uccm-sensors/frame_parser.h"

///// PRIVATE CONSTANTS AND DATATYPES /////

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

typedef enum : uint8_t {
    regular_operation,
    critical
} battery_state_e;

typedef enum : uint8_t {
    port = 0,
    stbd = 1
} port_stbd_e;

constexpr std::array<std::uint8_t, 3> port_bms             = {BMS1_FRAME1_ID, BMS2_FRAME1_ID, BMS3_FRAME1_ID};
constexpr std::array<std::uint8_t, 3> stbd_bms             = {BMS4_FRAME1_ID, BMS5_FRAME1_ID, BMS6_FRAME1_ID};
constexpr std::array<std::array<std::uint8_t, 3>, 2> bmses = {port_bms, stbd_bms};

constexpr uint64_t can_port_discharge_stbd_charge_cmd = 0x00000000003F0738;
constexpr uint64_t can_port_charge_stbd_discharge_cmd = 0x00000000003F3807;


///// PRIVATE GLOBAL VARIABLES /////

namespace {
battery_state_e curr_state       = regular_operation;
float           curr_threshold   = default_threshold;
port_stbd_e     curr_charge_side = port; // TODO: verify if OK to set port as default charging side
float           curr_port_volt   = max_voltage;
float           curr_stbd_volt   = max_voltage;
}

///// FUNCTIONS /////

static inline void SwapChargeDischargeFrame(struct can_frame &frame) {
    uint64_t cmd = curr_charge_side == port ? can_port_discharge_stbd_charge_cmd : can_port_charge_stbd_discharge_cmd;
    for (int i = 0; i < CAN_MAX_DLEN; i++) {
        frame.data[i] = (cmd >> (8 * i)) & 0xFF;
    }
}

static inline void GetCurrentStatus(const uint8_t &bms_frame_id, 
    float *curr_voltage, float &opposite_bms_side_volt, port_stbd_e &this_bms_side) {
    if (std::any_of(std::begin(port_bms), std::end(port_bms),
        [bms_frame_id](uint8_t i) { return i == bms_frame_id; })) { // If frame ID is in port
        curr_voltage = &curr_port_volt;
        opposite_bms_side_volt = curr_stbd_volt;
        this_bms_side = port;
    } else { // ID is in stbd
        curr_voltage = &curr_stbd_volt;
        opposite_bms_side_volt = curr_port_volt;
        this_bms_side = stbd;
    }

}

void OnNewVoltageReading(const uint8_t &bms_frame_id, const float &voltage, struct can_frame &frame) {
    float *curr_voltage = nullptr;
    float opposite_bms_side_volt;
    port_stbd_e this_bms_side;
    GetCurrentStatus(bms_frame_id, curr_voltage, opposite_bms_side_volt, this_bms_side);
    assert(curr_voltage == &curr_port_volt || curr_voltage == &curr_stbd_volt);
    *curr_voltage = voltage;
    switch (curr_state) {
        case regular_operation: {
            if (curr_charge_side == this_bms_side) { // Given voltage of side that is currently charging
                if (voltage >= (curr_threshold + surplus_to_increase_threshold)) {
                    if (curr_threshold < max_threshold) {
                        curr_threshold += increase_threshold_amount;
                        // TODO: Ask ELEC if better to swap or not swap in this scenario
                        SwapChargeDischargeFrame(frame); // Since this side has a higher charge, swap
                    }
                }
            } else { // Given voltage of side that is currently discharging
                if (voltage <= curr_threshold) {
                    SwapChargeDischargeFrame(frame);
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
                    SwapChargeDischargeFrame(frame);
                    curr_state = regular_operation;
                }
            } else {
                if (voltage <= life_support_threshold) {
                    SwapChargeDischargeFrame(frame);
                }
            }
            break;
        }
    }
}
