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

#include <cmath>
#include <cstdint>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <linux/can.h>

#include "bms_pwr_mgr.h"  // NOLINT(build/include)
#include "uccm-sensors/frame_parser.h"

constexpr int num_tests = 9;

int test_regular_operation_no_action(void) {
    BmsPwrMgr::configure(regular_operation, default_threshold, port, max_voltage, max_voltage);
    int mock_socket = open("/tmp", O_TMPFILE | O_RDWR);
    if (mock_socket == -1) {
        std::cerr << "Failed to create mock socket" << std::endl;
        return 0;
    }

    battery_state_e before_state  = BmsPwrMgr::getCurrState();
    float           before_thresh = BmsPwrMgr::getCurrThreshold();
    port_stbd_e     before_c_side = BmsPwrMgr::getCurrChargeSide();

    struct can_frame frame;
    frame.can_dlc = 8;
    frame.can_id  = BMS_CMD_FRAME_ID;

    float voltage = before_thresh + 0.1;
    BmsPwrMgr::onNewVoltageReading(BMS1_FRAME1_ID, voltage, frame, mock_socket);

    if ((BmsPwrMgr::getCurrState() != before_state) || (BmsPwrMgr::getCurrThreshold() != before_thresh) ||
         BmsPwrMgr::getCurrChargeSide() != before_c_side) {
        std::cerr << "BMS fields changed when they should not have!" << std::endl;
        return 0;
    }
    uint64_t buf;
    if (read(mock_socket, &buf, sizeof(uint64_t)) != 0) {
        std::cerr << "CAN CMD was written when it should not have!" << std::endl;
        return 0;
    }

    return 1;
}

int test_charge_side_at_max_threshold(void) {
    BmsPwrMgr::configure(regular_operation, max_threshold, port, max_voltage, max_voltage);
    int mock_socket = open("/tmp", O_TMPFILE | O_RDWR);
    if (mock_socket == -1) {
        std::cerr << "Failed to create mock socket" << std::endl;
        return 0;
    }
    float           before_thresh = BmsPwrMgr::getCurrThreshold();
    battery_state_e before_state  = BmsPwrMgr::getCurrState();
    port_stbd_e     before_c_side = BmsPwrMgr::getCurrChargeSide();

    struct can_frame frame;
    frame.can_dlc = 8;
    frame.can_id = BMS_CMD_FRAME_ID;

    canid_t bms_id = (before_c_side == port) ? BMS1_FRAME1_ID : BMS4_FRAME1_ID;

    BmsPwrMgr::onNewVoltageReading(bms_id, max_voltage, frame, mock_socket);

    if ((BmsPwrMgr::getCurrState() != before_state) || (BmsPwrMgr::getCurrThreshold() != before_thresh) ||
         BmsPwrMgr::getCurrChargeSide() != before_c_side) {
        std::cerr << "BMS fields changed when they should not have!" << std::endl;
        return 0;
    }
    uint64_t buf;
    if (read(mock_socket, &buf, sizeof(uint64_t)) != 0) {
        std::cerr << "CAN CMD was written when it should not have!" << std::endl;
        return 0;
    }

    return 1;
}

int test_discharge_reaches_threshold_do_not_lower() {
    BmsPwrMgr::configure(regular_operation, max_threshold, port, max_voltage, max_voltage);
    int mock_socket = open("/tmp", O_TMPFILE | O_RDWR);
    if (mock_socket == -1) {
        std::cerr << "Failed to create mock socket" << std::endl;
        return 0;
    }
    float           before_thresh = BmsPwrMgr::getCurrThreshold();
    battery_state_e before_state  = BmsPwrMgr::getCurrState();
    port_stbd_e     before_c_side = BmsPwrMgr::getCurrChargeSide();

    struct can_frame frame;
    frame.can_dlc = 8;
    frame.can_id = BMS_CMD_FRAME_ID;

    canid_t discharge_bms_id = (before_c_side == port) ? BMS4_FRAME1_ID : BMS1_FRAME1_ID;

    BmsPwrMgr::onNewVoltageReading(discharge_bms_id, max_threshold, frame, mock_socket);
    if ((BmsPwrMgr::getCurrState() != before_state) || (BmsPwrMgr::getCurrThreshold() != before_thresh)) {
        std::cerr << "BMS fields changed when they should not have!" << std::endl;
        return 0;
    }
    if (BmsPwrMgr::getCurrChargeSide() != ((~before_c_side) & 1)) {
        std::cerr << "BMS charge side did not swap!" << std::endl;
        return 0;
    }
    uint64_t expected_cmd;
    expected_cmd = (before_c_side == port) ? can_port_discharge_stbd_charge_cmd : can_port_charge_stbd_discharge_cmd;
    lseek(mock_socket, 0, SEEK_SET);
    if (read(mock_socket, &frame, sizeof(struct can_frame)) == 0) {
        std::cerr << "CAN CMD was not written!" << std::endl;
        return 0;
    }
    for (uint8_t i = 0; i < CAN_MAX_DLEN; i++) {
        if (frame.data[i] != ((expected_cmd >> (8 * i)) & 0xFF)) {
            std::cerr << "Unexpected CAN command written!" << std::endl;
            return 0;
        }
    }
    return 1;
}

int main(void) {
    int success_count = 0;
    // Insert test functions
    std::cout << "~~~ Test 1 ~~~" << std::endl;
    success_count += test_regular_operation_no_action();
    std::cout << "~~~ Test 2 ~~~" << std::endl;
    success_count += test_charge_side_at_max_threshold();
    std::cout << "~~~ Test 3 ~~~" << std::endl;
    success_count += test_discharge_reaches_threshold_do_not_lower();
    std::cout << success_count << " tests passed out of: " << num_tests << " total" << std::endl;
    return 0;
}
