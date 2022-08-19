/**
*
*  Copyright 2022 UBC Sailbot
*
*  @file  test_bms_pwr_mgr.cpp
*  @note  Tests where the expected result is "command not written" have a race condition
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
#include <thread>
#include <chrono>

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
    BmsPwrMgr::init(mock_socket);

    battery_state_e before_state  = BmsPwrMgr::getCurrState();
    float           before_thresh = BmsPwrMgr::getCurrThreshold();
    port_stbd_e     before_c_side = BmsPwrMgr::getCurrChargeSide();

    struct can_frame frame;
    float voltage = before_thresh + 0.1;
    BmsPwrMgr::onNewVoltageReading(BMS1_FRAME1_ID, voltage);

    if ((BmsPwrMgr::getCurrState() != before_state) || (BmsPwrMgr::getCurrThreshold() != before_thresh) ||
         BmsPwrMgr::getCurrChargeSide() != before_c_side) {
        std::cerr << "BMS fields changed when they should not have!" << std::endl;
        return 0;
    }
    if (read(mock_socket, &frame, sizeof(uint64_t)) != 0) {
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
    BmsPwrMgr::init(mock_socket);
    float           before_thresh = BmsPwrMgr::getCurrThreshold();
    battery_state_e before_state  = BmsPwrMgr::getCurrState();
    port_stbd_e     before_c_side = BmsPwrMgr::getCurrChargeSide();

    struct can_frame frame;
    canid_t bms_id = (before_c_side == port) ? BMS1_FRAME1_ID : BMS4_FRAME1_ID;

    BmsPwrMgr::onNewVoltageReading(bms_id, max_voltage);

    if ((BmsPwrMgr::getCurrState() != before_state) || (BmsPwrMgr::getCurrThreshold() != before_thresh) ||
         BmsPwrMgr::getCurrChargeSide() != before_c_side) {
        std::cerr << "BMS fields changed when they should not have!" << std::endl;
        return 0;
    }
    if (read(mock_socket, &frame, sizeof(uint64_t)) != 0) {
        std::cerr << "CAN CMD was written when it should not have!" << std::endl;
        return 0;
    }

    return 1;
}

int test_discharge_reaches_threshold_do_not_lower(void) {
    BmsPwrMgr::configure(regular_operation, max_threshold, port, max_voltage, max_voltage);
    int mock_socket = open("/tmp", O_TMPFILE | O_RDWR);
    if (mock_socket == -1) {
        std::cerr << "Failed to create mock socket" << std::endl;
        return 0;
    }
    BmsPwrMgr::init(mock_socket);
    float           before_thresh = BmsPwrMgr::getCurrThreshold();
    battery_state_e before_state  = BmsPwrMgr::getCurrState();
    port_stbd_e     before_c_side = BmsPwrMgr::getCurrChargeSide();

    struct can_frame frame;
    canid_t discharge_bms_id = (before_c_side == port) ? BMS4_FRAME1_ID : BMS1_FRAME1_ID;

    BmsPwrMgr::onNewVoltageReading(discharge_bms_id, max_threshold);
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
    std::this_thread::sleep_for(std::chrono::seconds(2 * can_write_interval_seconds));
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

int test_discharge_reaches_threshold_lower(void) {
    BmsPwrMgr::configure(regular_operation, max_threshold, port, max_threshold + surplus_to_decrease_threshold,
                         max_threshold + surplus_to_decrease_threshold);
    int mock_socket = open("/tmp", O_TMPFILE | O_RDWR);
    if (mock_socket == -1) {
        std::cerr << "Failed to create mock socket" << std::endl;
        return 0;
    }
    BmsPwrMgr::init(mock_socket);
    float           before_thresh = BmsPwrMgr::getCurrThreshold();
    battery_state_e before_state  = BmsPwrMgr::getCurrState();
    port_stbd_e     before_c_side = BmsPwrMgr::getCurrChargeSide();

    struct can_frame frame;
    canid_t discharge_bms_id = (before_c_side == port) ? BMS4_FRAME1_ID : BMS1_FRAME1_ID;

    BmsPwrMgr::onNewVoltageReading(discharge_bms_id, before_thresh);
    if (BmsPwrMgr::getCurrState() != before_state) {
        std::cerr << "BMS state changed when it should not have!" << std::endl;
        return 0;
    }
    if (BmsPwrMgr::getCurrChargeSide() != ((~before_c_side) & 1)) {
        std::cerr << "BMS charge side did not swap!" << std::endl;
        return 0;
    }
    if (std::fabs((max_threshold - BmsPwrMgr::getCurrThreshold()) - decrease_threshold_amount) > 0.0001) {
        std::cerr << "BMS threshold did not decrease as expected!" << std::endl;
        return 0;
    }
    uint64_t expected_cmd;
    expected_cmd = (before_c_side == port) ? can_port_discharge_stbd_charge_cmd : can_port_charge_stbd_discharge_cmd;
    std::this_thread::sleep_for(std::chrono::seconds(2 * can_write_interval_seconds));
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

int test_regular_operation_charge_reaches_threshold(void) {
    BmsPwrMgr::configure(regular_operation, max_threshold - decrease_threshold_amount, port,
                         max_threshold - decrease_threshold_amount + 0.1,
                         max_threshold - decrease_threshold_amount + 0.1);
    int mock_socket = open("/tmp", O_TMPFILE | O_RDWR);
    if (mock_socket == -1) {
        std::cerr << "Failed to create mock socket" << std::endl;
        return 0;
    }
    BmsPwrMgr::init(mock_socket);
    battery_state_e before_state  = BmsPwrMgr::getCurrState();
    port_stbd_e     before_c_side = BmsPwrMgr::getCurrChargeSide();

    struct can_frame frame;
    canid_t charge_bms_id    = (before_c_side == port) ? BMS1_FRAME1_ID : BMS4_FRAME1_ID;

    BmsPwrMgr::onNewVoltageReading(charge_bms_id, max_voltage);
    if (BmsPwrMgr::getCurrState() != before_state) {
        std::cerr << "BMS state changed when they should not have!" << std::endl;
        return 0;
    }
    if (BmsPwrMgr::getCurrChargeSide() != ((~before_c_side) & 1)) {
        std::cerr << "BMS charge side did not swap!" << std::endl;
        return 0;
    }
    if ((max_threshold - BmsPwrMgr::getCurrThreshold()) >= 0.0001) {
        std::cerr << "BMS threshold did not increase as expected!" << std::endl;
        return 0;
    }
    uint64_t expected_cmd;
    expected_cmd = (before_c_side == port) ? can_port_discharge_stbd_charge_cmd : can_port_charge_stbd_discharge_cmd;
    std::this_thread::sleep_for(std::chrono::seconds(2 * can_write_interval_seconds));
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

int test_regular_operation_to_critical(void) {
    BmsPwrMgr::configure(regular_operation, min_threshold, port, min_threshold + surplus_to_decrease_threshold,
                         min_threshold + surplus_to_decrease_threshold);
    int mock_socket = open("/tmp", O_TMPFILE | O_RDWR);
    if (mock_socket == -1) {
        std::cerr << "Failed to create mock socket" << std::endl;
        return 0;
    }
    BmsPwrMgr::init(mock_socket);
    float           before_thresh = BmsPwrMgr::getCurrThreshold();
    port_stbd_e     before_c_side = BmsPwrMgr::getCurrChargeSide();

    struct can_frame frame;
    canid_t discharge_bms_id = (before_c_side == port) ? BMS4_FRAME1_ID : BMS1_FRAME1_ID;

    BmsPwrMgr::onNewVoltageReading(discharge_bms_id, min_threshold);
    if (before_thresh != BmsPwrMgr::getCurrThreshold()) {
        std::cerr << "Threshold changed when it should not have!" << std::endl;
        return 0;
    }
    if (BmsPwrMgr::getCurrState() != critical) {
        std::cerr << "State machine did not transition to critical state!" << std::endl;
        return 0;
    }
    if (BmsPwrMgr::getCurrChargeSide() != ((~before_c_side) & 1)) {
        std::cerr << "BMS charge side did not swap!" << std::endl;
        return 0;
    }
    uint64_t expected_cmd;
    expected_cmd = (before_c_side == port) ? can_port_discharge_stbd_charge_cmd : can_port_charge_stbd_discharge_cmd;
    std::this_thread::sleep_for(std::chrono::seconds(2 * can_write_interval_seconds));
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

int test_critical_no_action(void) {
    BmsPwrMgr::configure(critical, min_threshold, port, min_threshold, min_threshold);
    int mock_socket = open("/tmp", O_TMPFILE | O_RDWR);
    if (mock_socket == -1) {
        std::cerr << "Failed to create mock socket" << std::endl;
        return 0;
    }
    BmsPwrMgr::init(mock_socket);
    battery_state_e before_state  = BmsPwrMgr::getCurrState();
    float           before_thresh = BmsPwrMgr::getCurrThreshold();
    port_stbd_e     before_c_side = BmsPwrMgr::getCurrChargeSide();

    struct can_frame frame;
    float voltage = before_thresh + 0.1;
    BmsPwrMgr::onNewVoltageReading(BMS1_FRAME1_ID, voltage);

    if ((BmsPwrMgr::getCurrState() != before_state) || (BmsPwrMgr::getCurrThreshold() != before_thresh) ||
         BmsPwrMgr::getCurrChargeSide() != before_c_side) {
        std::cerr << "BMS fields changed when they should not have!" << std::endl;
        return 0;
    }
    if (read(mock_socket, &frame, sizeof(uint64_t)) != 0) {
        std::cerr << "CAN CMD was written when it should not have!" << std::endl;
        return 0;
    }
    return 1;
}

int test_critical_life_support(void) {
    BmsPwrMgr::configure(critical, min_threshold, port, min_threshold,
                         min_threshold);
    int mock_socket = open("/tmp", O_TMPFILE | O_RDWR);
    if (mock_socket == -1) {
        std::cerr << "Failed to create mock socket" << std::endl;
        return 0;
    }
    BmsPwrMgr::init(mock_socket);
    float           before_thresh = BmsPwrMgr::getCurrThreshold();
    port_stbd_e     before_c_side = BmsPwrMgr::getCurrChargeSide();

    struct can_frame frame;
    canid_t discharge_bms_id = (before_c_side == port) ? BMS4_FRAME1_ID : BMS1_FRAME1_ID;

    BmsPwrMgr::onNewVoltageReading(discharge_bms_id, life_support_threshold);
    if ((BmsPwrMgr::getCurrThreshold() != before_thresh) || BmsPwrMgr::getCurrState() != critical) {
        std::cerr << "BMS fields changed when they should not have!" << std::endl;
        return 0;
    }
    if (BmsPwrMgr::getCurrChargeSide() != ((~before_c_side) & 1)) {
        std::cerr << "BMS charge side did not swap!" << std::endl;
        return 0;
    }
    uint64_t expected_cmd;
    expected_cmd = (before_c_side == port) ? can_port_discharge_stbd_charge_cmd : can_port_charge_stbd_discharge_cmd;
    std::this_thread::sleep_for(std::chrono::seconds(2 * can_write_interval_seconds));
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

int test_critical_recovery(void) {
    BmsPwrMgr::configure(critical, min_threshold, port, min_threshold,
                         min_threshold);
    int mock_socket = open("/tmp", O_TMPFILE | O_RDWR);
    if (mock_socket == -1) {
        std::cerr << "Failed to create mock socket" << std::endl;
        return 0;
    }
    BmsPwrMgr::init(mock_socket);
    float           before_thresh = BmsPwrMgr::getCurrThreshold();
    port_stbd_e     before_c_side = BmsPwrMgr::getCurrChargeSide();

    struct can_frame frame;
    canid_t charge_bms_id = (before_c_side == port) ? BMS1_FRAME1_ID : BMS4_FRAME1_ID;

    BmsPwrMgr::onNewVoltageReading(charge_bms_id, min_threshold + surplus_to_increase_threshold);
    if (BmsPwrMgr::getCurrThreshold() != before_thresh) {
        std::cerr << "BMS threshold changed when it should not have!" << std::endl;
        return 0;
    }
    if (BmsPwrMgr::getCurrState() != regular_operation) {
        std::cerr << "BMS state did not change when it should have!" << std::endl;
        return 0;
    }
    if (BmsPwrMgr::getCurrChargeSide() != ((~before_c_side) & 1)) {
        std::cerr << "BMS charge side did not swap!" << std::endl;
        return 0;
    }
    uint64_t expected_cmd;
    expected_cmd = (before_c_side == port) ? can_port_discharge_stbd_charge_cmd : can_port_charge_stbd_discharge_cmd;
    std::this_thread::sleep_for(std::chrono::seconds(2 * can_write_interval_seconds));
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
    BmsPwrMgr::terminate();
    std::cout << "~~~ Test 2 ~~~" << std::endl;
    success_count += test_charge_side_at_max_threshold();
    BmsPwrMgr::terminate();
    std::cout << "~~~ Test 3 ~~~" << std::endl;
    success_count += test_discharge_reaches_threshold_do_not_lower();
    BmsPwrMgr::terminate();
    std::cout << "~~~ Test 4 ~~~" << std::endl;
    success_count += test_discharge_reaches_threshold_lower();
    BmsPwrMgr::terminate();
    std::cout << "~~~ Test 5 ~~~" << std::endl;
    success_count += test_regular_operation_charge_reaches_threshold();
    BmsPwrMgr::terminate();
    std::cout << "~~~ Test 6 ~~~" << std::endl;
    success_count += test_regular_operation_to_critical();
    BmsPwrMgr::terminate();
    std::cout << "~~~ Test 7 ~~~" << std::endl;
    success_count += test_critical_no_action();
    BmsPwrMgr::terminate();
    std::cout << "~~~ Test 8 ~~~" << std::endl;
    success_count += test_critical_life_support();
    BmsPwrMgr::terminate();
    std::cout << "~~~ Test 9 ~~~" << std::endl;
    success_count += test_critical_recovery();
    BmsPwrMgr::terminate();
    std::cout << success_count << " tests passed out of: " << num_tests << " total" << std::endl;
    return 0;
}
