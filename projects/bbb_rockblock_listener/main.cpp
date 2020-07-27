// Copyright 2017 UBC Sailbot

#include <iostream>
#include <boost/asio.hpp>
#include <thread>
#include <bitset>
#include <string>
#include <cstring>
#include <iomanip>
#include <boost/algorithm/string.hpp>
#include <mutex>

#include "Connection.h"
#include "Help.h"
#include "Sensors.pb.h"
#include "Uccms.pb.h"
#include "Satellite.pb.h"

// Stores serialized sensor and uccm data to send to rockblock
std::string latest_sensors_satellite_string;  // NOLINT(runtime/string)
std::string latest_uccms_satellite_string;  // NOLINT(runtime/string)

// How often to send sensor and uccm data
uint16_t sendSensors_freq;
uint16_t sendUccm_freq;

std::mutex serialPort_lck;
boost::asio::io_service io;
boost::asio::serial_port serial(io);

/*
 * boost::asio::read uses a non const reference,
 * so this function dues the same.
 * cpplint doesnt like this.
 */
std::string readLine(boost::asio::serial_port &p) {  // NOLINT(runtime/references)
    // Reading data char by char, code is optimized for simplicity, not speed
    char c;
    std::string result;
    while (true) {
        boost::asio::read(p, boost::asio::buffer(&c, 1));
        switch (c) {
            case '\r':
                break;
            case '\n':
                return result;
            default:
                result += c;
        }
    }
}

void send(const std::string &data) {
    // Send length of message
    std::string message_length("AT+SBDWB="+std::to_string(data.size())+"\r");
    boost::asio::write(serial, \
        boost::asio::buffer(message_length.c_str(), message_length.size()));

    std::cout << readLine(serial) << std::endl;
    std::cout << readLine(serial) << std::endl;

    auto data_c_str = data.c_str();
    int sum = 0;
    for (unsigned int i = 0; i < data.length(); i++) {
        sum += static_cast<uint8_t>(data_c_str[i]);
    }

    // Check sum is lower two bytes of sum of the message
    uint16_t check_sum_low = sum & 0x00FF;
    uint16_t check_sum_high = (sum & 0xFF00) >> 8;

    std::string message = data;
    message += check_sum_high;
    message += check_sum_low;
    message += '\r';

    boost::asio::write(serial, \
        boost::asio::buffer(message, message.size()));

    std::cout << readLine(serial) << std::endl;
    std::cout << readLine(serial) << std::endl;
    std::cout << readLine(serial) << std::endl;

    std::string send_command = "AT+SBDIX\r";

    boost::asio::write(serial, \
        boost::asio::buffer(send_command, send_command.size()));

    std::cout << readLine(serial) << std::endl;
    std::cout << readLine(serial) << std::endl;
    std::cout << readLine(serial) << std::endl;
}

void receive_message(std::string* response, const std::string &status) {
    if (status == "1") {
        readLine(serial);
        *response = readLine(serial);
    }
    if (status == "2") {
        *response = "Error checking mailbox";
    }
    if (status == "0") {
        *response = "Mailbox empty";
    }
}

void receive() {
    std::string sbdInit("AT+SBDIX\r");
    boost::asio::write(serial, \
        boost::asio::buffer(sbdInit.c_str(), sbdInit.size()));

    std::cout << readLine(serial) << std::endl;
    std::string response(readLine(serial));

    std::vector<std::string> response_split;
    boost::split(response_split, response, boost::is_any_of(","));

    for (unsigned int i = 0; i < response_split.size(); i++) {
        boost::algorithm::trim(response_split[i]);
    }

    std::string status(response_split[2]);

    std::string received_message;
    receive_message(&received_message, status);
    std::cout << received_message <<std::endl;
}

/* Callback function called when network table updated */
void RootCallback(NetworkTable::Node node, \
    const std::map<std::string, NetworkTable::Value> &diffs, \
    bool is_self_reply) {

    // Store updated network table data
    NetworkTable::Satellite sensors_satellite;
    NetworkTable::Satellite uccms_satellite;
    sensors_satellite.set_type(NetworkTable::Satellite::SENSORS);
    uccms_satellite.set_type(NetworkTable::Satellite::UCCMS);

    NetworkTable::Sensors sensors = NetworkTable::RootToSensors(&node);
    NetworkTable::Uccms uccms = NetworkTable::RootToUccms(&node);

    // TODO(alex): I don't think this is a memory leak,
    // but should test with valgrind
    // This creates an extra object (one on the stack and one on the heap)
    // but it shouldnt matter much. The stack one is going to get deallocated
    // pretty soon
    sensors_satellite.set_allocated_sensors(new NetworkTable::Sensors(sensors));
    uccms_satellite.set_allocated_uccms(new NetworkTable::Uccms(uccms));

    sensors_satellite.SerializeToString(&latest_sensors_satellite_string);
    uccms_satellite.SerializeToString(&latest_uccms_satellite_string);
}

/* Thread to send sensor data */
void sendSensorData() {
    std::cout << "started sensor thread" << std::endl;
    while (true) {
        if (latest_sensors_satellite_string.size() != 0) {
            std::cout << "sending sensor data" << std::endl;
            serialPort_lck.lock();
            send(latest_sensors_satellite_string);
            serialPort_lck.unlock();
            sleep(sendSensors_freq);
        }
    }
}

/* Thread to send uccm data*/
void sendUccmData() {
    while (true) {
        if (latest_uccms_satellite_string.size() != 0) {
            std::cout << "sending uccm data" << std::endl;
            serialPort_lck.lock();
            send(latest_uccms_satellite_string);
            serialPort_lck.unlock();
            sleep(sendUccm_freq);
        }
    }
}

int main(int argc, char **argv) {
    if (argc != 4) {
        std::cout << "usage: ./rockblock <sensors send "\
            << "frequency (seconds)> <uccm send frequency (seconds)> "\
            << "<path to serial port>" << std::endl;
        return 0;
    }

    sendSensors_freq = std::stoi(argv[1]);
    sendUccm_freq = std::stoi(argv[2]);

    std::string serialPort = argv[3];
    serial.open(serialPort);

    NetworkTable::Connection connection;
    connection.SetTimeout(100);
    try {
        connection.Connect();
    } catch (NetworkTable::TimeoutException) {
        std::cout << "Failed to connect" << std::endl;
        return 0;
    }

    latest_sensors_satellite_string = "";
    latest_uccms_satellite_string = "";

    connection.Subscribe("/", &RootCallback);

    serial.set_option(boost::asio::serial_port_base::baud_rate(19200));
    boost::asio::write(serial, boost::asio::buffer("AT\r", 3));
    std::cout << readLine(serial) << std::endl;
    std::cout << readLine(serial) << std::endl;

    boost::asio::write(serial, boost::asio::buffer("AT&K0\r", 6));
    std::cout << readLine(serial) << std::endl;
    std::cout << readLine(serial) << std::endl;

    std::thread sensorThread(sendSensorData);
    std::thread uccmThread(sendUccmData);

    sensorThread.join();
    uccmThread.join();
}
