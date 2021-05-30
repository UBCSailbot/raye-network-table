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
#include <stdexcept>
#include <vector>
#include <sstream>
#include <iomanip>
#include <cstdio>

#include "Connection.h"
#include "Help.h"
#include "Sensors.pb.h"
#include "Uccms.pb.h"
#include "Satellite.pb.h"
#include "Value.pb.h"
#include "Exceptions.h"
#include "Uri.h"

// Stores serialized sensor and uccm data to send to rockblock
std::string latest_sensors_satellite_string;  // NOLINT(runtime/string)
std::string latest_uccms_satellite_string;  // NOLINT(runtime/string)

// How often to send sensor and uccm data
uint16_t sendSensors_freq;
uint16_t sendUccm_freq;

std::mutex serialPort_mtx;
boost::asio::io_service io;
boost::asio::serial_port serial(io);

uint16_t receive_size;
uint16_t receive_freq;

NetworkTable::Connection connection;

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

std::vector<char> HexToBytes(const std::string& hex) {
  std::vector<char> bytes;
  for (unsigned int i = 0; i < hex.length(); i += 2) {
    std::string byteString = hex.substr(i, 2);
    char byte = (char) strtol(byteString.c_str(), NULL, 16);
    bytes.push_back(byte);
  }
  return bytes;
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

    // Check mailbox
    std::string send_command = "AT+SBDIX\r";

    boost::asio::write(serial, \
        boost::asio::buffer(send_command, send_command.size()));

    std::cout << readLine(serial) << std::endl;
    std::cout << readLine(serial) << std::endl;
    std::cout << readLine(serial) << std::endl;

    std::cout << readLine(serial) << std::endl;
    std::cout << readLine(serial) << std::endl;
}

std::string decode_message(std::string message) {
    NetworkTable::Satellite satellite;
	//std::vector<char> hex_message;
	//char hex_message[100];
	char hex_message[receive_size];
    int i = 0;
    for (const auto &item : message) {
        std::sprintf(&hex_message[i*2], "%02x", int(item)); 
        i++;
    }

    //std::vector<char> byte_message = HexToBytes(std::string(hex_message.begin(), hex_message.end()));
    std::vector<char> byte_message = HexToBytes(std::string(hex_message));
    std::string str_data(byte_message.begin(), byte_message.end());
    satellite.ParseFromString(str_data);

    if (satellite.type() == NetworkTable::Satellite::VALUE &&
               satellite.value().type() == NetworkTable::Value::WAYPOINTS) {
        std::cout << "WAYPOINT DATA" << std::endl;
        connection.SetValue("waypoints", satellite.value());
        return satellite.DebugString();
    } else {
        throw std::runtime_error("Failed to decode satellite data");
    }
}

std::string receive_message(const std::string &status) {
    std::string message;
    //char temp[100];

    if (status == "1") {
        // Command to receive binary MT message
        std::string msg = "AT+SBDRB\r";
        boost::asio::write(serial, boost::asio::buffer(msg.c_str(), msg.size()));

		// Response includes embedded payload
		// AT+SBDRB__<str_payload>
        std::string response = readLine(serial);
        std::string str_payload = response.substr(10);
        std::cout << readLine(serial) << std::endl;

		message = decode_message(str_payload);

		// TODO: Check if there are other response messages sent from Rockblock
		// May cause the system to hang if there no other messages
        //std::cout << readLine(serial) << std::endl;
        //std::cout << readLine(serial) << std::endl;
        //std::cout << readLine(serial) << std::endl;

    } else if (status == "2") {
        message = "Error checking mailbox";
    } else if (status == "0") {
        std::string null_msg = "Mailbox empty";
        message  = null_msg;
    } else {
        message  = "Non-valid status";
    }
    return message;
}

/* Check for and receive MT messages in queue */
void receive() {
    std::lock_guard<std::mutex> lck(serialPort_mtx);
    std::cout << "Receiving Data" << std::endl;

    // Execute Ring Indicator Status command
    std::string alertInit("AT+CRIS\r");
    boost::asio::write(serial, boost::asio::buffer(alertInit.c_str(), alertInit.size()));
    std::cout << readLine(serial) << std::endl;
    std::string alertStatus(readLine(serial));
    std::cout << alertStatus <<std::endl;
    std::cout << readLine(serial) << std::endl;

    // Break up ring alert check response
    std::vector<std::string> alert_split;
    boost::split(alert_split, alertStatus, boost::is_any_of(","));

    for (unsigned int i = 0; i < alert_split.size(); i++) {
        boost::algorithm::trim(alert_split[i]);
        std::cout << alert_split[i] << std::endl;
    }

    int sri = std::stoul(alert_split[1], NULL, 0);
    if (sri) {
        // Initiate an Extended SBD Session
        std::string sbdInit("AT+SBDIXA\r");
        boost::asio::write(serial, boost::asio::buffer(sbdInit.c_str(), sbdInit.size()));

        std::cout << readLine(serial) << std::endl;
        std::cout << readLine(serial) << std::endl;
        std::string response(readLine(serial));
        std::cout << response << std::endl;

        // Break up Mailbox check response
        std::vector<std::string> response_split;
        boost::split(response_split, response, boost::is_any_of(","));

        for (unsigned int i = 0; i < response_split.size(); i++) {
            boost::algorithm::trim(response_split[i]);
        }

        receive_size = std::stoul(response_split[4], NULL, 0);
        std::string status(response_split[2]);
        int queueSize = std::stoi(response_split[5], NULL, 0);

        std::cout << readLine(serial) << std::endl;
        std::cout << readLine(serial) << std::endl;

        std::string received_message = receive_message(status);
        std::cout << received_message <<std::endl;
        queueSize--;

        // If the queue is empty, wait before polling again
        if (queueSize < 0) {
            std::cout << "waiting" << std::endl;
        }
    }
}

/* Callback function called when network table updated */
void RootCallback(NetworkTable::Node node, \
    const std::map<std::string, NetworkTable::Value> &diffs, \
    bool is_self_reply) {

    for (const auto& uris : diffs) {
        std::string uri = uris.first;
        std::cout << uri << std::endl;
		// We do Not want to send back waypoint data that was just received
        if (uri != WAYPOINTS_GP) {
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
	}
}

/* Send most recent sensor data */
void sendRecentSensors() {
    std::lock_guard<std::mutex> lck(serialPort_mtx);
    send(latest_sensors_satellite_string);
}

/* Send most recent uccm data */
void sendRecentUccms() {
    std::lock_guard<std::mutex> lck(serialPort_mtx);
    send(latest_uccms_satellite_string);
}

/* Thread to send sensor data */
void sendSensorData() {
    while (true) {
        if (latest_sensors_satellite_string.size() != 0) {
            std::cout << "sending sensor data" << std::endl;
            sendRecentSensors();
            sleep(sendSensors_freq);
        }
    }
}

/* Thread to send uccm data*/
void sendUccmData() {
    while (true) {
        if (latest_uccms_satellite_string.size() != 0) {
            std::cout << "sending uccm data" << std::endl;
            sendRecentUccms();
            sleep(sendUccm_freq);
        }
    }
}

void receiveData() {
    while (true) {
        receive();
        sleep(receive_freq);
    }
}

int main(int argc, char **argv) {
    if (argc != 5) {
        std::cout << "usage: ./bbb_rockblock_listener <sensors send "\
            << "frequency (seconds)> <uccm send frequency (seconds)> "\
            << "<waypt receive frequency (seconds)> "\
            << "<path to serial port>" << std::endl;
        return 0;
    }

    sendSensors_freq = std::stoi(argv[1]);
    sendUccm_freq = std::stoi(argv[2]);
    receive_freq = std::stoi(argv[3]);

    std::string serialPort = argv[4];
    serial.open(serialPort);

    receive_size = 0;

    connection.Connect(1000, true);

    latest_sensors_satellite_string = "";
    latest_uccms_satellite_string = "";

    bool is_subscribed = false;

    while (!is_subscribed) {
        try {
            // TODO: Satellite will try to send back received waypoint data
            //       This is an unecessary use of credits
            connection.Subscribe("/", &RootCallback);
            is_subscribed = true;
        }
        catch (NetworkTable::NotConnectedException) {
            std::cout << "Fail to connect" << std::endl;
            sleep(1);
        }
    }

    serial.set_option(boost::asio::serial_port_base::baud_rate(19200));
    boost::asio::write(serial, boost::asio::buffer("AT\r", 3));
    std::cout << readLine(serial) << std::endl;
    std::cout << readLine(serial) << std::endl;

    boost::asio::write(serial, boost::asio::buffer("AT&K0\r", 6));
    std::cout << readLine(serial) << std::endl;
    std::cout << readLine(serial) << std::endl;

    // Enable ring alerts
    boost::asio::write(serial, boost::asio::buffer("AT+SBDMTA=1\r", 12));
    std::cout << readLine(serial) << std::endl;
    std::cout << readLine(serial) << std::endl;

    std::thread sensorThread(sendSensorData);
    std::thread uccmThread(sendUccmData);
    std::thread receiveThread(receiveData);

    sensorThread.join();
    uccmThread.join();
    receiveThread.join();
}
