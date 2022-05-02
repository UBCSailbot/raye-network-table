/**
*
*  Copyright 2017 UBC Sailbot
* 
*  @file  bbb_sat_listener.cpp
*  @brief Facilitates communication between the BBB 
*         Network Table & Land Server Network Table
*         via satellite
*  
*  Polls sensor data from the CANbus and publishes 
*  it to the network table. Writes actuation angles
*  outputted from the boat controller to the CANbus
*  
*  @author Alex Macdonald (Alexmac22347)
*  @author Brielle Law (briellelaw)
*  @author Henry Huang (hhenry01)
*  @author Vlada Kozachok (vladakozachok)
*
*/

#include <boost/asio/io_service.hpp>
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
NetworkTable::Value waypoint_data;  // Where we store waypoint segments before writing to the Network Table
                                    // Has type NetworkTable::Value::WAYPOINTS

/**
 *  Reads off the serial port until a newline detected
 *
 *  @param p serial port to read off of

 *  boost::asio::read uses a non const reference,
 *  so this function dues the same. cpplint doesnt like this.
 * 
 */
std::string readLine(boost::asio::serial_port &p, bool hex = false) {  // NOLINT(runtime/references)
    // Reading data char by char, code is optimized for simplicity, not speed
    // TODO(Henry) This will break if <cr><lf> appears naturally in data
    char c;
    std::string result;
    bool cr = false;
    while (true) {
        try {
            boost::asio::read(p, boost::asio::buffer(&c, 1));
            switch (c) {
                case '\r':
                    cr = true;
                    break;
                case '\n':
                    if (cr) {
                        std::cout << std::endl;
                       return result;
                    }
                default:
                    if (hex) {
                        char hex_byte[3];
                        std::sprintf(hex_byte, "%02x", c);  // NOLINT(runtime/printf)
                        if (cr) {
                            cr = false;
                            result += "0d";  // '\r' in hex
                        }
                        result += hex_byte;
                    } else {
                        if (cr) {
                            cr = false;
                            result += '\r';
                        }
                        result += c;
                    }
            }
        }
        catch (std::exception& e) {
            std::string error("ERROR - ");
            return error + e.what();
        }
    }
}

/**
 *  Converts a hex string to a bytestream 
 *
 *  @param hex The hex string to be converted 
 *
 */
std::vector<char> HexToBytes(const std::string& hex) {
    std::vector<char> bytes;

    // Convert each hex val to a long, but store within a char
    for (unsigned int i = 0; i < hex.length(); i += 2) {
        std::string byteString = hex.substr(i, 2);
        char byte = static_cast<char>(strtol(byteString.c_str(), NULL, 16));
        bytes.push_back(byte);
    }

    return bytes;
}

/**
 *  Sends data to the satellite 
 *
 *  @param data Data to be transmitted to the satellite
 *              Must be a string
 *
 */
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

    // Append the checksum to the message payload
    std::string message = data;
    message += check_sum_high;
    message += check_sum_low;
    message += '\r';

    // Send the message
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

/**
 * Callback function that runs when we timeout on waiting for more waypoint
 * message segments from the landserver
 */
void set_waypoints(void) {
    NetworkTable::Satellite sat;
    sat.set_type(NetworkTable::Satellite::VALUE);
    // Deallocation automatically handled:
    // https://stackoverflow.com/questions/33960999/protobuf-will-set-allocated-delete-the-allocated-object
    sat.set_allocated_value(new NetworkTable::Value(waypoint_data));
    connection.SetValue(WAYPOINTS_GP, sat.value());
    std::cout << "Setting waypoint data: \n" << sat.DebugString() << std::endl;
    waypoint_data.clear_waypoints();
}

/**
 *  Decodes messages received from the satellite as waypoints 
 *  and updates the network table
 *
 *  @param message Data to be decoded 
 *                 Expecting the message to contain waypoints
 *
 */
std::string decode_message(const std::string &message) {
    NetworkTable::Satellite satellite;

    // Convert the message back to a bytestream which can then be decoded as a protobuf obj
    std::vector<char> byte_message = HexToBytes(message);
    std::string str_data(byte_message.begin(), byte_message.end());
    satellite.ParseFromString(str_data);
    if (satellite.type() == NetworkTable::Satellite::VALUE &&
            satellite.value().type() == NetworkTable::Value::WAYPOINTS) {
        // append waypoints
        for (const auto &new_waypoint : satellite.value().waypoints()) {
            NetworkTable::Value_Waypoint *w = waypoint_data.add_waypoints();
            w->set_latitude(new_waypoint.latitude());
            w->set_longitude(new_waypoint.longitude());
        }
        return satellite.DebugString();
    } else {
        throw std::runtime_error("Failed to decode satellite data");
    }
}

/**
 *  Retrieves messages sent to the satellite 
 *
 *  @param status 1 if mailbox check was successful
 *                2 if mailbox check failed
 *                0 if mailbox is empty 
 *  Returns false on error
 */
bool receive_message(const std::string &status) {
    std::string message;

    if (status == "1") {
        // Command to receive binary MT message
        std::string msg = "AT+SBDRB\r";
        boost::asio::write(serial, boost::asio::buffer(msg.c_str(), msg.size()));

        // Response includes embedded payload
        // AT+SBDRB\nD<str_payload>
        std::string response = readLine(serial, true);
        std::string str_payload = response.substr(22);
        std::cout << readLine(serial) << std::endl;

        // Retrieve the decoded waypoint data
        message = decode_message(str_payload);
        std::cout << message << std::endl;
        return true;
    } else if (status == "2") {
        std::cout << "Error checking mailbox" << std::endl;
        return false;
    } else if (status == "0") {
        std::cout << "Mailbox empty" << std::endl;
        return true;
    } else {
        std::cout << "Non-valid status" << std::endl;
        return false;
    }
}

/**
 *  Check for and receive MT messages in the queue
 */
void receive() {
    std::lock_guard<std::mutex> lck(serialPort_mtx);
    std::cout << "Receiving Data" << std::endl;

    bool queueEmpty = true;
    int queueSize = 0;

    do {
        // Initiate an Extended SBD Session
        std::string sbdInit("AT+SBDIX\r");
        boost::asio::write(serial, boost::asio::buffer(sbdInit.c_str(), sbdInit.size()));
        std::cout << readLine(serial) << std::endl;

        std::string response(readLine(serial));
        std::cout << response << std::endl;

        // Break up Mailbox check response
        std::vector<std::string> response_split;
        boost::split(response_split, response, boost::is_any_of(","));

        for (unsigned int i = 0; i < response_split.size(); i++) {
            boost::algorithm::trim(response_split[i]);
        }

        // Extract the mailbox status and queue size
        receive_size = std::stoul(response_split[4], NULL, 0);
        std::string status(response_split[2]);
        std::cout << readLine(serial) << std::endl;
        std::cout << readLine(serial) << std::endl;
        // Read the expected waypoints off the seral port
        if (receive_message(status)) {
            queueSize = std::stoi(response_split[5], NULL, 0);
            if (queueSize > 0)
                queueEmpty = false;
        }
    } while (queueSize > 0);

    // Only set waypoints if we actually have contents from the queue
    if (!queueEmpty)
        set_waypoints();
}

/** 
 *  Sends updated sensor data back to the satellite 
 * 
 *  @param node          Updated network table node (expecting ActuationAngle)
 *  @param diffs         Uri and value of updated nt node
 *  @param is_self_reply Prevents recursive callbacks
 *
 */
void RootCallback(NetworkTable::Node node, \
    const std::map<std::string, NetworkTable::Value> &diffs, \
    bool is_self_reply) {

    // Check which nodes in the network table were updated
    bool is_waypoint = false;
    for (const auto& uris : diffs) {
        std::string uri = uris.first;
        std::cout << uri << std::endl;
        if (uri == WAYPOINTS_GP) {
            is_waypoint = true;
        }
    }

    // We do Not want to send back waypoint data that was just received
    if (!is_waypoint) {
        // Store updated network table data
        NetworkTable::Satellite sensors_satellite;
        NetworkTable::Satellite uccms_satellite;
        sensors_satellite.set_type(NetworkTable::Satellite::SENSORS);
        uccms_satellite.set_type(NetworkTable::Satellite::UCCMS);

        NetworkTable::Sensors sensors = NetworkTable::RootToSensors(&node);
        NetworkTable::Uccms uccms = NetworkTable::RootToUccms(&node);

        sensors_satellite.set_allocated_sensors(new NetworkTable::Sensors(sensors));
        uccms_satellite.set_allocated_uccms(new NetworkTable::Uccms(uccms));

        sensors_satellite.SerializeToString(&latest_sensors_satellite_string);
        uccms_satellite.SerializeToString(&latest_uccms_satellite_string);
    }
}

/**
 *  Send most recent sensor data
 */
void sendRecentSensors() {
    std::lock_guard<std::mutex> lck(serialPort_mtx);
    send(latest_sensors_satellite_string);
}

/**
 *  Send most recent uccm data
 */
void sendRecentUccms() {
    std::lock_guard<std::mutex> lck(serialPort_mtx);
    send(latest_uccms_satellite_string);
}

/** 
 *  Thread to periodically send sensor data
 */
void sendSensorData() {
    while (true) {
        if (latest_sensors_satellite_string.size() != 0) {
            std::cout << "sending sensor data" << std::endl;
            sendRecentSensors();
            sleep(sendSensors_freq);
        }
    }
}

/** 
 *  Thread to periodically send uccm data
 */
void sendUccmData() {
    while (true) {
        if (latest_uccms_satellite_string.size() != 0) {
            std::cout << "sending uccm data" << std::endl;
            sendRecentUccms();
            sleep(sendUccm_freq);
        }
    }
}

/** 
 *  Thread to periodically retrieve satellite messages 
 */
void receiveData() {
    while (true) {
        receive();
        sleep(receive_freq);
    }
}

/**
 *  Spin off the threads responsible for receiving from and sending to satellite
 */
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

    // TODO(brielle) surround in try/catch
    connection.Connect(1000, true);

    latest_sensors_satellite_string = "";
    latest_uccms_satellite_string = "";

    // Send all network updates back to the land server via satellite
    bool is_subscribed = false;
    while (!is_subscribed) {
        try {
            connection.Subscribe("/", &RootCallback);
            is_subscribed = true;
        }
        catch (NetworkTable::NotConnectedException) {
            std::cout << "Fail to connect" << std::endl;
            sleep(1);
        }
    }

    waypoint_data.set_type(NetworkTable::Value::WAYPOINTS);

    serial.set_option(boost::asio::serial_port_base::baud_rate(19200));

    // Clear SBD Message buffers
    boost::asio::write(serial, boost::asio::buffer("AT+SBDD2\r", 9));
    std::cout << readLine(serial) << std::endl;
    std::cout << readLine(serial) << std::endl;
    std::cout << readLine(serial) << std::endl;
    std::cout << readLine(serial) << std::endl;

    boost::asio::write(serial, boost::asio::buffer("AT\r", 3));
    std::cout << readLine(serial) << std::endl;
    std::cout << readLine(serial) << std::endl;

    boost::asio::write(serial, boost::asio::buffer("AT&K0\r", 6));
    std::cout << readLine(serial) << std::endl;
    std::cout << readLine(serial) << std::endl;

    std::thread sensorThread(sendSensorData);
    std::thread uccmThread(sendUccmData);
    std::thread receiveThread(receiveData);

    sensorThread.join();
    uccmThread.join();
    receiveThread.join();
}
