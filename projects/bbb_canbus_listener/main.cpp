/**
*
*  Copyright 2017 UBC Sailbot
* 
*  @file  bbb_canbus_listener.cpp
*  @brief Facilitates communication between CANbus and Network Table
*  
*  Polls sensor data from the CANbus and publishes 
*  it to the network table. Writes actuation angles
*  outputted from the boat controller to the CANbus
*  
*  @author Alex Macdonald (Alexmac22347)
*  @author Brielle Law (briellelaw)
*  @author John Ahn (jahn18)
*
*/

#include <stdio.h>
#include <map>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <iostream>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <stdint.h>
#include "uccm-sensors/frame_parser.h"
#include "Connection.h"
#include "Value.pb.h"
#include "Exceptions.h"
#include "Uri.h"

#define WIND_ID1 "1"
#define WIND_ID2 "2"
#define WIND_ID3 "3"
#define CAN_DLC  8

int s;
NetworkTable::Connection connection;

/**
 *  Set wind sensor data in the network table. 
 *  
 *  @param angle  value of wind angle (degrees)  
 *  @param speed  value of wind speed (knots)
 *  @param id     wind sensor id should be 1, 2, or 3
 *
 */
void SetWindSensorData(int angle, float speed, const std::string &id) {
    // Assign values to new network table nodes
    NetworkTable::Value angle_nt;
    angle_nt.set_type(NetworkTable::Value::INT);
    angle_nt.set_int_data(static_cast<int>(angle));

    NetworkTable::Value speed_nt;
    speed_nt.set_type(NetworkTable::Value::FLOAT);
    speed_nt.set_float_data(static_cast<float>(speed));

    std::cout << "WindSensor" << id << std::endl;
    std::cout << "Angle: " << std::to_string(angle) \
        << " || Speed: " << std::to_string(speed) << std::endl;

    std::map<std::string, NetworkTable::Value> values;

    // Based on the sensor id, map the nodes to the corresponding uri
    if (id == WIND_ID1) {
        values.insert((std::pair<std::string, NetworkTable::Value> \
                (WIND1_SPEED, speed_nt)));
        values.insert((std::pair<std::string, NetworkTable::Value> \
                (WIND1_ANGLE, angle_nt)));
    } else if (id == WIND_ID2) {
        values.insert((std::pair<std::string, NetworkTable::Value> \
                (WIND2_SPEED, speed_nt)));
        values.insert((std::pair<std::string, NetworkTable::Value> \
                (WIND2_ANGLE, angle_nt)));
    } else if (id == WIND_ID3) {
        values.insert((std::pair<std::string, NetworkTable::Value> \
                (WIND3_SPEED, speed_nt)));
        values.insert((std::pair<std::string, NetworkTable::Value> \
                (WIND3_ANGLE, angle_nt)));
    }

    // Try to set the values in the network table
    try {
        connection.SetValues(values);
    } catch (NetworkTable::NotConnectedException) {
        std::cout << "Failed to set value" << std::endl;
    } catch (NetworkTable::TimeoutException) {
        std::cout << "Timeout" << std::endl;
    }
}

/**
 * Set BMS daa in the network table.
 *
 * @param frame: can frame containing the data to set.
 * @param BMSX_VOLTAGE: URI for voltage of BMSX (ex. BMS1_VOLTAGE). See uri.h for more info.
 * @param BMSX_CURRENT: URI for current of BMSX.
 * @param BMSX_MAXCELL: URI for maxcell of BMSX.
 * @param BMSX_MINCELL: URI for mincell of BMSX.
 */
void setBMSData(const struct can_frame& frame, const std::string& BMSX_VOLTAGE,
                const std::string& BMSX_CURRENT, const std::string& BMSX_MAXCELL,
                const std::string& BMSX_MINCELL) {
     std::map<std::string, NetworkTable::Value> values;
    NetworkTable::Value bms_volt_data;
    float volt_data = GET_BMS_BATTERY_VOLT_DATA(frame.data);
    bms_volt_data.set_type(NetworkTable::Value::FLOAT);
    bms_volt_data.set_float_data(volt_data);
    values.insert(std::pair<std::string, NetworkTable::Value>\
            (BMSX_VOLTAGE, bms_volt_data));
    std::cout << "volt_data:" << volt_data << std::endl;

    NetworkTable::Value bms_curr_data;
    float curr_data = GET_BMS_BATTERY_CURR_DATA(frame.data);
    bms_curr_data.set_type(NetworkTable::Value::FLOAT);
    bms_curr_data.set_float_data(curr_data);
    values.insert(std::pair<std::string, NetworkTable::Value>\
            (BMSX_CURRENT, bms_curr_data));
    std::cout << "curr_data:" << curr_data << std::endl;

    NetworkTable::Value bms_maxcell_data;
    uint16_t maxcell_data = GET_BMS_BATTERY_MAX_VOLT_DATA(frame.data);
    bms_maxcell_data.set_type(NetworkTable::Value::INT);
    bms_maxcell_data.set_int_data(maxcell_data);
    values.insert(std::pair<std::string, NetworkTable::Value>\
            (BMSX_MAXCELL, bms_maxcell_data));
    std::cout << "maxcell_data:" << maxcell_data << std::endl;

    NetworkTable::Value bms_mincell_data;
    uint16_t mincell_data = GET_BMS_BATTERY_MIN_VOLT_DATA(frame.data);
    bms_mincell_data.set_type(NetworkTable::Value::INT);
    bms_mincell_data.set_int_data(mincell_data);
    values.insert(std::pair<std::string, NetworkTable::Value>\
            (BMSX_MINCELL, bms_mincell_data));
    std::cout << "mincell_data:" << mincell_data << std::endl;

    try {
        connection.SetValues(values);
    } catch (NetworkTable::NotConnectedException) {
        std::cout << "Failed to set value" << std::endl;
    } catch (NetworkTable::TimeoutException) {
        std::cout << "Timeout" << std::endl;
    }
}

inline void SetRudderFrame(float angle, struct can_frame *frame) {
    // Manually split the float into bytes, and
    // put each byte into the frame.data array
    // The CAN frame data format is the same for all four angles
    uint8_t const *angle_array = reinterpret_cast<uint8_t *>(&angle);
    frame->data[0] = angle_array[0];
    frame->data[1] = angle_array[1];
    frame->data[2] = angle_array[2];
    frame->data[3] = angle_array[3];
}

inline void SetWinchJibFrame(uint16_t angle, struct can_frame *frame) {
    // Winch and jib are only 16 bit integers,
    // so we need to reset the upper 2 bytes.
    frame->data[0] = angle & 0xFF;
    frame->data[1] = (angle >> 8) & 0xFF;
    frame->data[2] = 0;
    frame->data[3] = 0;
}
/**
 *  Write the actuation angles to the CANbus 
 *  Callback in response to updates to the ActuationAngle nt node
 *  
 *  @param node          Updated network table node (expecting ActuationAngle)
 *  @param diffs         Uri and value of updated nt node
 *  @param is_self_reply Prevents recursive callbacks
 *
 */
void MotorCallback(NetworkTable::Node node, \
        const std::map<std::string, NetworkTable::Value> &diffs, \
        bool is_self_reply) {
    struct can_frame frame;
    frame.can_dlc = CAN_DLC;

    for (const auto& uris : diffs) {
        std::string uri = uris.first;

        // Determine which URI the data is associated with
        if (uri == RUDDER_PORT_ANGLE) {
            frame.can_id = RUDDER_PORT_CMD_FRAME_ID;
            float angle = \
                static_cast<float>(node.children().at("rudder_port").children().at("angle").value().float_data());
            SetRudderFrame(angle, &frame);
            std::cout << "Sending rudder port angle: " << angle << std::endl;
        } else if (uri == RUDDER_STBD_ANGLE) {
            frame.can_id = RUDDER_STBD_CMD_FRAME_ID;
            float angle = \
                static_cast<float>(node.children().at("rudder_stbd").children().at("angle").value().float_data());
            SetRudderFrame(angle, &frame);
            std::cout << "Sending rudder starboard angle: " << angle << std::endl;
        } else if (uri == WINCH_MAIN_ANGLE) {
            frame.can_id = WINCH_MAIN_ANGLE_FRAME_ID;
            uint16_t angle = \
                static_cast<int>(node.children().at("winch_main").children().at("angle").value().int_data());
            SetWinchJibFrame(angle, &frame);
            std::cout << "Sending winch main angle: " << angle << std::endl;
        } else if (uri == WINCH_JIB_ANGLE) {
            frame.can_id = WINCH_JIB_ANGLE_FRAME_ID;
            uint16_t angle = \
                static_cast<int>(node.children().at("winch_jib").children().at("angle").value().int_data());
            SetWinchJibFrame(angle, &frame);
            std::cout << "Sending jib angle: " << angle << std::endl;
        } else {
            break;
        }

        std::cout << uris.first << std::endl;

        // Write the rudder angle to the corresponding can frame
        if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            perror("Write");
            return;
        }
    }
}

/**
 *  Write the Power Controller outputs to the CANbus 
 *  Callback in response to updates to the PowerController nt node
 *  
 *  @param node          Updated network table node (expecting PowerController)
 *  @param diffs         Uri and value of updated nt node
 *  @param is_self_reply Prevents recursive callbacks
 *
 */
void PowerControllerCallback(NetworkTable::Node node, \
        const std::map<std::string, NetworkTable::Value> &diffs, \
        bool is_self_reply) {
    struct can_frame frame;
    int power_data;
    frame.can_dlc = CAN_DLC;

    for (const auto& uris : diffs) {
        std::string uri = uris.first;

        // TODO(brielle&john): not accessing battery_state's children nodes
        // Verify updated node corresponds to power controller output
        if (uri == POWER_CONTROLLER) {
            frame.can_id = BMS_CMD_FRAME_ID;
            power_data = static_cast<float>(node.children().at("battery_state").value().int_data());
        } else {
            break;
        }

        std::cout << uris.first << std::endl;
        // Manually put split the float into bytes, and
        // put each byte into the frame.data array
        uint8_t const *power_array = reinterpret_cast<uint8_t *>(&power_data);
        frame.data[0] = power_array[0];
        frame.data[1] = power_array[1];
        frame.data[2] = power_array[2];
        frame.data[3] = power_array[3];
        std::cout << "Sending Power Controller state: " << power_data << std::endl;

        // Write the Power Controller state to the corresponding can frame
        if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            perror("Write");
            return;
        }
    }
}

/**
 * Connects to the network table, polls the CANbus for sensor updates,
 * and writes actuation angle and power controller updates to the CANbus  
 *
 */
int main(int argc, char **argv) {
    if (argc != 2) {
        printf("Please provide the name of the canbus interface. \n");
        printf("Example usage: './bbb_canbus_listener can0' \n");
        return 0;
    }

    std::cout << "Running bbb_canbus_listener..." << std::endl;

    // Connect to the network table
    // TODO(brielle): Surround in try/catch
    connection.Connect(1000, true);

    // Connect to the canbus network.
    // It should show up as a network interface.
    // You should see it with the ifconfig command.
    struct sockaddr_can addr;
    struct can_frame frame;
    struct ifreq ifr;

    char *ifname = argv[1];

    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        std::cout << "Error while opening socket";
        return -1;
    }

    strcpy(ifr.ifr_name, ifname);  // NOLINT(runtime/printf)
    ioctl(s, SIOCGIFINDEX, &ifr);

    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        std::cout << "Error in socket bind";
        return -2;
    }

    // Subscribe to rudder node in the network-table
    bool is_subscribed = false;

    while (!is_subscribed) {
        try {
            connection.Subscribe(RUDDER, &MotorCallback);
            is_subscribed = true;
        }
        catch (NetworkTable::NotConnectedException) {
            std::cout << "Failed to subscribe to actuation_angle" << std::endl;
            sleep(1);
        }
    }

    is_subscribed = false;

    // Subscribe to Power Controller node in the network-table
    while (!is_subscribed) {
        try {
            connection.Subscribe(BMS, &PowerControllerCallback);
            is_subscribed = true;
        }
        catch (NetworkTable::NotConnectedException) {
            std::cout << "Failed to subscribe to Power Controller" << std::endl;
            sleep(1);
        }
    }

    // Keep on reading the sensor data off canbus, and
    // placing the latest data in the network table.
    while (true) {
        read(s, &frame, sizeof(struct can_frame));
        std::cout << "Can ID = " << std::hex << frame.can_id << std::endl << std::dec;
        switch (frame.can_id) {
            case WIND_SENS1_FRAME_ID : {
                std::cout << "Received Wind Sensor 0 Frame" << std::endl;
                int angle = GET_WIND_ANGLE(frame.data);
                float speed = GET_WIND_SPEED(frame.data);
                SetWindSensorData(angle, speed, "1");
                break;
            }
            case WIND_SENS2_FRAME_ID : {
                std::cout << "Received Wind Sensor 1 Frame" << std::endl;
                int angle = GET_WIND_ANGLE(frame.data);
                float speed = GET_WIND_SPEED(frame.data);
                SetWindSensorData(angle, speed, "2");
                break;
            }
            case WIND_SENS3_FRAME_ID : {
                std::cout << "Received Wind Sensor 2 Frame" << std::endl;
                int angle = GET_WIND_ANGLE(frame.data);
                float speed = GET_WIND_SPEED(frame.data);
                SetWindSensorData(angle, speed, "3");
                break;
            }
            case SAILENCODER_FRAME_ID : {
                std::cout << "Received Sailencoder Frame" << std::endl;
                std::map<std::string, NetworkTable::Value> values;
                NetworkTable::Value boom_angle;
                int angle = GET_SAILENCODER_ANGLE(frame.data);
                boom_angle.set_type(NetworkTable::Value::INT);
                boom_angle.set_int_data(static_cast<int>(angle));
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        (SAILENCODER_ANGLE, boom_angle));
                try {
                    connection.SetValues(values);
                } catch (NetworkTable::NotConnectedException) {
                    std::cout << "Failed to set value" << std::endl;
                } catch (NetworkTable::TimeoutException) {
                    std::cout << "Timeout" << std::endl;
                }

                std::cout << "sailencoder value: " << std::dec << angle << std::dec << std::endl;
                break;
            }
            case GPS_LONG_FRAME_ID : {
                std::cout << "Received GPS Long Frame" << std::endl;

                std::map<std::string, NetworkTable::Value> values;
                NetworkTable::Value gps_longitude;

                // Perform a unit conversion from decimal degrees minutes to degrees.
                double longitudeDDM = GET_GPS_LONG(frame.data);
                int longitudeDD = longitudeDDM / 100;
                double longitudeM = longitudeDDM - (longitudeDD * 100);
                // Set to negative as we are always in the Western hempishere
                double longitude_degrees = -(longitudeDD + (longitudeM / 60));

                gps_longitude.set_type(NetworkTable::Value::FLOAT);
                gps_longitude.set_float_data(static_cast<float>(longitude_degrees));
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        (GPS_CAN_LON, gps_longitude));
                try {
                    connection.SetValues(values);
                } catch (NetworkTable::NotConnectedException) {
                    std::cout << "Failed to set value" << std::endl;
                } catch (NetworkTable::TimeoutException) {
                    std::cout << "Timeout" << std::endl;
                }

                std::cout << "longitude = " << longitude_degrees << " " << std::endl;
                break;
            }
            case GPS_LAT_FRAME_ID : {
                std::cout << "Received GPS Lat Frame" << std::endl;

                std::map<std::string, NetworkTable::Value> values;
                NetworkTable::Value gps_latitude;

                // Perform a unit conversion from decimal degrees minutes to degrees
                double latitudeDDM = GET_GPS_LAT(frame.data);
                int latitudeDD = latitudeDDM / 100;
                double latitudeM = latitudeDDM - (latitudeDD * 100);
                double latitude_degrees = latitudeDD + (latitudeM / 60);

                gps_latitude.set_type(NetworkTable::Value::FLOAT);
                gps_latitude.set_float_data(static_cast<float>(latitude_degrees));
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        (GPS_CAN_LAT, gps_latitude));
                try {
                    connection.SetValues(values);
                } catch (NetworkTable::NotConnectedException) {
                    std::cout << "Failed to set value" << std::endl;
                } catch (NetworkTable::TimeoutException) {
                    std::cout << "Timeout" << std::endl;
                }

                std::cout << "latitude = " << latitude_degrees << " " << std::endl;
                break;
            }
            case GPS_OTHER_FRAME_ID : {
                std::cout << "Received GPS Other Frame" << std::endl;

                std::map<std::string, NetworkTable::Value> values;
                NetworkTable::Value gps_gndSpeed;
                float gndSpeed = GET_GPS_GND_SPEED(frame.data);
                gps_gndSpeed.set_type(NetworkTable::Value::FLOAT);
                gps_gndSpeed.set_float_data(static_cast<float>(gndSpeed));
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        (GPS_CAN_GNDSPEED, gps_gndSpeed));
                std::cout << "gnd speed = " << gndSpeed << " " << std::endl;

                NetworkTable::Value gps_magVar;
                float magVar = GET_GPS_MAG_VAR(frame.data);
                gps_magVar.set_type(NetworkTable::Value::FLOAT);
                gps_magVar.set_float_data(static_cast<float>(magVar));
                values.insert((std::pair<std::string, NetworkTable::Value>\
                        (GPS_CAN_MAGVAR, gps_magVar)));
                std::cout << "mag var =  " << magVar << " " << std::endl;

                NetworkTable::Value gps_TMG;
                float gpsTMG = GET_GPS_TMG(frame.data);
                gps_TMG.set_type(NetworkTable::Value::FLOAT);
                gps_TMG.set_float_data(static_cast<float>(gpsTMG));
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        (GPS_CAN_TMG, gps_TMG));
                std::cout << "gps tmg =  " << gpsTMG << " " << std::endl;

                NetworkTable::Value gps_true_heading;
                float true_heading = GET_GPS_TRUE_HEADING(frame.data);
                gps_true_heading.set_type(NetworkTable::Value::FLOAT);
                gps_true_heading.set_float_data(static_cast<float>(true_heading));
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        (GPS_CAN_TRUE_HEADING, gps_true_heading));
                std::cout << "gps true heading =  " << true_heading << " " << std::endl;

                try {
                    connection.SetValues(values);
                } catch (NetworkTable::NotConnectedException) {
                    std::cout << "Failed to set value" << std::endl;
                } catch (NetworkTable::TimeoutException) {
                    std::cout << "Timeout" << std::endl;
                }
                break;
            }
            case GPS_DATE_FRAME_ID : {
                std::cout << "Received GPS Date Frame" << std::endl;

                std::map<std::string, NetworkTable::Value> values;
                NetworkTable::Value gps_timestamp;
                std::string hour = std::to_string(GET_HOUR(frame.data));
                std::string minute = std::to_string(GET_MINUTE(frame.data));
                std::string second = std::to_string(GET_SECOND(frame.data));
                std::string day = std::to_string(GET_DAY(frame.data));
                std::string month = std::to_string(GET_MONTH(frame.data));
                std::string year = std::to_string(GET_YEAR(frame.data));
                std::string utc_timestamp = year + "/" + month + "/" + day + \
                                            "-" + hour + ":" + minute + ":" + second;
                gps_timestamp.set_type(NetworkTable::Value::STRING);
                gps_timestamp.set_string_data(utc_timestamp);
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        (GPS_CAN_TIME, gps_timestamp));


                NetworkTable::Value gps_valid;
                bool valid = GET_STATUS(frame.data);
                gps_valid.set_type(NetworkTable::Value::BOOL);
                gps_valid.set_bool_data(valid);
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        (GPS_CAN_VALID, gps_valid));

                NetworkTable::Value gps_varWest;
                bool var_west = GET_VAR_WEST(frame.data);
                gps_varWest.set_type(NetworkTable::Value::BOOL);
                gps_varWest.set_bool_data(var_west);
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        (GPS_CAN_VARWEST, gps_varWest));

                NetworkTable::Value gps_latNorth;
                bool lat_north = GET_VAR_NORTH(frame.data);
                gps_latNorth.set_type(NetworkTable::Value::BOOL);
                gps_latNorth.set_bool_data(lat_north);
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        (GPS_CAN_LATNORTH, gps_latNorth));

                NetworkTable::Value gps_lonWest;
                bool var_long_west = GET_LONG_WEST(frame.data);
                gps_lonWest.set_type(NetworkTable::Value::BOOL);
                gps_lonWest.set_bool_data(var_long_west);
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        (GPS_CAN_LONWEST, gps_lonWest));

                try {
                    connection.SetValues(values);
                } catch (NetworkTable::NotConnectedException) {
                    std::cout << "Failed to set value" << std::endl;
                } catch (NetworkTable::TimeoutException) {
                    std::cout << "Timeout" << std::endl;
                }
                break;
            }
            case GYRO_FRAME_ID: {
                std::cout << "Received Gyroscope Frame" << std::endl;
                std::map<std::string, NetworkTable::Value> values;

                NetworkTable::Value gyro_x_data;
                float x_gyro = GET_GYRO_X_DATA(frame.data);
                gyro_x_data.set_type(NetworkTable::Value::FLOAT);
                gyro_x_data.set_float_data(x_gyro);
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        (GYROSCOPE_X, gyro_x_data));
                std::cout << "gyro_x_data:" << x_gyro << std::endl;

                NetworkTable::Value gyro_y_data;
                float y_gyro = GET_GYRO_Y_DATA(frame.data);
                gyro_y_data.set_type(NetworkTable::Value::FLOAT);
                gyro_y_data.set_float_data(y_gyro);
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        (GYROSCOPE_Y, gyro_y_data));
                std::cout << "gyro_y_data:" << y_gyro << std::endl;

                NetworkTable::Value gyro_z_data;
                float z_gyro = GET_GYRO_Z_DATA(frame.data);
                gyro_z_data.set_type(NetworkTable::Value::FLOAT);
                gyro_z_data.set_float_data(z_gyro);
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        (GYROSCOPE_Z, gyro_z_data));
                std::cout << "gyro_z_data:" << z_gyro << std::endl;

                try {
                    connection.SetValues(values);
                } catch (NetworkTable::NotConnectedException) {
                    std::cout << "Failed to set value" << std::endl;
                } catch (NetworkTable::TimeoutException) {
                    std::cout << "Timeout" << std::endl;
                }
                break;
            }
            case BMS1_FRAME1_ID: {
                std::cout << "Received BMS1 Frame 1:" << std::endl;
                setBMSData(frame, BMS1_VOLTAGE, BMS1_CURRENT, BMS1_MAXCELL, BMS1_MINCELL);
                break;
            }
            case BMS2_FRAME1_ID: {
                std::cout << "Reeived BMS2 Frame 1:" << std::endl;
                setBMSData(frame, BMS2_VOLTAGE, BMS2_CURRENT, BMS2_MAXCELL, BMS2_MINCELL);
                break;
            }
            case BMS3_FRAME1_ID: {
                std::cout << "Reeived BMS3 Frame 1:" << std::endl;
                setBMSData(frame, BMS3_VOLTAGE, BMS3_CURRENT, BMS3_MAXCELL, BMS3_MINCELL);
                break;
            }
            case BMS4_FRAME1_ID: {
                std::cout << "Reeived BMS4 Frame 1:" << std::endl;
                setBMSData(frame, BMS4_VOLTAGE, BMS4_CURRENT, BMS4_MAXCELL, BMS4_MINCELL);
                break;
            }
            case BMS5_FRAME1_ID: {
                std::cout << "Reeived BMS5 Frame 1:" << std::endl;
                setBMSData(frame, BMS5_VOLTAGE, BMS5_CURRENT, BMS5_MAXCELL, BMS5_MINCELL);
                break;
            }
            case BMS6_FRAME1_ID: {
                std::cout << "Reeived BMS6 Frame 1:" << std::endl;
                setBMSData(frame, BMS6_VOLTAGE, BMS6_CURRENT, BMS6_MAXCELL, BMS6_MINCELL);
                break;
            }
            case ACCEL_FRAME_ID: {
                std::cout << "Received Accel Frame:" << std::endl;
                NetworkTable::Value accel_x_pos;
                std::map<std::string, NetworkTable::Value> values;
                float x_pos = GET_ACCEL_X_DATA(frame.data);
                accel_x_pos.set_type(NetworkTable::Value::FLOAT);
                accel_x_pos.set_float_data(static_cast<float>(x_pos));
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        (ACCELEROMETER_X, accel_x_pos));
                std::cout << "x_pos " << x_pos << std::endl;

                NetworkTable::Value accel_y_pos;
                float y_pos = GET_ACCEL_Y_DATA(frame.data);
                accel_y_pos.set_type(NetworkTable::Value::FLOAT);
                accel_y_pos.set_float_data(static_cast<float>(y_pos));
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        (ACCELEROMETER_Y, accel_y_pos));
                std::cout << "y_pos " << y_pos << std::endl;

                NetworkTable::Value accel_z_pos;
                float z_pos = GET_ACCEL_Z_DATA(frame.data);
                accel_z_pos.set_type(NetworkTable::Value::FLOAT);
                accel_z_pos.set_float_data(static_cast<float>(z_pos));
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        (ACCELEROMETER_Z, accel_z_pos));
                std::cout << "z_pos " << z_pos << std::endl;

                try {
                    connection.SetValues(values);
                } catch (NetworkTable::NotConnectedException) {
                    std::cout << "Failed to set value" << std::endl;
                } catch (NetworkTable::TimeoutException) {
                    std::cout << "Timeout" << std::endl;
                }
                break;
            }
        }
    }

    return 0;
}
