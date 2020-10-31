// Copyright 2017 UBC Sailbot

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
int s;
NetworkTable::Connection connection;

/*
 * Set wind sensor with given id. id should be 0, 1, or 2
 */
void SetWindSensorData(int angle, int speed, const std::string &id) {
    NetworkTable::Value angle_nt;
    angle_nt.set_type(NetworkTable::Value::INT);
    angle_nt.set_int_data(static_cast<int>(angle));

    std::cout << "id: "<< id << " got wind sensor angle and speed:" << std::to_string(angle) \
        << " " << std::to_string(speed) << std::endl;
    NetworkTable::Value speed_nt;
    speed_nt.set_type(NetworkTable::Value::INT);
    speed_nt.set_int_data(static_cast<int>(speed));

    std::map<std::string, NetworkTable::Value> values;
    values.insert((std::pair<std::string, NetworkTable::Value> \
                ("wind_sensor_"+id+"/iimwv/wind_direction", angle_nt)));
    values.insert((std::pair<std::string, NetworkTable::Value> \
                ("wind_sensor_"+id+"/iimwv/wind_speed", speed_nt)));

    try {
        connection.SetValues(values);
    }
    catch (NetworkTable::NotConnectedException) {
        std::cout << "Failed to set value" << std::endl;
    }
}

void MotorCallback(NetworkTable::Node node, \
        const std::map<std::string, NetworkTable::Value> &diffs, \
        bool is_self_reply) {
    struct can_frame frame;
    float angle = static_cast<float>(node.value().float_data());
    frame.can_id = 0xAB;
    frame.can_dlc = 8;

    // Manually put split the float into bytes, and
    // put each byte into the frame.data array
    uint8_t const *angle_array = reinterpret_cast<uint8_t *>(&angle);
    frame.data[0] = angle_array[0];
    frame.data[1] = angle_array[1];
    frame.data[2] = angle_array[2];
    frame.data[3] = angle_array[3];
    std::cout << "Sending angle:" << angle << std::endl;
    if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write");
        return;
    }
}

int main(int argc, char **argv) {
    if (argc != 2) {
        printf("Please provide the name of the canbus interface. \n");
        printf("Example usage: './bbb_canbus_listener vcan0' \n");
        return 0;
    }

    // Connect to the network table
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

    // subscribe to network-table
    try {
        connection.Subscribe("actuation_angle/winch", &MotorCallback);
    } catch (NetworkTable::NotConnectedException) {
        std::cout << "Failed to subscribe to actuation_angle" << std::endl;
        // TODO: fix it.
    }

    // Keep on reading the wind sensor data off canbus, and
    // placing the latest data in the network table.
    while (true) {
        read(s, &frame, sizeof(struct can_frame));
        std::cout << "Can ID = " << std::hex << frame.can_id << std::endl << std::dec;
        switch (frame.can_id) {
            case WIND_SENS0_FRAME_ID : {
                int angle = GET_WIND_ANGLE(frame.data);
                int speed = GET_WIND_SPEED(frame.data);

                SetWindSensorData(angle, speed, "0");
                break;
            }
            case WIND_SENS1_FRAME_ID : {
                int angle = GET_WIND_ANGLE(frame.data);
                int speed = GET_WIND_SPEED(frame.data);
                SetWindSensorData(angle, speed, "1");
                break;
            }
            case WIND_SENS2_FRAME_ID : {
                int angle = GET_WIND_ANGLE(frame.data);
                int speed = GET_WIND_SPEED(frame.data);
                SetWindSensorData(angle, speed, "2");
                break;
            }
            case SAILENCODER_FRAME_ID : {
                std::map<std::string, NetworkTable::Value> values;
                NetworkTable::Value boom_angle;
                int angle = GET_SAILENCODER_ANGLE(frame.data);
                boom_angle.set_type(NetworkTable::Value::INT);
                boom_angle.set_int_data(static_cast<int>(angle));
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        ("boom_angle_sensor/sensor_data/angle", boom_angle));
                try {
                    connection.SetValues(values);
                }
                catch (NetworkTable::NotConnectedException) {
                    std::cout << "Failed to set value" << std::endl;
                }
                

                std::cout << "sailencoder value: " << std::dec << angle << std::dec << std::endl;
                break;
            }
            case GPS_LONG_FRAME_ID : {
                std::cout << "Received GPS Long Frame" << std::endl;

                std::map<std::string, NetworkTable::Value> values;
                NetworkTable::Value gps_longitude;
                double longitude = GET_GPS_LONG(frame.data);
                gps_longitude.set_type(NetworkTable::Value::INT);
                gps_longitude.set_int_data(static_cast<int>(longitude));
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        ("gps/gprmc/longitude", gps_longitude));
                try {
                    connection.SetValues(values);
                }
                catch (NetworkTable::NotConnectedException) {
                    std::cout << "Failed to set value" << std::endl;
                }

                std::cout << "longitude = " << longitude << " " << std::endl;
                break;
            }
            case GPS_LAT_FRAME_ID : {
                std::cout << "Received GPS Lat Frame" << std::endl;

                std::map<std::string, NetworkTable::Value> values;
                NetworkTable::Value gps_latitude;
                double latitude = GET_GPS_LAT(frame.data);
                gps_latitude.set_type(NetworkTable::Value::INT);
                gps_latitude.set_int_data(static_cast<int>(latitude));
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        ("gps/gprmc/latitude", gps_latitude));
                try {
                    connection.SetValues(values);
                }
                catch (NetworkTable::NotConnectedException) {
                    std::cout << "Failed to set value" << std::endl;
                }

                std::cout << "latitude = " << latitude << " " << std::endl;
                break;
            }
            case GPS_OTHER_FRAME_ID : {
                std::cout << "Received GPS Other Frame" << std::endl;

                std::map<std::string, NetworkTable::Value> values;
                NetworkTable::Value gps_gndSpeed;
                float gndSpeed = GET_GPS_GND_SPEED(frame.data);
                gps_gndSpeed.set_type(NetworkTable::Value::INT);
                gps_gndSpeed.set_int_data(static_cast<int>(gndSpeed));
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        ("gps/gprmc/gndSpeed", gps_gndSpeed));
                std::cout << "gnd speed = " << gndSpeed << " " << std::endl;

                NetworkTable::Value gps_magVar;
                float magVar = GET_GPS_MAG_VAR(frame.data);
                gps_magVar.set_type(NetworkTable::Value::INT);
                gps_magVar.set_int_data(static_cast<int>(magVar));
                values.insert((std::pair<std::string, NetworkTable::Value>\
                        ("gps/gprmc/magVar", gps_magVar)));
                std::cout << "mag var =  " << magVar << " " << std::endl;

                NetworkTable::Value gps_TMG;
                float gpsTMG = GET_GPS_TMG(frame.data);
                gps_TMG.set_type(NetworkTable::Value::INT);
                gps_TMG.set_int_data(static_cast<int>(gpsTMG));
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        ("gps/gprmc/TMG", gps_TMG));
                std::cout << "gps tmg =  " << gpsTMG << " " << std::endl;

                try {
                    connection.SetValues(values);
                }
                catch (NetworkTable::NotConnectedException) {
                    std::cout << "Failed to set value" << std::endl;
                }
                break;
            }
            case GPS_DATE_FRAME_ID : {
                std::cout << "Received GPS Date Frame" << std::endl;

                std::map<std::string, NetworkTable::Value> values;
                NetworkTable::Value gps_date_hour;
                int hour = GET_HOUR(frame.data);
                gps_date_hour.set_type(NetworkTable::Value::INT);
                gps_date_hour.set_int_data(hour);
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        ("gps/gps_date/hour", gps_date_hour));

                NetworkTable::Value gps_date_minute;
                int minute = GET_MINUTE(frame.data);
                gps_date_minute.set_type(NetworkTable::Value::INT);
                gps_date_hour.set_int_data(minute);
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        ("gps/gps_date/minute", gps_date_minute));

                NetworkTable::Value gps_date_second;
                int second = GET_SECOND(frame.data);
                gps_date_second.set_type(NetworkTable::Value::INT);
                gps_date_second.set_int_data(second);
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        ("gps/gps_date/second", gps_date_second));

                NetworkTable::Value gps_date_day;
                int day = GET_DAY(frame.data);
                gps_date_day.set_type(NetworkTable::Value::INT);
                gps_date_day.set_int_data(day);
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        ("gps/gps_date/day", gps_date_day));

                NetworkTable::Value gps_date_month;
                int month = GET_MONTH(frame.data);
                gps_date_month.set_type(NetworkTable::Value::INT);
                gps_date_month.set_int_data(month);
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        ("gps/gps_date/day", gps_date_day));

                NetworkTable::Value gps_date_year;
                int year = GET_YEAR(frame.data);
                gps_date_year.set_type(NetworkTable::Value::INT);
                gps_date_year.set_int_data(year);
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        ("gps/gps_date/year", gps_date_year));

                NetworkTable::Value gps_date_status;
                bool status = GET_STATUS(frame.data);
                gps_date_status.set_type(NetworkTable::Value::BOOL);
                gps_date_status.set_bool_data(status);
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        ("gps/gps_date/status", gps_date_status));

                NetworkTable::Value gps_date_varWest;
                bool var_west = GET_VAR_WEST(frame.data);
                gps_date_varWest.set_type(NetworkTable::Value::BOOL);
                gps_date_varWest.set_bool_data(var_west);
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        ("gps/gps_date/var_west", gps_date_varWest));

                NetworkTable::Value gps_date_varNorth;
                bool var_north = GET_VAR_NORTH(frame.data);
                gps_date_varNorth.set_type(NetworkTable::Value::BOOL);
                gps_date_varWest.set_bool_data(var_north);
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        ("gps/gps_date/var_north", gps_date_varNorth));

                NetworkTable::Value gps_date_varLongWest;
                bool var_long_west = GET_LONG_WEST(frame.data);
                gps_date_varLongWest.set_type(NetworkTable::Value::BOOL);
                gps_date_varLongWest.set_bool_data(var_long_west);
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        ("gps/gps_date/long_west", gps_date_varLongWest));

                try {
                    connection.SetValues(values);
                }
                catch (NetworkTable::NotConnectedException) {
                    std::cout << "Failed to set value" << std::endl;
                }
                break;
            }
            case BMS_FRAME_ID_1: {
                std::cout << "bms 1:" << std::endl;

                std::map<std::string, NetworkTable::Value> values;
                NetworkTable::Value bms_volt_data;
                uint16_t volt_data = GET_BMS_VOLT_DATA(frame.data);
                bms_volt_data.set_type(NetworkTable::Value::INT);
                bms_volt_data.set_int_data(volt_data);
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        ("bms/uccm/voltage", bms_volt_data));

                std::cout << "volt_data:" << volt_data << std::endl;
                NetworkTable::Value bms_curr_data;
                uint16_t curr_data = GET_BMS_CURR_DATA(frame.data);
                bms_curr_data.set_type(NetworkTable::Value::INT);
                bms_curr_data.set_int_data(curr_data);
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        ("bms/uccm/current", bms_curr_data));
                std::cout << "curr_data:" << curr_data << std::endl;

                NetworkTable::Value bms_maxcell_data;
                uint16_t maxcell_data = GET_BMS_MAXCELL_DATA(frame.data);
                bms_maxcell_data.set_type(NetworkTable::Value::INT);
                bms_maxcell_data.set_int_data(maxcell_data);
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        ("bms/uccm/maxcell", bms_maxcell_data));
                std::cout << "maxcell_data:" << maxcell_data << std::endl;

                NetworkTable::Value bms_mincell_data;
                uint16_t mincell_data = GET_BMS_MINCELL_DATA(frame.data);
                bms_mincell_data.set_type(NetworkTable::Value::INT);
                bms_mincell_data.set_int_data(mincell_data);
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        ("bms/uccm/mincell", bms_mincell_data));
                std::cout << "mincell_data:" << mincell_data << std::endl;

                try {
                    connection.SetValues(values);
                }
                catch (NetworkTable::NotConnectedException) {
                    std::cout << "Failed to set value" << std::endl;
                }
                break;
            }
            case ACCEL_FRAME_ID: {
                NetworkTable::Value accel_x_pos;
                std::map<std::string, NetworkTable::Value> values;
                int16_t x_pos = GET_ACCEL_X_DATA(frame.data);
                accel_x_pos.set_type(NetworkTable::Value::INT);
                accel_x_pos.set_int_data(static_cast<int>(x_pos));
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        ("accelerometer/boat_orientation_data/x_axis_acceleration", accel_x_pos));
                std::cout << "x_pos " << x_pos << std::endl;

                NetworkTable::Value accel_y_pos;
                int16_t y_pos = GET_ACCEL_Y_DATA(frame.data);
                accel_y_pos.set_type(NetworkTable::Value::INT);
                accel_y_pos.set_int_data(static_cast<int>(y_pos));
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        ("accelerometer/boat_orientation_data/y_axis_acceleration", accel_y_pos));
                std::cout << "y_pos " << y_pos << std::endl;

                NetworkTable::Value accel_z_pos;
                int16_t z_pos = GET_ACCEL_Z_DATA(frame.data);
                accel_z_pos.set_type(NetworkTable::Value::INT);
                accel_z_pos.set_int_data(static_cast<int>(z_pos));
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        ("accelerometer/boat_orientation_data/z_axis_acceleration", accel_z_pos));
                std::cout << "z_pos " << z_pos << std::endl;

                try {
                    connection.SetValues(values);
                }
                catch (NetworkTable::NotConnectedException) {
                    std::cout << "Failed to set value" << std::endl;
                }
                break;
            }
        }
    }

    return 0;
}
