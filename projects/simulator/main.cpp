#include "ros/ros.h"
#include "simulator/Sensors.h"

#include "Connection.h"
#include "Help.h"

ros::Publisher nt_pub;

void RootCallback(NetworkTable::Node node, \
        std::map<std::string, NetworkTable::Value> diffs, \
        bool is_self_reply) {
    if (ros::ok()) {
        simulator::Sensors sensors;
        NetworkTable::Sensors proto_sensors = NetworkTable::RootToSensors(&node);
        sensors.boom_angle_sensor_angle = proto_sensors.boom_angle_sensor().sensor_data().angle();

        sensors.wind_sensor_0_speed = proto_sensors.wind_sensor_0().iimwv().wind_speed();
        sensors.wind_sensor_0_direction = proto_sensors.wind_sensor_0().iimwv().wind_direction();
        sensors.wind_sensor_0_reference = proto_sensors.wind_sensor_0().iimwv().wind_reference();
        sensors.wind_sensor_1_speed = proto_sensors.wind_sensor_1().iimwv().wind_speed();
        sensors.wind_sensor_1_direction = proto_sensors.wind_sensor_1().iimwv().wind_direction();
        sensors.wind_sensor_1_reference = proto_sensors.wind_sensor_1().iimwv().wind_reference();
        sensors.wind_sensor_2_speed = proto_sensors.wind_sensor_2().iimwv().wind_speed();
        sensors.wind_sensor_2_direction = proto_sensors.wind_sensor_2().iimwv().wind_direction();
        sensors.wind_sensor_2_reference = proto_sensors.wind_sensor_2().iimwv().wind_reference();

        sensors.gps_0_timestamp = proto_sensors.gps_0().gprmc().utc_timestamp();
        sensors.gps_0_latitude = proto_sensors.gps_0().gprmc().latitude();
        sensors.gps_0_longitude = proto_sensors.gps_0().gprmc().longitude();
        sensors.gps_0_latitude_loc = proto_sensors.gps_0().gprmc().latitude_loc();
        sensors.gps_0_longitude_loc = proto_sensors.gps_0().gprmc().longitude_loc();
        sensors.gps_0_groundspeed = proto_sensors.gps_0().gprmc().ground_speed();
        sensors.gps_0_track_made_good = proto_sensors.gps_0().gprmc().track_made_good();
        sensors.gps_0_magnetic_variation = proto_sensors.gps_0().gprmc().magnetic_variation();
        sensors.gps_0_magnetic_variation_sense = proto_sensors.gps_0().gprmc().magnetic_variation_sense();
        sensors.gps_1_timestamp = proto_sensors.gps_1().gprmc().utc_timestamp();
        sensors.gps_1_latitude = proto_sensors.gps_1().gprmc().latitude();
        sensors.gps_1_longitude = proto_sensors.gps_1().gprmc().longitude();
        sensors.gps_1_latitude_loc = proto_sensors.gps_1().gprmc().latitude_loc();
        sensors.gps_1_longitude_loc = proto_sensors.gps_1().gprmc().longitude_loc();
        sensors.gps_1_groundspeed = proto_sensors.gps_1().gprmc().ground_speed();
        sensors.gps_1_track_made_good = proto_sensors.gps_1().gprmc().track_made_good();
        sensors.gps_1_magnetic_variation = proto_sensors.gps_1().gprmc().magnetic_variation();
        sensors.gps_1_magnetic_variation_sense = proto_sensors.gps_1().gprmc().magnetic_variation_sense();

        sensors.accelerometer_x_axis_acceleration = \
            proto_sensors.accelerometer().boat_orientation_data().x_axis_acceleration();
        sensors.accelerometer_y_axis_acceleration = \
            proto_sensors.accelerometer().boat_orientation_data().y_axis_acceleration();
        sensors.accelerometer_z_axis_acceleration = \
            proto_sensors.accelerometer().boat_orientation_data().z_axis_acceleration();

        nt_pub.publish(sensors);
    } else {
        std::cout << "Failed to send ros message" << std::endl;
    }
}

int main(int argc, char **argv) {
    NetworkTable::Connection connection;
    connection.SetTimeout(1000);
    try {
        connection.Connect();
    } catch (NetworkTable::TimeoutException) {
        std::cout << "Connection to server timed out" << std::endl;
        return 0;
    }

    ros::init(argc, argv, "nt");
    ros::NodeHandle n;
    nt_pub = n.advertise<simulator::Sensors>("nt", 1000);

    connection.Subscribe("/", &RootCallback);

    // Sleep forever
    while(true) {
        std::this_thread::sleep_for(std::chrono::hours(1000));
    }
}
