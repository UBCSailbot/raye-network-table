#include "ros/ros.h"

#include "Connection.h"
#include "Help.h"
#include "nuc_to_bbb/actuation_angle.h"

ros::Subscriber nt_sub;
NetworkTable::Connection connection;

void ActuationCallBack(const nuc_to_bbb::actuation_angle actuation_angle) {
    if (ros::ok()) {
        // Both angles are in radians (CONFIRM WITH BRUCE)
        NetworkTable::Value rudder_angle;
        NetworkTable::Value winch_angle;

        rudder_angle.set_type(NetworkTable::Value::FLOAT);
        winch_angle.set_type(NetworkTable::Value::FLOAT);

        rudder_angle.set_float_data(actuation_angle.rudder);
        winch_angle.set_float_data(actuation_angle.winch);

        connection.SetValue("rudder_motor_control", rudder_angle);
        connection.SetValue("winch_motor_control", winch_angle);
    }
    else {
        std::cout << "Failed to receive actuation angle" << std::endl;    
    }
}

int main(int argc, char** argv) {
    // TODO: send over ethernet cable
    connection.SetTimeout(1000);
    
    try {
        connection.Connect();
    }   catch (NetworkTable::TimeoutException) {
        std::cout << "Connection to server timed out" << std::endl;
        return 0;
    }

    ros::init(argc, argv, "nuc_to_bbb"); 
    ros::NodeHandle n;
    nt_sub = n.subscribe("actuation_angle", 1000, ActuationCallBack);

    while(true) {
        std::this_thread::sleep_for(std::chrono::hours(1000));
    }
}
