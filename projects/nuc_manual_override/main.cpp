#include <thread>
#include <iostream>

#include <ros/ros.h>
#include "Help.h"
#include "ActuationAngle.pb.h"
#include "Node.pb.h"
#include "sailbot_msg/actuation_angle.h"
#include "Controller.pb.h"
#include "Uri.h"

ros::publisher manual_override_pub;

void PublishManualOverride() {
  while(ros::ok()) {
    double rudder_angle_degrees, abs_sail_angle_degrees;

    std::cout << "Manually enter rudder angle and sail angle (degrees):" << std::endl;

    std::cin >> rudder_angle_degrees;
    std::cin >> abs_sail_angle_degrees;

    manual_override_pub.publish(rudder_angle_degrees, abs_sail_angle_degrees);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ros_manual_override_publisher");

  ros::NodeHandle n;

  manual_override_pub = n.advertise<sailbot_msg::actuation_angle>("manual override", 100);

  std::thread publish_manual_override(PublishManualOverride);

  ros::spin();

  publish_manual_override.join();
}
