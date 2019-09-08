#include <stdio.h>
#include <map>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <stdint.h>
#include "Connection.h"
#include "Value.pb.h"

int
main(void)
{
    // Connect to the network table
    NetworkTable::Connection connection;
    connection.SetTimeout(100);
    try {
        connection.Connect();
    } catch (NetworkTable::TimeoutException) {
        std::cout << "Failed to connect to server" << std::endl;
        return 0;
    }

    // Connect to the canbus network.
    // It should show up as a network interface.
    // You should see it with the ifconfig command.
	int s;
	int nbytes;
	struct sockaddr_can addr;
	struct can_frame frame;
	struct ifreq ifr;

	const char *ifname = "can0";

	if((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        std::cout << "Error while opening socket";
		return -1;
	}

	strcpy(ifr.ifr_name, ifname);
	ioctl(s, SIOCGIFINDEX, &ifr);
	
	addr.can_family  = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

    frame.can_id = 2;

	if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        std::cout << "Error in socket bind";
		return -2;
	}

    // Keep on reading the wind sensor data off canbus, and
    // placing the latest data in the network table.
    while (true){
        nbytes = read(s, &frame, sizeof(struct can_frame));

        uint32_t angle;
        angle = frame.data[0];
        angle += frame.data[1] << 8;
        angle += frame.data[2] << 16;
        angle += frame.data[3] << 24;
        
        uint32_t speed;
        speed = frame.data[4];
        speed += frame.data[5] << 8;
        speed += frame.data[6] << 16;
        speed += frame.data[7] << 24;
        float speed_decimal = (float) speed/10.00;


        NetworkTable::Value angle_nt ;
        angle_nt.set_type(NetworkTable::Value::INT);
        angle_nt.set_int_data(static_cast<int>(angle));

        NetworkTable::Value speed_nt ;
        speed_nt.set_type(NetworkTable::Value::INT);
        speed_nt.set_int_data(static_cast<int>(speed));

        std::map<std::string, NetworkTable::Value> values;
        values.insert((std::pair<std::string, NetworkTable::Value>\
                    ("wind_sensor/iimwv/wind_direction",angle_nt)));
        values.insert((std::pair<std::string, NetworkTable::Value>\
                    ("wind_sensor/iimwv/wind_speed",speed_nt)));

        connection.SetValues(values);

        std::cout << "1. Angle:" << angle << " degrees  |  Speed:"\
            << speed_decimal << " knots" << std::endl;
    }

	return 0;
}
