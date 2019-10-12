// Copyright 2017 UBC Sailbot

#include <boost/asio.hpp>
#include <iostream>
#include "Connection.h"
#include <thread>
#include <bitset>
#include <cstring>
#include <iomanip>
#include <boost/algorithm/string.hpp>

void receive_message(std::string&, std::string);

std::string readLine(boost::asio::serial_port &p)
{
    //Reading data char by char, code is optimized for simplicity, not speed
    using namespace boost;
    char c;
    std::string result;
    for(;;)
    {
        asio::read(p,asio::buffer(&c,1));
        switch(c)
        {
            case '\r':
                break;
            case '\n':
                return result;
            default:
                result+=c;
        }
    }
}


boost::asio::io_service io;
boost::asio::serial_port serial(io,"/dev/ttyO4");

void send(const std::string &data) {
//    std::string message_length("AT+SBDWB=" + std::string(data.size()) + "\r");
    std::string message_length("AT+SBDWB="+std::to_string(data.size())+"\r");
    boost::asio::write(serial,boost::asio::buffer(message_length.c_str(),message_length.size()));
	
    std::cout << readLine(serial)<<std::endl;
	std::cout << readLine(serial)<<std::endl;
    
    auto data_c_str = data.c_str();
    int sum = 0;
    for (int i = 0; i < data.length(); i++) {
        sum += data_c_str[i];
    }

    uint16_t check_sum_low = sum & 0x00FF;
    uint16_t check_sum_high = (sum & 0xFF00) >> 8;

    std::string message = data;
    message += check_sum_high;
    message += check_sum_low;
    message += '\r';

    boost::asio::write(serial,boost::asio::buffer(message,message.size()));
    
    std::cout << readLine(serial)<<std::endl;
	std::cout << readLine(serial)<<std::endl;
	std::cout << readLine(serial)<<std::endl;

    std::string send_command = "AT+SBDIX\r";

    boost::asio::write(serial,boost::asio::buffer(send_command,send_command.size()));

	std::cout << readLine(serial)<<std::endl;
	std::cout << readLine(serial)<<std::endl;
	std::cout << readLine(serial)<<std::endl;
}

void receive(){
    std::string sbdInit("AT+SBDIX\r");
    boost::asio::write(serial,boost::asio::buffer(sbdInit.c_str(),sbdInit.size()));

    std::cout << readLine(serial)<<std::endl;
    std::string response(readLine(serial));
    
    std::vector<std::string> response_split;
    boost::split(response_split,response,boost::is_any_of(","));
    
    std::vector<std::string> trimmed_response;

    for (int i = 0; i < response_split.size(); i++) {
        boost::algorithm::trim(response_split[i]);
    }
    
    std::string status(response_split[2]);

    std::string received_message;
    receive_message(received_message, status);
    std::cout << received_message <<std::endl;
}

void receive_message(std::string& response, std::string status){
    if (status == "1") {
        std::string msg = "AT+SBDRT\r";
	    readLine(serial);
        response = readLine(serial);
    }
    if (status == "2") {
        response = "Error checking mailbox";
    }
    if (status == "0") {
        std::string null_msg = "Mailbox empty";
    }
}


int main() {
    serial.set_option(boost::asio::serial_port_base::baud_rate(19200));
    boost::asio::write(serial,boost::asio::buffer("AT\r",3));
	std::cout << readLine(serial)<<std::endl;
	std::cout << readLine(serial)<<std::endl;

    boost::asio::write(serial,boost::asio::buffer("AT&K0\r",6));
	std::cout << readLine(serial)<<std::endl;
	std::cout << readLine(serial)<<std::endl;
    
   //send("hello");
    receive();
}
