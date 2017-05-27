// Copyright 2017 UBC Sailbot

#include <SimpleAmqpClient/SimpleAmqpClient.h>

int main(){
    AmqpClient::Channel::ptr_t connection = AmqpClient::Channel::Create("localhost");
}
