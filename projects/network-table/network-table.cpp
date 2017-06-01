// Copyright 2017 UBC Sailbot

#include <boost/algorithm/string.hpp>
#include <iostream>
#include <SimpleAmqpClient/SimpleAmqpClient.h>

#include "network-table.h"

using AmqpClient::BasicMessage;
using AmqpClient::Channel;
using AmqpClient::Envelope;

void NetworkTable::Run() {
    Channel::ptr_t channel = Channel::Create("localhost");
    channel->DeclareQueue("alex", false, false, false);

    std::string consumer_tag = channel->BasicConsume("alex");
    Envelope::ptr_t envelope;

    std::cout << "Begin receiving messages..." << std::endl;
    while (true) {
        // Receive any new messages:
        envelope = channel->BasicConsumeMessage(consumer_tag);
        std::string message = envelope->Message()->Body();
        std::cout << "Received message: " << message << std::endl;

        // Split the message into components:
        std::vector<std::string> message_parts;
        boost::split(message_parts, message, boost::is_any_of(":"));

        if (message_parts.size() == 2) {
            std::string action = message_parts.at(0);
            std::string key = message_parts.at(1);

            if (action.compare("GET") == 0) {
                std::string correlation_id = envelope->Message()->CorrelationId();
                std::string reply_to = envelope->Message()->ReplyTo();

                std::string reply_body = table_[key];
                BasicMessage::ptr_t reply_message = BasicMessage::Create(reply_body);
                reply_message->CorrelationId(correlation_id);

                channel->BasicPublish("", reply_to, reply_message);
            }
        } else if (message_parts.size() == 3) {
            std::string action = message_parts.at(0);
            std::string key = message_parts.at(1);
            std::string value = message_parts.at(2);

            if (action.compare("SET") == 0) {
                table_[key] = value;
            }
        } else {
            std::cout << "Ignoring invalid message: " << message << std::endl;
        }
    }
}
