// Copyright 2017 UBC Sailbot

#include <amqp.h>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>  
#include <SimpleAmqpClient/SimpleAmqpClient.h>
#include <iostream>

using namespace AmqpClient;

int main() {
    Channel::ptr_t channel = Channel::Create("localhost");
    std::string queue = channel->DeclareQueue("alex", false, false, false);

    /*
     * Send a message to update the table with new data.
     */
    {
        BasicMessage::ptr_t message = BasicMessage::Create("SET:/wind/speed/:111");
        channel->BasicPublish("", queue, message);
    }

    /*
     * Send a message to get a the value of a table entry.
     */
    {
        /*
         * Send a request to get the wind speed.
         */
        std::string callback_queue = channel->DeclareQueue("");

        BasicMessage::ptr_t message = BasicMessage::Create("GET:/wind/speed/");
        message->ReplyTo(callback_queue);

        std::string correlation_id = boost::uuids::to_string(boost::uuids::random_generator()());
        message->CorrelationId(correlation_id);

        channel->BasicPublish("", queue, message);

        /*
         * Get the reply.
         */
        std::string consumer_tag = channel->BasicConsume(callback_queue);
        Envelope::ptr_t envelope = channel->BasicConsumeMessage(consumer_tag);
        std::string reply_correlation_id = envelope->Message()->CorrelationId();
        if (correlation_id == reply_correlation_id) {
            std::cout << "Wind speed is " << envelope->Message()->Body() << std::endl;
        }
   }
} 
