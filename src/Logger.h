// Copyright 2021 UBC Sailbot

#ifndef LOGGER_H_
#define LOGGER_H_

#define BOOST_LOG_DYN_LINK 1

// Boost includes
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sources/logger.hpp>
#include <boost/log/support/date_time.hpp>
#include <boost/log/sinks/sync_frontend.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/sinks/text_ostream_backend.hpp>
#include <boost/log/attributes/named_scope.hpp>


#include <string>
#include <iostream>
#include <memory>

#include "Exceptions.h"
#include "Connection.h"

namespace NetworkTable {
class Logger {
    public:
    // Constructor
    Logger();
    Logger(std::string);

    /*
        * Different levels of logging severity
        * 
        * @param msg - error message passed in by caller
    */
    void Trace(std::string msg); // Trace log
    void Debug(std::string msg); // Debug log
    void Info(std::string msg); // Info log
    void Warning(std::string msg); // Warning log
    void Error(std::string msg); // Error log
    void Fatal(std::string msg); // Fatal log

    enum Severity {
        TRACE,
        DEBUG,
        INFO,
        WARNING,
        ERROR,
        FATAL
    };

    /*
        * Setter for Network Table connection
        * @param connection 
    */
    void SetNTConn(std::shared_ptr<NetworkTable::Connection> conn_ptr);

    /*
        * Setter for file path
    */
    void SetFilePath(std::string);

    private:
    // Init function to set up boost log paramters
    void Init(std::string);

    /*
    * Generates a protobuf error object
    */
    //NetworkTable::Value GenPBErr(std::string msg, int severity);
    void GenPBErr(std::string msg, Logger::Severity);

    //NetworkTable::Connection nt_conn_;
    std::shared_ptr<NetworkTable::Connection> conn_ptr_;
    std::string file_path_;
    bool nt_logging_;

};
}

#include <Exceptions.h>
#include <Connection.h>

namespace NetworkTable {
    class Logger {
    public:
        // Constructor
        Logger(bool text_logging);

        /*
         * Different levels of logging severity
         * 
         * @param msg - error message passed in by caller
        */
        void Trace(std::string msg); // Trace log
        void Debug(std::string msg); // Debug log
        void Info(std::string msg); // Info log
        void Warning(std::string msg); // Warning log
        void Error(std::string msg); // Error log
        void Fatal(std::string msg); // Fatal log

        /*
         * Setter for Network Table connection
         * 
         * @param connection 
        */
        void SetNTConn(NetworkTable::Connection conn);

    private:
        // Init function to set up boost log paramters
        void Init();

        /*
        * Generates a protobuf error object
        */
        NetworkTable::Value GenPBErr(std::string msg, int severity);

        NetworkTable::Connection nt_conn;
        std::string file_path;

    };
}


#endif  // LOGGER_H_
