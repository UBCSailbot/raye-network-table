// Copyright 2021 UBC Sailbot

#include "Logger.h"

NetworkTable::Logger::Logger() {
    nt_logging_ = true;
    Logger::Init();
}

NetworkTable::Logger::Logger(std::string file_path) {
    //nt_logging_ = nt_logging;
    nt_logging_ = true;
    Logger::Init(file_path);
}

void NetworkTable::Logger::Init(std::string file_path="../logs/temp.log") {
    boost::log::add_common_attributes();

    auto timeFmt = 
      boost::log::expressions::format_date_time<boost::posix_time::ptime>("TimeStamp", "%Y-%m-%d %H:%M");

    auto fmtSeverity = boost::log::expressions::
            attr<boost::log::trivial::severity_level>("Severity");

    boost::log::formatter logFmt =
        boost::log::expressions::format("[%1%] (%2%) %3%")
        % timeFmt % fmtSeverity
        % boost::log::expressions::smessage;

    auto fsSink = boost::log::add_file_log(
            boost::log::keywords::file_name = file_path,
            boost::log::keywords::scan_method = boost::log::sinks::file::scan_matching);

    fsSink->set_formatter(logFmt);
                
    boost::log::core::get()->set_filter(boost::log::trivial::severity >= boost::log::trivial::trace);
}

void NetworkTable::Logger::SetNTConn(std::shared_ptr<NetworkTable::Connection> conn) {
    conn_ptr_ = conn;
}

void NetworkTable::Logger::SetFilePath(std::string path) {
    file_path_ = path;
}

void NetworkTable::Logger::Trace(std::string msg) {
    GenPBErr(msg, Logger::TRACE);
}

void NetworkTable::Logger::Debug(std::string msg) {
    GenPBErr(msg, Logger::DEBUG);
}

void NetworkTable::Logger::Info(std::string msg) {
    GenPBErr(msg, Logger::INFO);
}

void NetworkTable::Logger::Warning(std::string msg) {
    GenPBErr(msg, Logger::WARNING);
}

void NetworkTable::Logger::Error(std::string msg) {
    GenPBErr(msg, Logger::ERROR);
}

void NetworkTable::Logger::Fatal(std::string msg) {
    GenPBErr(msg, Logger::FATAL);
}

void NetworkTable::Logger::GenPBErr(std::string msg, Logger::Severity severity) {
    std::cout << msg << " " << severity << std::endl;
    switch(severity) {
        case NetworkTable::Logger::TRACE:
        BOOST_LOG_TRIVIAL(trace) << msg;
        break;

        case NetworkTable::Logger::DEBUG:
        BOOST_LOG_TRIVIAL(debug) << msg;
        break;

        case NetworkTable::Logger::INFO:
        BOOST_LOG_TRIVIAL(info) << msg;
        break;

        case NetworkTable::Logger::WARNING:
        BOOST_LOG_TRIVIAL(warning) << msg;
        break;

        case NetworkTable::Logger::ERROR:
        BOOST_LOG_TRIVIAL(error) << msg;
        break;

        case NetworkTable::Logger::FATAL:
        BOOST_LOG_TRIVIAL(fatal) << msg;
        break;
    }
}
