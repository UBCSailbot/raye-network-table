// Copyright 2017 UBC Sailbot

#ifndef EXCEPTIONS_H_
#define EXCEPTIONS_H_

#include <string>
#include <stdexcept>

namespace NetworkTable {

class ConnectionException : public std::runtime_error {
 public:
     explicit ConnectionException(const std::string &what) : std::runtime_error(what.c_str()) { };
};

class NotConnectedException : public std::runtime_error {
public:
    explicit NodeNotFoundException(const std::string &what) : std::runtime_error(what.c_str()) { };
};

}  // namespace NetworkTable

#endif  // EXCEPTIONS_H_
