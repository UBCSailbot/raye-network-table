// Copyright 2017 UBC Sailbot

#include "NonProtoConnection.h"

#include <stdio.h>
#include <string.h>


////////////////////// PUBLIC //////////////////////

NetworkTable::NonProtoConnection::NonProtoConnection() {
}

void NetworkTable::NonProtoConnection::SetIntValue(const std::string &uri, const int &value) {
    std::map<std::string, int> values = {{uri, value}};
    SetIntValues(values);
}

void NetworkTable::NonProtoConnection::SetIntValues(const std::map<std::string, int> &values) {
    std::map<std::string, NetworkTable::Value> proto_values;
    for (auto const &entry : values) {
        std::string uri = entry.first;
        NetworkTable::Value int_val;
        int_val.set_type(NetworkTable::Value::INT);
        int_val.set_int_data(entry.second);
        proto_values[uri] = int_val;
    }

    SetValues(proto_values);
}

void NetworkTable::NonProtoConnection::SetFloatValue(const std::string &uri, const float &value) {
    std::map<std::string, float> values = {{uri, value}};
    SetFloatValues(values);
}

void NetworkTable::NonProtoConnection::SetFloatValues(const std::map<std::string, float> &values) {
    std::map<std::string, NetworkTable::Value> proto_values;
    for (auto const &entry : values) {
        std::string uri = entry.first;
        NetworkTable::Value float_val;
        float_val.set_type(NetworkTable::Value::FLOAT);
        float_val.set_float_data(entry.second);
        proto_values[uri] = float_val;
    }

    SetValues(proto_values);
}

void NetworkTable::NonProtoConnection::SetStringValue(const std::string &uri, const std::string &value) {
    std::map<std::string, std::string> values = {{uri, value}};
    SetStringValues(values);
}

void NetworkTable::NonProtoConnection::SetStringValues(const std::map<std::string, std::string> &values) {
    std::map<std::string, NetworkTable::Value> proto_values;
    for (auto const &entry : values) {
        std::string uri = entry.first;
        NetworkTable::Value string_val;
        string_val.set_type(NetworkTable::Value::STRING);
        string_val.set_string_data(entry.second);
        proto_values[uri] = string_val;
    }

    SetValues(proto_values);
}

void NetworkTable::NonProtoConnection::SetBooleanValue(const std::string &uri, const bool &value) {
    std::map<std::string, bool> values = {{uri, value}};
    SetBooleanValues(values);
}

void NetworkTable::NonProtoConnection::SetBooleanValues(const std::map<std::string, bool> &values) {
    std::map<std::string, NetworkTable::Value> proto_values;
    for (auto const &entry : values) {
        std::string uri = entry.first;
        NetworkTable::Value bool_val;
        bool_val.set_type(NetworkTable::Value::BOOL);
        bool_val.set_bool_data(entry.second);
        proto_values[uri] = bool_val;
    }

    SetValues(proto_values);
}

void NetworkTable::NonProtoConnection::SetWaypointValue(const std::pair<double, double> &value) {
    std::list<std::pair<double, double>> coordinates;
    coordinates.push_back(value);

    SetWaypointValues(coordinates);
}

void NetworkTable::NonProtoConnection::SetWaypointValues(std::list<std::pair<double, double>> &values) {
    std::map<std::string, NetworkTable::Value> proto_values;
    std::string uri = "waypoints";
    NetworkTable::Value waypoints_val;
    waypoints_val.set_type(NetworkTable::Value::WAYPOINTS);

    for (auto const &coordinates : values) {
        NetworkTable::Value_Waypoint *waypoint = waypoints_val.add_waypoints();
        waypoint->set_latitude(coordinates.first);
        waypoint->set_longitude(coordinates.second);
    }
    proto_values[uri] = waypoints_val;

    SetValues(proto_values);
}

std::list<std::pair<double, double>> NetworkTable::NonProtoConnection::GetCurrentWaypoints() {
    NetworkTable::Value waypoints_val;
    const std::string uri = "waypoints";
    std::list<std::pair<double, double>> coordinates;

    waypoints_val = GetValue(uri);

    for (auto const &coord : waypoints_val.waypoints()) {
        std::pair<double, double> curr_coord;
        curr_coord.first = coord.latitude();
        curr_coord.second = coord.longitude();
        coordinates.push_back(curr_coord);
    }

    return coordinates;
}
