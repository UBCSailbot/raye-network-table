// Copyright 2017 UBC Sailbot

#ifndef HELP_H_
#define HELP_H_

#include <string>

#include "Nt.pb.h"
#include "Value.pb.h"
#include "Node.pb.h"

namespace NetworkTable {

void PrintNode(const NetworkTable::Node &root);

/*
 * Returns node at given uri.
 * Does not modify node, but I couldn't make node a const reference.
 * @param uri - path to the node to get, seperated by '/'.
 *              eg. "/gps/lat" or "gps/lat"
 * @throws - NodeNotFoundException if the node at the uri doesn't exist
 */
NetworkTable::Node GetNode(std::string uri, NetworkTable::Node &node);

/*
 * Sets a given node in the tree. Creates the intermediate nodes
 * if they don't exist.
 * @param uri - path to the node to set, seperated by '/'.
 *              eg. "/gps/lat" or "gps/lat"
 * @param value - The value at the given uri.
 */
void SetNode(std::string uri, NetworkTable::Value value, NetworkTable::Node &root);

void Write(std::string filepath, const NetworkTable::Node &root);

NetworkTable::Node Load(const std::string &filepath);

NetworkTable::Nt RootToNt(NetworkTable::Node &root);

NetworkTable::Node NtToRoot(const NetworkTable::Nt &nt);

}  // namespace NetworkTable

#endif  // HELP_H_
