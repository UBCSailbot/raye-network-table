// Copyright 2017 UBC Sailbot

#include "Tree.h"

#include <boost/algorithm/string.hpp>
#include <vector>
#include <iostream>


void NetworkTable::PrintTree(NetworkTable::Node root) {
    auto children = root.children();

    if (children.size() == 0) {
        std::cout << root.value().int_data() << std::endl;
    } else {
        for (auto it = children.begin(); it != children.end(); ++it) {
            std::cout << it->first << "/";
            PrintTree(it->second);
        }
    }
}

NetworkTable::Node NetworkTable::Tree::GetNode(std::string uri) {
    if (uri == "/" || uri == "") {
        return root_;
    }

    // Get each "slice" of the uri. eg "/gps/lat"
    // becomes {"gps", "lat"}:
    boost::trim_left_if(uri, boost::is_any_of("/"));
    std::vector<std::string> slices;
    boost::split(slices, uri, boost::is_any_of("/"));

    NetworkTable::Node *current_node = &root_;
    for (unsigned int i = 0; i < slices.size(); i++) {
        if (current_node->children().find(slices[i]) != current_node->children().end()) {
            current_node = &(current_node->mutable_children()->at(slices[i]));
        } else {
            throw NetworkTable::NodeNotFoundException("Could not find: " + uri);
        }
    }

    return *current_node;
}

void NetworkTable::Tree::SetNode(std::string uri, NetworkTable::Value value) {
    // Get each "slice" of the uri. eg "/gps/lat"
    // becomes {"gps", "lat"}:
    boost::trim_left_if(uri, boost::is_any_of("/"));
    std::vector<std::string> slices;
    boost::split(slices, uri, boost::is_any_of("/"));

    NetworkTable::Node *current_node = &root_;
    for (unsigned int i = 0; i < slices.size(); i++) {
        std::string slice = slices[i];
        auto children = current_node->mutable_children();

        current_node = &(*children)[slice];
    }

    current_node->set_allocated_value(new NetworkTable::Value(value));
}

void NetworkTable::Tree::PrintTree() {
    ::NetworkTable::PrintTree(root_);
}
