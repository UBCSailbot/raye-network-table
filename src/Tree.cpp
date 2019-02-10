// Copyright 2017 UBC Sailbot

#include "Tree.h"
#include "Exceptions.h"

#include <boost/algorithm/string.hpp>
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>

void PrintTree(NetworkTable::Node root, int depth);

void NetworkTable::PrintTree(NetworkTable::Node root) {
    ::PrintTree(root, 0);
    std::cout << std::endl;
}

// Helper function
void PrintTree(NetworkTable::Node root, int depth) {
    auto children = root.children();

    // If root is a leaf, just print out the data.
    if (children.size() == 0) {
        std::cout << '\t';
        switch (root.value().type()) {
            case NetworkTable::Value::INT : std::cout << root.value().int_data();
                                            break;
            case NetworkTable::Value::BOOL : std::cout << root.value().bool_data();
                                            break;
            case NetworkTable::Value::DOUBLE : std::cout << root.value().double_data();
                                            break;
            case NetworkTable::Value::STRING : std::cout << root.value().string_data();
                                            break;
            case NetworkTable::Value::BYTES : std::cout << root.value().bytes_data();
                                            break;
            default: std::cout << "Unknown type\n";
        }
    } else {
        // Otherwise, recursively print out the children
        for (auto it = children.begin(); it != children.end(); ++it) {
            std::cout << std::endl;
            for (int i = 0; i < depth; i++) {
                std::cout << '\t';
            }
            std::cout << it->first;
            PrintTree(it->second, depth + 1);
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

void NetworkTable::Tree::Write(std::string filepath) {
    std::ofstream output_filestream(filepath);
    output_filestream << root_.SerializeAsString();
    output_filestream.close();
}

void NetworkTable::Tree::Load(std::string filepath) {
    // http://insanecoding.blogspot.ca/2011/11/how-to-read-in-file-in-c.html
    std::ifstream input_filestream(filepath);
    std::ostringstream contents;
    if (input_filestream) {
        contents << input_filestream.rdbuf();
        root_.ParseFromString(contents.str());
    } else {
        throw(errno);
    }
}
