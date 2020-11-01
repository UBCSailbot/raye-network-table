#!/usr/bin/env python3

# Script generates a JSON file that contains all of the CANBUS frame IDs
# defined within frame_parser.h (frame_parser.h is single source of truth)

# Result JSON file will be stored as /network-table/src/frame_ids.json

import json
import os
import re

script_location = os.path.dirname(os.path.realpath(__file__))
src_location = os.path.abspath(script_location +
                               '/../../src/')


def read_frame_parser():
    id_array = []
    frame_parser_location = os.path.abspath(src_location +
                                            '/uccm-sensors/frame_parser.h')
    with open(frame_parser_location, "r") as frame_parser:
        read_file = frame_parser.readlines()

    for line in read_file:
        # matches lines like #define BMS_FRAME_ID_1       0x00000008
        if re.match(r'^#define.*0x[0-9A-F]{1,8}', line):
            id_array.append(line)

    return id_array


def parse_ids(id_array):
    id_dict = {}

    for id in id_array:
        id = id[8:].split()
        id_dict[id[0]] = hex(int(id[1], 16))

    return id_dict


def generate_JSON(id_dict):
    # Create JSON object from dictionary
    json_object = json.dumps(id_dict, indent=4)

    # Write JSON object to file
    with open(src_location + "/frame_ids.json", "w") as output_file:
        output_file.write(json_object)


def main():
    # Read frame defines from top of frame_parser.h
    id_array = read_frame_parser()

    # Process each string in id_array and return dictionary
    # Dictionary entries in the form $NAME : $ID
    id_dict = parse_ids(id_array)

    # Generate JSON file from id_dict dictionary
    generate_JSON(id_dict)
    return


if __name__ == "__main__":
    main()
