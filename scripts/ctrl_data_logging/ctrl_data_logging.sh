# !/bin/bash

# Logs the relevant rostopics on the NUC for
# CTRL testing.


SENSORS="/sensors"
HEAD="/desired_heading_degrees"
ACTUATE_ANG="/actuation_angle"

ROS_ECHO="rostopic echo"

OUTPUT_FILE="CTRL_data.log"

# Go to the real directory and not a symlink
# directory
cd "$(dirname "$(realpath "$0")")"
OUTPUT_LOC="$PWD$OUTPUT_FILE"

printf "\n Starting data logging for CTRL\n"
printf "\n================================\n"

# Source the ROS topocs
source ../../build/devel/setup.sh
# Clear the output file
eval ": > $OUTPUT_FILE$"
printf "Saving output to %s\n" $OUTPUT_LOC
printf "


eval "$ROS_ECHO $SENSORS >> $OUTPUT_FILE"
eval "$ROS_ECHO $HEAD >> $OUTPUT_FILE"
eval "$ROS_ECHO $ACTUATE_ANG >> "$OUTPUT_FILE"
