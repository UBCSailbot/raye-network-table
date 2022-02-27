# !/bin/bash

# Logs the relevant rostopics on the NUC for
# CTRL testing. See the Confluence page on
# "Feb 27 2022 Testing in Preparation for Marina"
# for more info.
#
# Make sure to source the ROS topics before
# running this script
# Ex. source ../../build/devel/setup.bash

SENSORS="/sensors"
HEAD="/desired_heading_degrees"
ACTUATE_ANG="/rudder_winch_actuation_angle"

ROS_ECHO="rostopic echo -n 1"

OUTPUT_SENSORS="sensor_data.log"
OUTPUT_HEAD="head_data.log"
OUTPUT_ACT="act_data.log"

ECHO_DATE="date >> $OUTPUT_SENSORS && date >> $OUTPUT_HEAD && date >> $OUTPUT_ACT"
# Go to the real directory and not a symlink
# directory
cd "$(dirname "$(realpath "$0")")"

printf "\n Starting data logging for CTRL\n"
printf "\n================================\n"

# Clear the output file
eval ": > $OUTPUT_SENSORS"
eval ": > $OUTPUT_HEAD"
eval ": > $OUTPUT_ACT"
printf "Saving output to %s\n" $PWD

while :
do
  eval "$ECHO_DATE"
  eval "$ROS_ECHO $SENSORS >> $OUTPUT_SENSORS"
  eval "$ROS_ECHO $HEAD >> $OUTPUT_HEAD"
  eval "$ROS_ECHO $ACTUATE_ANG >> $OUTPUT_ACT"
  sleep 1
done
