#!/bin/bash

script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
log_dir="$script_dir/logfiles"
candump_cmd="candump can0"
run_cmd=$candump_cmd
timestamp_cmd="date --rfc-3339=ns"
new_log_name_cmd="date -Ins"

# Specify a different command for testing
while getopts t: flag
do
    case "${flag}" in
        t) run_cmd=${OPTARG}
    esac
done

if [ ! -d $log_dir ]
then
    mkdir $log_dir
fi

new_log=$(exec $new_log_name_cmd)
out_log="$log_dir/$new_log.log"
touch $out_log
echo "Outputting to $out_log"

exec $run_cmd |
while read line
do
    now=$(exec $timestamp_cmd)
    echo "[$now] $line" >> "$out_log"
done
