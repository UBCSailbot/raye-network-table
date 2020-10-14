#!/usr/bin/env bash

set -o nounset

echo "===== Running CPP lint ====="

SCRIPTS_DIRECTORY=${BASH_SOURCE%/*}
CPP_LINT=${SCRIPTS_DIRECTORY}/cpplint.py
CPPLINTOUTPUT_FILE=${SCRIPTS_DIRECTORY}/cpplint.txt

filtered_output() {
    cat ${CPPLINTOUTPUT_FILE} | grep -v "Total errors found: 0"
}

# Run cpplint on every cpp file that is a part of this git project
for CPP_FILE in $(git ls-files | grep .cpp$); do
    ${CPP_LINT} \
    --linelength=120 --counting=detailed \
    ${CPP_FILE} \
    &>> ${CPPLINTOUTPUT_FILE}
done

filtered_output 1>&2

# in cppcheck 1.59 missingInclude suppression appears to be broken -- this is a workaround:
#
error_line_count=$(filtered_output | wc -l)

rm ${CPPLINTOUTPUT_FILE}

if [ ${error_line_count} != 0 ];
then
    echo "===== CPP lint Failed =====";
    exit 1;
fi

echo "===== Completed CPP lint ====="
