#!/usr/bin/env bash

set -o nounset

echo "===== Running Python lint ====="

SCRIPTS_DIRECTORY=${BASH_SOURCE%/*}
PYTHON_LINT=pep8
PYTHONLINTOUTPUT_FILE=${SCRIPTS_DIRECTORY}/pythonlint.txt

filtered_output() {
    cat ${PYTHONLINTOUTPUT_FILE} | grep -v "Total errors found: 0"
}

# Run cpplint on every cpp file that is a part of this git project
for PYTHON_FILE in $(git ls-files | grep .py$); do
    ${PYTHON_LINT} \
    --max-line-length=120 \
    ${PYTHON_FILE} \
    &>> ${PYTHONLINTOUTPUT_FILE}
done

filtered_output 1>&2

error_line_count=$(filtered_output | wc -l)

rm ${PYTHONLINTOUTPUT_FILE}

if [ ${error_line_count} != 0 ];
then
    echo "===== Python lint Failed =====";
    exit 1;
fi

echo "===== Completed Python lint ====="
