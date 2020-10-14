#!/usr/bin/env bash

set -o nounset

echo "===== Running Python lint ====="

SCRIPTDIR="$(cd "$(dirname "$0")" && pwd)"
PYTHON_LINT=pep8
ROOTDIR=${SCRIPTDIR}/../../
PYTHONLINTOUTPUT_FILE=${SCRIPTDIR}/pythonlint.txt

filtered_output() {
    cat ${PYTHONLINTOUTPUT_FILE} | grep -v "Total errors found: 0"
}

# Run cpplint on every cpp file that is a part of this git project
for PYTHON_FILE in $(git ls-files ${ROOTDIR} | grep .py$); do
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
