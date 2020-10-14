#!/usr/bin/env bash

set -o nounset

echo "===== Running CPP check ====="

SCRIPTDIR="$(cd "$(dirname "$0")" && pwd)"
ROOTDIR=${SCRIPTDIR}/../../
CPPCHECKOUTPUT_FILE=${SCRIPTDIR}/cppcheck.txt

CPPCHECK_ARGS="\
--enable=all --std=c++14 --force --verbose --quiet \
--template='{file}:{line}:{severity}:{message}' \
--suppress=unusedFunction \
--suppress=unmatchedSuppression"

filtered_output() {
    cat ${CPPCHECKOUTPUT_FILE} | grep -v "Cppcheck cannot find all the include files"
}

# Run cpplint on every cpp file that is a part of this git project
for CPP_FILE in $(git ls-files ${ROOTDIR} | grep .cpp$); do
    cppcheck ${CPPCHECK_ARGS} \
    ${CPP_FILE} \
    &>> ${CPPCHECKOUTPUT_FILE}
done

filtered_output 1>&2

# in cppcheck 1.59 missingInclude suppression appears to be broken -- this is a workaround:
#
error_line_count=$(filtered_output | wc -l)

rm ${CPPCHECKOUTPUT_FILE}

if [ ${error_line_count} != 0 ];
then
    echo "===== CPP check Failed =====";
    exit 1;
fi

echo "===== Completed CPP check ====="
