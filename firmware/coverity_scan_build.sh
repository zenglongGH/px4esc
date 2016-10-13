#!/bin/bash
#
# This script builds the application using the Coverity Scan build tool,
# and prepares the archive for uploading to the cloud static analyzer.
#

OUTPUT_DIR=build/cov-int
OUTPUT_ARCHIVE=coverity.tgz

function die() { echo "$@" 1>&2; exit 1; }

which cov-configure && which cov-build || die "Coverity Build Tool is not in PATH"

cov-configure --comptype gcc --compiler arm-none-eabi-gcc --template

make clean                              || die "Clean failed"
rm -rf $OUTPUT_DIR $OUTPUT_ARCHIVE &> /dev/null
mkdir -p $OUTPUT_DIR
cov-build --dir $OUTPUT_DIR make -j8    || die "Build failed"
tar czvf $OUTPUT_ARCHIVE $OUTPUT_DIR

echo "Done. Please submit the archive '$OUTPUT_ARCHIVE' to Coverity Scan now."
