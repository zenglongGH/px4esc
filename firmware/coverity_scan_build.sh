#!/bin/bash
#
# This script builds the application using the Coverity Scan build tool,
# and prepares the archive for uploading to the cloud static analyzer.
#

function die() { echo "$@" 1>&2; exit 1; }

which cov-configure && which cov-build || die "Coverity Build Tool is not in PATH"

cov-configure --comptype gcc --compiler arm-none-eabi-gcc --template

make clean                              || die "Clean failed"
cov-build --dir build/cov-int make -j8  || die "Build failed"
cd build
tar czvf coverity.tgz cov-int

echo "Done. Please submit the archive to Coverity Scan now."
