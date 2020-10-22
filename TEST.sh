#!/bin/sh
# Using --security-opt seccomp=unconfined to avoid GDB error: warning: Error disabling address space randomization: Operation not permitted
# As per https://stackoverflow.com/questions/35860527/warning-error-disabling-address-space-randomization-operation-not-permitted
docker run $TERMINAL \
    --security-opt seccomp=unconfined \
    --rm \
    -u $( id -u $USER ):$( id -g $USER ) \
    -v $(pwd):/build \
    -w /build \
    -t \
    sirehna/base-image-debian9-gcc6-xdyn \
    /bin/bash -c "export LD_LIBRARY_PATH=/build/build_deb10_gcc8_debug; ./test_dtmb5415.sh `echo $*`"
