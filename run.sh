#!/usr/bin/sh

BUILD=Release

set -e
mkdir -p build/${BUILD}
cd build/${BUILD}
cmake -DOpenCV_DIR=/usr/local/share/OpenCV \
      -DOpenIGTLink_DIR=/usr/local/lib/igtl\
      -DCMAKE_BUILD_TYPE=${BUILD}\
      ../../
make -j 5
echo
echo ===============================================
echo
./camera-calibration ../../data/settings.xml | tee log_$(date +"%Y-%m-%d_%H_%M_%S").txt
