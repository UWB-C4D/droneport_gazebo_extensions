#! /usr/bin/env bash

mkdir -p ./mavlink/mavlink/
cd ./mavlink/mavlink/
git clone https://github.com/mavlink/c_library_v2.git v2.0

cd ..
cd ..

mkdir build
cd build
cmake ..
make -j 6
