#! /bin/bash

usebag=false
usebag_flag=""

if [[ $# == 1 && $1 = "rosbag" ]]; then
    echo "build as usebag mode..."
    usebag=true
fi

if [ $usebag ]; then
    usebag_flag="-DUSE_BAG=ON"
fi

echo "exec cmake -B build $usebag_flag"

cmake -B build $usebag_flag

cmake --build build -- -j 8