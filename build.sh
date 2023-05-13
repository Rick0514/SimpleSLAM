#! /bin/bash

usebag_flag=""

cmd="$@"

if echo $cmd | grep -q "rosbag"; then
    echo "build as usebag mode..."
    usebag_flag="-DUSE_BAG=ON"
fi

if echo $cmd | grep -q "rebuild"; then
    if [ -d "./build" ]; then
        echo "remove build dir first"
        rm -rf build
    else
        echo "build dir is not exist, no need to remove"
    fi 
    echo "rebuild all target..."
fi

echo "exec cmake -B build $usebag_flag"

start=$(date +%s%N)

cmake -B build $usebag_flag
cmake --build build -- -j 8

end=$(date +%s%N)

echo "Compilation elapsed time: $(($(($end-$start))/1000000)) ms"