#! /bin/bash

rosnode list > /dev/null 2>&1

if [ $? != 0 ]; then
    echo "please run roscore first!!"
    exit 1
fi

cmd="$@"

if echo $cmd | grep -q "rviz"; then
    echo "start rviz..."

    rviz -d ./test/rviz/slam.rviz > /dev/null 2>&1 &
    
    sleep 1
fi

if echo $cmd | grep -q "rebuild"; then

    echo "rebuild map, rm all files in map dir!!"

    rm ./test/data/map/*

fi

echo "run app..."

./build/app/app

# rosbag play --clock /home/hgy/sda2/rosbag/zhengzhou0711.orig.bag
# rosbag play --clock /home/rick/jtcx/hgy_gitee/minimal_ws/bag/zhengzhou0711.orig.bag
# rosbag play --clock /home/hgy/sda2/rosbag/huaqiaocheng-1.bag
# rosbag play --clock /home/rick/jtcx/hgy_gitee/minimal_ws/bag/huaqiaocheng-1.bag


