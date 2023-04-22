#! /bin/bash

rosnode list > /dev/null 2>&1

if [ $? != 0 ]; then
    echo "please run roscore first!!"
    exit 1
fi

echo "start rviz..."

rviz -d ./test/rviz/slam.rviz > /dev/null 2>&1 &

sleep 1

echo "run app..."

./build/app/app &

sleep 1

echo "play rosbag..."

# rosbag play --clock /home/hgy/sda2/rosbag/huaqiaocheng-1.bag
rosbag play --clock /home/rick/jtcx/hgy_gitee/minimal_ws/bag/huaqiaocheng-1.bag


