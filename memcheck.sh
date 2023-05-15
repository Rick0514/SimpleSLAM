#! /bin/bash

rosnode list > /dev/null 2>&1

if [ $? != 0 ]; then
    echo "please run roscore first!!"
    exit 1
fi

echo "use valgrind to memcheck, rebuild map, rm all files in map dir!!"

rm ./test/data/map/*

valgrind --log-file=mem.log --tool=memcheck --leak-check=full \
--show-leak-kinds=all ./build/app/app memcheck