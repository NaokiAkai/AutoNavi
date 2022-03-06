#! /bin/bash

source devel/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release -i -j 8
# catkin_make -DCMAKE_BUILD_TYPE=Release -i -j 8
