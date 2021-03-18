#!/bin/bash

xfce4-terminal -T master -e "bash -iC 'echo start!; $SHELL'" \
            --tab -T ros_core -e "bash -ic 'roscore; $SHELL'" \