#!/bin/bash
set -e

if [ -f devel/setup.bash ]; then source devel/setup.bash; fi
catkin test
