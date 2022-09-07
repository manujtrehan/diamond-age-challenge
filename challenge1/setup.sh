#!/bin/bash
set -e

vcs import < src/ros.repos src
vcs import < src/.rosinstall src
sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -y
