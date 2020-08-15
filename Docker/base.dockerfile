# This Dockerfile will create a basic ROS image which runs a SSH server and uses X11

## ROS Setup ##################################################################
# Start with base ROS image
FROM osrf/ros:melodic-desktop-bionic

# Install ros packages, because why would they be included in the ROS image? :/
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-desktop-full=1.4.1-0* \
    && rm -rf /var/lib/apt/lists/*

## Run Installs ###############################################################
# These are run first to avoid having to re-run them on file edits/rebuilds
# Update apt
RUN apt-get update
# Text editor
RUN apt-get -y install nano
# Networking tools
RUN apt-get -y install net-tools
RUN apt-get -y install iputils-ping
# SSH server
RUN apt-get -y install openssh-server
# X11 server
RUN apt-get -y install x11-apps
# Higher version of C++
RUN apt-get -y install gcc-8
RUN apt-get -y install g++-8

