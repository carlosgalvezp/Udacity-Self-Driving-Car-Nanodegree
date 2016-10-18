FROM ubuntu:14.04

MAINTAINER Carlos Galvez <carlosgalvezp@gmail.com>

# Install base packages
RUN apt-get update
RUN apt-get install -y build-essential
    
# Install ROS Indigo    
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
RUN apt-get update
RUN apt-get install -y ros-indigo-desktop-full
RUN rosdep init
RUN rosdep update
RUN echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
RUN apt-get install -y python-rosinstall

CMD ["/bin/bash"]
