# Ubuntu 16.04를 베이스로
FROM ubuntu:16.04

# 
RUN apt-get -y update && apt-get -y upgrade && apt-get install -y wget sudo curl && \
    wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_kinetic.sh && \
    chmod 755 ./install_ros_kinetic.sh && \
    bash ./install_ros_kinetic.sh && \
    sudo apt-get install -y ros-kinetic-joy ros-kinetic-teleop-twist-joy \
    ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc \
    ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan \
    ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python \
    ros-kinetic-rosserial-server ros-kinetic-rosserial-client \
    ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server \
    ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro \
    ros-kinetic-compressed-image-transport ros-kinetic-rqt* \
    ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers && \
    sudo apt-get install -y ros-kinetic-dynamixel-sdk && \
    sudo apt-get install -y ros-kinetic-turtlebot3-msgs && \
    sudo apt-get install -y ros-kinetic-turtlebot3 && \
    echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc

  
