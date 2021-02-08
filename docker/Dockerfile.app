# FROM ubuntu:16.04
FROM osrf/ros:melodic-desktop-full


RUN echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc
RUN apt-get update && apt-get install -y morse-simulator python3-morse-simulator 
RUN apt-get update && apt-get install -y python3-dev python3-yaml apt-utils python-rospkg python3-pip
RUN pip3 install rospkg
# RUN apt-get update && apt-get install -y python3-catkin-tools


WORKDIR /ros_ws

RUN \
  apt-get update && \
  apt-get -y install libgl1-mesa-glx libgl1-mesa-dri && \
  rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get install -y mesa-utils

#NVIDIA
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}


# RUN version="$(glxinfo | grep "OpenGL version string" | rev | cut -d" " -f1 | rev)"
# RUN wget http://us.download.nvidia.com/XFree86/Linux-x86_64/"$version"/NVIDIA-Linux-x86_64-"$version".run
# RUN mv NVIDIA-Linux-x86_64-"$version".run NVIDIA-DRIVER.run
# RUN ./NVIDIA-DRIVER.run -a -N --ui=none --no-kernel-module

RUN mkdir src
COPY hos /ros_ws/src
RUN /bin/bash -c "source /ros_entrypoint.sh && catkin_make"
RUN echo "source /ros_ws/devel/setup.bash" >> /root/.bashrc