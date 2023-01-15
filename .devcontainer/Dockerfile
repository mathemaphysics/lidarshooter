FROM ubuntu:22.04

ARG DEVUSER devuser

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get -y update \
 && apt-get -y install --no-install-recommends build-essential \
	g++ gdb doxygen git openssh-client openssl libssl-dev wget \
	vim sudo libboost-all-dev python3-pip \
 && python3 -m pip install cookiecutter sphinx breathe \
 	sphinx_rtd_theme catkin-tools

RUN apt-get -y update \
	&& apt-get -y install catkin ros-desktop-full libtf2-sensor-msgs-dev \
	libstd-srvs-dev ros-std-srvs python3-std-srvs libroslib-dev python3-roslib \
	python3-rostest librostest-dev ros-geometry-msgs libjsoncpp-dev libpoco-dev \
	ros-pcl-msgs libpcl-msgs-dev libpcl-conversions-dev rosbash libusb-1.0-0-dev \
	libspdlog-dev libfmt-dev

RUN apt-get -y update \
	&& apt-get -y install qtbase5-dev qtbase5-dev-tools libqwt-qt5-dev libqwt-headers

# Install Embree binaries for Linux
WORKDIR /usr/src
RUN wget https://github.com/embree/embree/releases/download/v3.13.4/embree-3.13.4.x86_64.linux.tar.gz \
	&& tar xvf embree-3.13.4.x86_64.linux.tar.gz

# Add a testing layer
RUN apt-get -y update \
	&& apt-get -y install libgtest-dev

ENV DEVUSER=${DEVUSER}
RUN adduser --uid 1000 --disabled-password --gecos "" ${DEVUSER} \
    && echo "${DEVUSER} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${DEVUSER}
USER ${DEVUSER}
