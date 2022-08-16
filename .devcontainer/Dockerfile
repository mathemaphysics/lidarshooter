FROM mathemaphysics/spatial:0.0.1

ARG DEVUSER devuser

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get -y update \
 && apt-get -y install --no-install-recommends build-essential \
 					       g++ \
					       gdb \
						   doxygen \
					       git \
					       openssh-client \
						   openssl \
						   libssl-dev \
					       wget \
					       vim \
					       sudo \
					       libboost-all-dev \
 && python3 -m pip install cookiecutter sphinx breathe sphinx_rtd_theme catkin-tools
WORKDIR /builds
RUN git clone https://github.com/Kitware/CMake.git -b v3.22.1 \
 && cd CMake && ./configure --prefix=/usr && make && make install

RUN apt-get -y update \
	&& apt-get -y install ros-desktop-full libtf2-sensor-msgs-dev \
	libstd-srvs-dev ros-std-srvs python3-std-srvs libroslib-dev python3-roslib \
	python3-rostest librostest-dev ros-geometry-msgs libjsoncpp-dev libpoco-dev \
	libspdlog-dev

ENV DEVUSER=${DEVUSER}
RUN adduser --uid 1000 --disabled-password --gecos "" ${DEVUSER} \
    && echo "${DEVUSER} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${DEVUSER}
USER ${DEVUSER}
