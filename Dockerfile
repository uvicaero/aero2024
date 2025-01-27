# PX4 base development environment
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV DISPLAY=:99
ENV TERM=xterm
ENV TZ=UTC

ARG USER_NAME=ardupilot
ARG USER_UID=1001
ARG USER_GID=1001
ARG COPTER_TAG=Copter-4.5.7
ARG SKIP_AP_EXT_ENV=0
ARG SKIP_AP_GRAPHIC_ENV=1
ARG SKIP_AP_COV_ENV=1
ARG SKIP_AP_GIT_CHECK=1
ARG DO_AP_STM_ENV=1

# SITL UDP PORTS
EXPOSE 14556
EXPOSE 14557
EXPOSE 14550
EXPOSE 14540

# install git 
RUN apt-get update && apt-get install -y git; git config --global url."https://github.com/".insteadOf git://github.com/ && apt-get install -y iproute2

# Now grab PX4 from GitHub
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive PX4-Autopilot
WORKDIR /PX4-Autopilot

# Install ubuntu prereqs
RUN touch /.dockerenv
RUN bash /PX4-Autopilot/Tools/setup/ubuntu.sh


RUN git config --global --add safe.directory '*'

#RUN useradd --shell /bin/bash -u 1001 -c "" -m user && usermod -a -G dialout user
WORKDIR /

RUN git clone https://github.com/ArduPilot/ardupilot.git ardupilot
WORKDIR /ardupilot

# Checkout the latest Copter...
RUN git checkout ${COPTER_TAG}

# Now start build instructions from http://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html
RUN git submodule update --init --recursive

# Need sudo and lsb-release for the installation prerequisites
RUN apt-get install -y sudo lsb-release tzdata

# Create non root user for pip
RUN groupadd ${USER_NAME} --gid ${USER_GID}\
    && useradd -l -m ${USER_NAME} -u ${USER_UID} -g ${USER_GID} -s /bin/bash

RUN echo "ardupilot ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER_NAME}
RUN chmod 0440 /etc/sudoers.d/${USER_NAME}

RUN chown -R ${USER_NAME}:${USER_NAME} /${USER_NAME}

USER ${USER_NAME}

# Need USER set so usermod does not fail...
# Install all prerequisites now
RUN SKIP_AP_EXT_ENV=$SKIP_AP_EXT_ENV SKIP_AP_GRAPHIC_ENV=$SKIP_AP_GRAPHIC_ENV SKIP_AP_COV_ENV=$SKIP_AP_COV_ENV SKIP_AP_GIT_CHECK=$SKIP_AP_GIT_CHECK \
    DO_AP_STM_ENV=$DO_AP_STM_ENV \
    AP_DOCKER_BUILD=1 \
    USER=${USER_NAME} \
    Tools/environment_install/install-prereqs-ubuntu.sh -y

# Install python dependencies
#RUN sudo apt-get install python3-dev python3-opencv python3-wxgtk4.0 python3-pip python3-matplotlib python3-lxml python3-pygame -y
RUN sudo pip3 install -U mavproxy

# Continue build instructions from https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md
RUN ./waf distclean
RUN ./waf configure --board sitl
RUN ./waf copter

# Install Mav SDK
RUN sudo pip3 install -U mavsdk

# Fix could not load QT
RUN sudo pip3 uninstall opencv-python
RUN sudo pip3 install opencv-python-headless

#Enter Aero2024 Directory
WORKDIR /aero2024

# Copy the source code and test files into the container
COPY ./src /aero2024/src
COPY ./tests /aero2024/tests
COPY ./utils /aero2024/utils
COPY ./data /aero2024/data

# Set environment variables
ENV PYTHONPATH="${PYTHONPATH}: /aero2024"

CMD ["/ardupilot/Tools/autotest/sim_vehicle.py", "-v", "ArduCopter", "-w"]