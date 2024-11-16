FROM ubuntu:22.04

ARG USER_NAME=ardupilot
ARG USER_UID=1000
ARG USER_GID=1000
ARG COPTER_TAG=Copter-4.5.7
ARG SKIP_AP_EXT_ENV=0
ARG SKIP_AP_GRAPHIC_ENV=1
ARG SKIP_AP_COV_ENV=1
ARG SKIP_AP_GIT_CHECK=1
ARG DO_AP_STM_ENV=1

# Trick to get apt-get to not prompt for timezone in tzdata
ENV DEBIAN_FRONTEND=noninteractive

# install git 
RUN apt-get update && apt-get install -y git; git config --global url."https://github.com/".insteadOf git://github.com/

# Now grab ArduPilot from GitHub
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
RUN sudo apt-get install python3-dev python3-opencv python3-wxgtk4.0 python3-pip python3-matplotlib python3-lxml python3-pygame -y
RUN sudo pip3 install PyYAML mavproxy --user

# Continue build instructions from https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md
RUN ./waf distclean
RUN ./waf configure --board sitl
RUN ./waf copter

# Set up working directory
WORKDIR /aero2024

# Copy the source code and test files into the container
COPY ./src /aero2024/src
COPY ./tests /aero2024/tests
COPY ./utils /aero2024/utils
COPY ./data /aero2024/data

# Set environment variables
ENV PYTHONPATH=/aero2024/src

# Expose SITL and MAVProxy ports
EXPOSE 5760/tcp
EXPOSE 5762/tcp
EXPOSE 5763/tcp
EXPOSE 5501

# Multicast
EXPOSE 14550/tcp
EXPOSE 14550/udp

# Variables for simulator
ENV INSTANCE=0
ENV MODEL=gazebo-iris
ENV SPEEDUP=1
ENV VEHICLE=ArduCopter

# Finally the command
#ENTRYPOINT /ardupilot/Tools/autotest/sim_vehicle.py --vehicle ${VEHICLE} -I${INSTANCE} -w --model ${MODEL} --no-rebuild --speedup ${SPEEDUP}
# Command to start SITL
CMD ["/ardupilot/Tools/autotest/sim_vehicle.py", "-v", "ArduCopter", "--console", "--map", "-w"]