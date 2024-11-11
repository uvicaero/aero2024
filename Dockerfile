# Use an official ArduPilot SITL image
FROM radarku/ardupilot-sitl

# Install additional dependencies for your Python scripts
RUN apt-get update && apt-get install -y \
    python3 python3-pip python3-opencv \
    && pip3 install numpy pandas

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
EXPOSE 5760 14550

# Command to start SITL
CMD ["ardupilot/Tools/autotest/sim_vehicle.py", "-v", "ArduCopter", "--model", "gazebo-iris", "--console"]
