FROM osrf/ros:kinetic-desktop-xenial

MAINTAINER Rizwin Antony <antonyrizwin@gmail.com> 

# Install ROS and other dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-kinetic-desktop-full=1.3.2-0* \
    ros-kinetic-navigation \
    ros-kinetic-slam-gmapping \
    ros-kinetic-rosbridge-suite \
    ros-kinetic-tf2-web-republisher \
    ros-kinetic-robot-pose-publisher \
    python-pip \
    python-catkin-tools \
    python-rosdep \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool \
    build-essential \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Upgrade pip using get-pip.py
RUN curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py && \
    python get-pip.py && \
    rm get-pip.py

# Copy and install Python requirements
COPY requirements.txt /root/
RUN pip install -r /root/requirements.txt

# Set up the ROS workspace
ENV ROS_WS /root/ros_ws
RUN mkdir -p $ROS_WS/src
WORKDIR $ROS_WS/src

# Clone ROS package repositories
RUN git clone https://github.com/ROBOTIS-GIT/turtlebot3.git 
RUN git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git 
RUN git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
WORKDIR $ROS_WS

# Copy launch files
COPY ["turtlebot3_bringup.launch", "turtlebot3_navigation.launch", "/root/ros_ws/src/turtlebot3/turtlebot3_navigation/launch/"]
COPY ["turtlebot3_slam.launch", "/root/ros_ws/src/turtlebot3/turtlebot3_slam/launch/"]

# Install ROS package dependencies
RUN apt-get update && \
    rosdep init && \
    rosdep update && \
    rosdep install -y \
      --from-paths \
        src/turtlebot3/turtlebot3 \
        src/turtlebot3/turtlebot3_bringup \
        src/turtlebot3/turtlebot3_description \
        src/turtlebot3/turtlebot3_example \
        src/turtlebot3/turtlebot3_navigation \
        src/turtlebot3/turtlebot3_slam \
        src/turtlebot3/turtlebot3_teleop \
        src/turtlebot3_msgs \
        src/turtlebot3_simulations/turtlebot3_fake \
        src/turtlebot3_simulations/turtlebot3_gazebo \
        src/turtlebot3_simulations/turtlebot3_simulations \
      --ignore-src && \
    rm -rf /var/lib/apt/lists/*

# Build ROS packages
RUN catkin config --extend /opt/ros/$ROS_DISTRO && \
    catkin build \
      turtlebot3 \
      turtlebot3_bringup \
      turtlebot3_description \
      turtlebot3_example \
      turtlebot3_navigation \
      turtlebot3_slam \
      turtlebot3_teleop \
      turtlebot3_msgs \
      turtlebot3_fake \
      turtlebot3_gazebo \
      turtlebot3_simulations

# Source ROS package from entrypoint
RUN sed --in-place --expression '$isource "$ROS_WS/devel/setup.bash"' /ros_entrypoint.sh

# Set working directory for Flask app
WORKDIR /root/

# Copy Flask app files
COPY static /root/static/
COPY templates /root/templates
COPY app.py /root

# Set environment variables for Flask
ENV FLASK_APP=app.py
ENV TURTLEBOT3_MODEL=waffle

# Expose Flask app port
EXPOSE 5000

# Run the Flask app
CMD ["flask", "run", "--host=0.0.0.0"]
