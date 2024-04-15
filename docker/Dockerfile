# ROS distribution to use
ARG ROS_DISTRO=noetic

###############################################################################################
# Base Image for #
###############################################################################################

FROM osrf/ros:${ROS_DISTRO}-desktop-full as base
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

# Install basic apt packages
RUN apt-get update && apt-get install -y \
  cmake \
  g++ \
  git \
  libcanberra-gtk* \
  python3-catkin-tools \
  python3-pip \
  python3-scipy \
  wget \
  vim \
  iputils-ping \
  python3-vcstool \
  ros-${ROS_DISTRO}-robot-localization* \
  ros-${ROS_DISTRO}-realsense* \
  ros-${ROS_DISTRO}-geodesy \
  ros-${ROS_DISTRO}-geographic-msgs \
  ros-${ROS_DISTRO}-costmap-2d \
  ros-${ROS_DISTRO}-octomap-msgs \
  ros-${ROS_DISTRO}-cv-bridge \
  ros-${ROS_DISTRO}-vision-opencv \
  ros-${ROS_DISTRO}-mavros && \
  rm -rf /var/lib/apt/lists/*

# Install Armadillo
RUN apt-get update && apt-get install -y \
  libopenblas-openmp-dev \
  libarmadillo-dev && \
  rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --user scipy

# Create Catkin workspace 
RUN mkdir -p /underlay_ws/src 
WORKDIR /underlay_ws/src
COPY dependencies.repos .
RUN vcs import < dependencies.repos
RUN source /opt/ros/noetic/setup.bash \
 && catkin init 

# Build the base Catkin workspace
RUN cd /underlay_ws \
 && source /opt/ros/noetic/setup.bash \
 && rosdep install -y --from-paths src --ignore-src \
 && catkin build -j2

# Remove display warnings
RUN mkdir /tmp/runtime-root
ENV XDG_RUNTIME_DIR "/tmp/runtime-root"
ENV NO_AT_BRIDGE 1

# Set up the work directory and entrypoint
WORKDIR /underlay_ws
COPY ./docker/entrypoint.sh /

# Alias python3 as python
RUN ln -s /usr/bin/python3 /usr/bin/python

ENTRYPOINT [ "/entrypoint.sh" ]


###############################################################################################
# Overlay Image #
###############################################################################################
FROM base AS overlay
# Create an overlay Catkin workspace
RUN source /opt/ros/noetic/setup.bash \
 && mkdir -p /overlay_ws/src \
 && cd /overlay_ws \ 
 && catkin init
#COPY ./some_folder/ /overlay_ws/src/some_folder/
#COPY ./some_other_folder/ /overlay_ws/src/some_other_folder/
RUN source /opt/ros/noetic/setup.bash \
 && catkin build

# Set up the work directory and entrypoint
WORKDIR /overlay_ws
COPY ./docker/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]


################################################################################################
## Development Image #
################################################################################################
FROM overlay as dev

# Dev container arguments
ARG USERNAME=devuser
ARG UID=1000
ARG GID=${UID}

# Install extra tools for development
RUN apt-get update && apt-get install -y --no-install-recommends gdb gdbserver nano


# RUN git clone https://gitlab.gbar.dtu.dk/dtu-asl/courses/34763-autonomous-marine-robotics && \
#     cd 34763-autonomous-marine-robotics && git remote rename origin course_repo

# RUN cd /overlay_ws/34763-autonomous-marine-robotics/ros_ws && \
#     /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make -j2 -l2"

# RUN echo "source /overlay_ws/34763-autonomous-marine-robotics/ros_ws/devel/setup.bash" >> /overlay_ws/.bashrc

ARG CONDA_URL=https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
ARG PATH="/root/miniconda3/bin:${PATH}"
ENV PATH="/root/miniconda3/bin:${PATH}"
RUN wget $CONDA_URL \
    && mkdir /root/.conda \
    && bash $(basename $CONDA_URL) -b \
    && rm -f $(basename $CONDA_URL)


# Create new user and home directory
RUN groupadd --gid $GID $USERNAME \
 && useradd --uid ${GID} --gid ${UID} --create-home ${USERNAME} \
 && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME} \
 && mkdir -p /home/${USERNAME} \
 && chown -R ${UID}:${GID} /home/${USERNAME}

# Set the ownership of the overlay workspace to the new user
RUN chown -R ${UID}:${GID} /underlay_ws/ /overlay_ws/


# Set the user and source entrypoint in the user's .bashrc file
USER ${USERNAME}
RUN echo "source /entrypoint.sh" >> /home/${USERNAME}/.bashrc