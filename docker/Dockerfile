ARG ROS_DISTRO=noetic

#############################
# Base Image for Controller #
#############################
FROM osrf/ros:${ROS_DISTRO}-desktop AS base
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

# Install miniconda
RUN apt update && apt install -y curl && \
    mkdir conda && \
    curl -s https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -o /conda/install.sh && \
    bash /conda/install.sh -b -u -p /conda && \
    /conda/bin/conda init bash
ENV PATH=$PATH:/conda/bin

# Install conda environment
COPY ./environment.yml /
RUN conda env create -f environment.yml && \
    echo "conda activate ros" >> ~/.bashrc


################################
# Overlay Image for Controller #
################################
FROM base AS overlay

# Install ROS packages
RUN apt install -y ros-${ROS_DISTRO}-foxglove*

# Create over workspace
COPY ./work/ /work/

# Set default workdir
WORKDIR /work


#################
# Develop Image #
#################
FROM overlay AS dev

# Install extra tools for development
RUN conda install -y jupyterlab && \
    apt install -y nano

# Dev container arguments
ARG USERNAME=devuser
ARG UID=1000
ARG GID=${UID}

# Create new user and home directory
RUN groupadd --gid $GID $USERNAME \
    && useradd --uid ${GID} --gid ${UID} --create-home ${USERNAME} \
    && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && mkdir -p /home/${USERNAME} \
    && chown -R ${UID}:${GID} /home/${USERNAME}

# Set the ownership of the overlay workspace to the new user
RUN chown -R ${UID}:${GID} /work /conda

# Set the user 
USER ${USERNAME}

# Set envrioment
RUN conda init bash \
    && echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
