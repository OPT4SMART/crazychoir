ARG UBUNTU_VERSION=20.04
ARG WEBOTS_VERSION=R2022a

FROM cyberbotics/webots:${WEBOTS_VERSION}-ubuntu${UBUNTU_VERSION}

ENV DEV_NAME=opt4smart
ENV ROS_DISTRO=foxy
ENV WS_NAME=crazychoir_ws

##########################################################
# Solves bugs of cyberboics/webots:R2022a image
RUN rm /etc/apt/sources.list.d/cuda.list && \
    apt-key del 7fa2af80  && \
    apt-get update && apt-get install -y --no-install-recommends wget  && \
    wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-keyring_1.0-1_all.deb  && \
    dpkg -i cuda-keyring_1.0-1_all.deb && \
    rm cuda-keyring_1.0-1_all.deb
##########################################################

RUN apt-get update -q && \
    apt-get upgrade -yq && \
    apt-get install -yq \
        apt-transport-https \
        apt-utils \
        bash-completion \
        build-essential \
        cmake \
        curl \
        git \
        gedit \
        locales \
        lsb-release \
        nano \
        make gcc-arm-none-eabi \
        openssh-client \
        python3-argcomplete \
        python3-pip \
        software-properties-common \
        sudo \
        swig \
        vim \
        wget \
        xterm
    # && rm -rf /var/lib/apt/lists/*

##########################################################
# Install ROS 2
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8 && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt update -qq && \
    apt install -yq --no-install-recommends\
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-vision-msgs \
    ros-${ROS_DISTRO}-hardware-interface \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-rqt-graph \
    ros-${ROS_DISTRO}-gazebo-* \
    ros-${ROS_DISTRO}-dynamixel-sdk \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-turtlebot3-msgs \
    ros-${ROS_DISTRO}-turtlebot3 \
    python3-rosdep \
    python3-colcon-common-extensions \
    ros-dev-tools \
    python3-argcomplete \
    python3-vcstool 

RUN groupadd -g 1000 ${DEV_NAME} && \
    useradd -d /home/${DEV_NAME} -s /bin/bash -m ${DEV_NAME} -u 1000 -g 1000 && \
    usermod -aG sudo ${DEV_NAME} && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers  && \
    usermod -a -G plugdev ${DEV_NAME}

##########################################################
# Install CrazyChoir 
RUN mkdir -p /home/${DEV_NAME}/${WS_NAME}
COPY ./requirements.txt /home/${DEV_NAME}/${WS_NAME}/

# Clone Crazyflie firmware
USER root
WORKDIR /home/${DEV_NAME}/
RUN git clone --recursive https://github.com/bitcraze/crazyflie-firmware.git --branch 2023.02
WORKDIR /home/${DEV_NAME}/crazyflie-firmware/
RUN git submodule init
RUN git submodule update

RUN make cf2_defconfig
RUN make -j 12 
RUN make bindings_python

##########################################################
# Install python tools
RUN pip3 install -U pip setuptools==58.2.0 tqdm tikzplotlib
WORKDIR /home/${DEV_NAME}/${WS_NAME}
RUN pip3 install --upgrade pip && \
    pip3 install -r requirements.txt && \
    pip3 install --no-deps disropt 

##########################################################
# Install qpSWIFT
WORKDIR /home/${DEV_NAME}/
RUN git clone https://github.com/qpSWIFT/qpSWIFT && \
    cd /home/${DEV_NAME}/qpSWIFT/python && \
    sudo python3 setup.py install

##########################################################
# Clean up unnecessary files (if any)
USER root
RUN apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

##########################################################
# install vicon libs and fix paths
COPY ./final_setup.sh /home/${DEV_NAME}/
RUN chmod +x /home/${DEV_NAME}/final_setup.sh

USER ${DEV_NAME}

# Set useful .bashrc aliases
RUN echo 'PROMPT_DIRTRIM=1' >> /home/${DEV_NAME}/.bashrc && \
    echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /home/${DEV_NAME}/.bashrc &&\
    echo 'cat<<TF' >> /home/${DEV_NAME}/.bashrc && \
    echo '' >> /home/${DEV_NAME}/.bashrc && \
    echo '' >> /home/${DEV_NAME}/.bashrc && \
    echo '' >> /home/${DEV_NAME}/.bashrc && \
    echo 'Welcome to...' >> /home/${DEV_NAME}/.bashrc && \
    echo '' >> /home/${DEV_NAME}/.bashrc && \
    echo '                  // \    ' >> /home/${DEV_NAME}/.bashrc && \
    echo '                  \\_/ // ' >> /home/${DEV_NAME}/.bashrc && \
    echo '**-.._.-**-.._.. -(||)(*) ' >> /home/${DEV_NAME}/.bashrc && \    
    echo '' >> /home/${DEV_NAME}/.bashrc && \
    echo '------- CCCC  RRRR       A     ZZZZZ  Y   Y CCCC  H   H   OOO   I  RRRR   --- ' >> /home/${DEV_NAME}/.bashrc && \
    echo '------ C     R   R     A A       Z    Y Y  C     H   H  O   O  I  R   R  ---- ' >> /home/${DEV_NAME}/.bashrc && \
    echo '----- C     RRRR     AAAAA     Z      Y   C     HHHHH  O   O  I  RRRR   ----- ' >> /home/${DEV_NAME}/.bashrc && \
    echo '---- C     R  R    A     A   Z       Y   C     H   H  O   O  I  R  R   ------ ' >> /home/${DEV_NAME}/.bashrc && \
    echo '---  CCCC R    R A       A ZZZZZ    Y    CCCC H   H   OOO   I  R    R ------- ' >> /home/${DEV_NAME}/.bashrc && \
    echo '' >> /home/${DEV_NAME}/.bashrc && \
    echo 'TF' >> /home/${DEV_NAME}/.bashrc

###########################################################

WORKDIR /home/${DEV_NAME}/${WS_NAME}
ENTRYPOINT /home/${DEV_NAME}/final_setup.sh && /bin/bash
