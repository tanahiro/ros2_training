FROM osrf/ros:jazzy-desktop

ARG uid gid USER_PASSWD

RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y ros-dev-tools ros-jazzy-random-numbers ros-jazzy-control-msgs \
        clangd zsh vim less iproute2 iputils-ping dnsutils openssh-server gosu && \
    apt-get clean && rm -rf /var/lib/apt/lists/* && \
    mkdir -p /var/run/sshd

# add dialout for /dev/tty*
RUN groupadd -f -g ${gid} rosuser &&\
    useradd -u ${uid} -g ${gid} -s /bin/zsh -d /tmp/home rosuser && \
    usermod -aG dialout rosuser

RUN echo "rosuser:${USER_PASSWD}" | chpasswd && \
    sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/g' /etc/ssh/sshd_config

COPY entrypoint.sh /entrypoint.sh

EXPOSE 22

ENTRYPOINT ["/entrypoint.sh"]
