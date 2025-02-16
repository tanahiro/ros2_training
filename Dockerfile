FROM osrf/ros:jazzy-desktop

ARG uid gid

RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y ros-dev-tools ros-jazzy-random-numbers ros-jazzy-control-msgs \
        clangd zsh vim less iproute2 iputils-ping dnsutils openssh-server gosu && \
    apt-get clean && rm -rf /var/lib/apt/lists/* && \
    mkdir -p /var/run/sshd

# add dialout for /dev/tty*
RUN groupadd -f -g ${gid} rosuser &&\
    useradd -u ${uid} -g ${gid} -s /bin/zsh -d /tmp/home rosuser && \
    usermod -aG dialout rosuser

RUN --mount=type=secret,id=rosuser_password cat /run/secrets/rosuser_password | chpasswd && \
    sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/g' /etc/ssh/sshd_config && \
    sed -i 's/#X11UseLocalhost yes/X11UseLocalhost no/g' /etc/ssh/sshd_config

COPY entrypoint.sh /entrypoint.sh

EXPOSE 22

ENTRYPOINT ["/entrypoint.sh"]
