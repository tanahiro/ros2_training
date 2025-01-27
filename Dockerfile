FROM osrf/ros:jazzy-desktop

ARG uid gid

RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y ros-dev-tools ros-jazzy-random-numbers ros-jazzy-control-msgs \
    zsh vim less iproute2 iputils-ping dnsutils && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

RUN groupadd -f -g ${gid} rosuser &&\
    useradd -u ${uid} -g ${gid} -s /bin/zsh rosuser

USER ${uid}:${gid}

ENTRYPOINT ["/bin/zsh"]
