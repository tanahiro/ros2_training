FROM osrf/ros:jazzy-desktop

ARG uid gid

RUN apt-get update && apt-get upgrade -y && \
  apt-get install -y zsh iproute2 iputils-ping dnsutils && \
  apt-get clean && rm -rf /var/lib/apt/lists/*

RUN groupadd -f -g ${gid} rosuser &&\
  useradd -u ${uid} -g ${gid} -s /bin/zsh rosuser

ENTRYPOINT ["/bin/zsh"]

