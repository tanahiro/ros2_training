

.home:
	mkdir -p .home

ros2_ws:
	mkdir -p ros2_ws

uid = `id -u`
gid = `id -g`

X_SCREEN = $(shell echo ${DISPLAY} | awk '{split($$1, x, ":"); print x[2]}')

.PHONY: run-container
run-container: | .home ros2_ws
	@docker run \
		--rm -it \
		--volume ${PWD}/.home:/tmp/home \
		--volume ${PWD}/ros2_ws:/ros2_ws \
		--env HOME=/tmp/home \
		--env "DISPLAY=host.docker.internal:${X_SCREEN}" \
		--env "HISTFILE=/tmp/home/.zhistory" \
		--env "SAVEHIST=5000" \
		--workdir /ros2_ws \
		--name ros2 \
		--hostname ros2 \
		ros:jazzy-desktop

.PHONY: attach-to-container
attach-to-container:
	@docker exec -it ros2 /bin/zsh

.PHONY: build-image
build-image:
	@docker build \
		--tag ros:jazzy-desktop \
		--build-arg uid=${uid} \
		--build-arg gid=${gid} \
		.


#QT_DEBUG_PLUGINS=1 ros2 run turtlesim turtlesim_node

CONTAINER_NAME := ros2
WORKING_DIR_NAME := /tmp/home/ros2_ws
HEX_CONFIG := $(shell printf "{\"containerName\":\"/${CONTAINER_NAME}\",\"settings\":{\"context\":\"desktop-linux\"}}" | od -A n -t x1 | tr -d '[\n\t ]')

.PHONY: code
code:
	code --folder-uri "vscode-remote://attached-container+${HEX_CONFIG}${WORKING_DIR_NAME}"
