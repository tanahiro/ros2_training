CONTAINER_NAME := ros2
WORKING_DIR_NAME := /ros2_ws/src/trn
HEX_CONFIG := $(shell printf "{\"containerName\":\"/${CONTAINER_NAME}\",\"settings\":{\"context\":\"desktop-linux\"}}" | od -A n -t x1 | tr -d '[\n\t ]')

# open workspace in Docker container
.PHONY: code
code:
	@code \
		--file-uri "vscode-remote://attached-container+${HEX_CONFIG}${WORKING_DIR_NAME}/trn.code-workspace"
