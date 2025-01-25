# ROS2 (self) training

## Commands
Assuming the ros2 workspace directory is `ros2_ws`
### creating package
In `ros2_ws/src`,

```bash
ros2 pkg create --build-type ament_cmake PACKAGE_NAME
```

### building pacakges
In `ros2_ws`,
```bash
# build all packages under ros2_ws
$ colcon build

# build specific pacakge
$ colcon build --packages-select PACKAGE_NAME
```

You may also need to source `ros2_ws/install/setup.[sh|zsh|bash]`

## Docker dev env
```bash
# build ros2 image
$ make build-image 

# run ros2 container
$ make run-container

# attach to running ros2 container
$ make attach-to-container
```

You can open workspaces in the container with VisualStudio Code,
but cpptool may fail to execute.
In that case, change the permission of the following files in
`~/.vscode-server/extensions/ms-vscode.cpptools-x.xx.xx-linux-x64/bin` to `0x755`

* `cpptools`
* `cpptools-srv`
* `cpptools-wordexp`
* `libc.so`



## Links
* [ROS 2 Tutorials](https://docs.ros.org/en/jazzy/Tutorials.html) (jazzy)
