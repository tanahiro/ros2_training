# ROS2 (self) training

## Commands
Assuming the ros2 workspace directory is `ros2_ws`
### Creating package
In `ros2_ws/src`,

```bash
$ ros2 pkg create --build-type ament_cmake PACKAGE_NAME
```

```bash
# list installed packages
$ ros2 pkg list
```

### Interfaces
```bash
# list interfaces (msg, srv, action)
$ ros2 interface lits
```

### Topic
```bash
# list topics
$ ros2 topic list

# subscribe topic
$ ros2 topic echo TOPIC_NAME
```

### Service
```bash
# list services
$ ros2 service list

# call service
$ ros2 service call SERVICE_NAME SERVICE_TYPE VALUES
$ ros2 service call /trn/command trn_interfaces/srv/Command "cmd: open"
```

### Action
```bash
# list actions
$ ros2 action list

# call action
$ ros2 action send_goal ACTION_NAME TYPE VALUE
$ ros2 action send_goal /trn/action_cmd trn_interfaces/action/Command "{cmd: open}" --feedback
```

### Params
```bash
# list params
$ ros2 param list

# set param
$ ros2 param set NODE_NAME PARAMETER_NAME VALUE

# dump params in yaml format
$ ros2 param dump NODE_NAME

# run with param
$ ros2 run PACKAGE_NAME EXECUTABLE_NAME --ros-args -p PARAM_NAME:=VALUE

# run with param in file
$ ros2 run PACKAGE_NAME EXECUTABLE_NAME --ros-args --params-file PARAM_FILE.yml
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
* [ROS2 Tutorials](https://docs.ros.org/en/jazzy/Tutorials.html) (jazzy)
* [ROS2 Package list](https://index.ros.org/)
