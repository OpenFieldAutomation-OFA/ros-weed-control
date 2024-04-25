# ofa-weed-control-ros
This repo contains all ROS 2 packages that were developed for the OFA Weed Control Unit.

<!-- TODO: detailed explanation, some images -->

## Docker Setup
We use Docker to simplify deployment and development of our application.

First, install Docker using the [official install guide](https://docs.docker.com/engine/install/ubuntu/). Then clone this repository and go into the top-level folder.
```bash
git clone https://github.com/janmacro/ofa-weed-control-ros.git
cd ofa-weed-control-ros
```

You also need to install the udev rules for the camera on the host OS.
```bash
sudo ./setup
```

### Deployment
To deploy the application you only need to build and run the `prod` docker container.
```bash
docker build --target=prod -t weed_control .
docker run weed_control COMMAND
```
<!-- TODO: explain different commands, e. g.
docker run --net=host -e "DISPLAY=$DISPLAY" weed_control ros2 launch ofa_visualization display1.launch.py
-->

### Development
For development we use [Dev Containers](https://code.visualstudio.com/docs/devcontainers/containers).

To get started just open the `ofa-weed-control-ros` folder in VS Code and run **Dev Containers: Rebuild and Reopen in Container**. This will automatically build and run the container with `--target=dev` (specified in the `.devcontainer/devcontainer.json` file). Then open two new terminals, one for running `colcon build` and one for sourcing the workspace (using two terminals avoids [install artifacts](https://colcon.readthedocs.io/en/released/user/what-is-a-workspace.html#install-artifacts)).

After that you can run all the same commands as specified above.

## Local Setup
TODO