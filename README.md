# ofa-weed-control-ros
This repo contains all ROS 2 packages that were developed for the OFA Weed Control Unit.

<!-- TODO: detailed explanation, some images -->

## Docker Setup (Recommended)
We use Docker to simplify deployment and development of our application. The `Dockerfile` has a `prod` and `dev` stage.

To use Docker you first have to install it on your host machine. Then clone this repository and go into the top-level folder.
```bash
git clone https://github.com/janmacro/ofa-weed-control-ros.git
cd ofa-weed-control-ros
```

### Deployment
To deploy the application you only need to build and run the `prod` docker container.
```bash
docker build --target=prod -t ofa_container .
docker run ofa_container COMMAND
```
<!-- TODO: explain different commands -->

### Development
For development we use [Dev Containers](https://code.visualstudio.com/docs/devcontainers/containers).

To get started just open the `ofa-weed-control-ros` folder in VS Code and run **Dev Containers: Rebuild and Reopen in Container**. This will automatically build and run the container with `--target=dev` (specified in the `.devcontainer/devcontainer.json` file). Then open two new terminals, one for running `colcon build` and one for sourcing the workspace (using two terminals avoids [install artifacts](https://colcon.readthedocs.io/en/released/user/what-is-a-workspace.html#install-artifacts)).

After that you can run all the same commands as specified above.

## Local Setup
If you do not want to use Docker and you have ROS 2 installed locally, you can directly clone this repo into your `src` folder. You can use the following commands to setup a new workspace.
```bash
mkdir -p ros2_ws/src
cd ros2_ws
git clone https://github.com/janmacro/ofa-weed-control-ros.git
```
Then you need to clone the external dependencies with `vcstool`.

<!-- TODO: complete local setup (equivalent to Dockerfile) -->
