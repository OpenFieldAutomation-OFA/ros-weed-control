# weed-control-ros
This repo contains all ROS 2 packages that were developed for the OFA Weed Control Unit.

<!-- TODO: detailed explanation, some images -->

## Cloning
The repo can be cloned directly into the `src` folder of your ROS workspace.

You can use the following commands if you don't have a workspace yet.

```bash
cd ~
mkdir -p ros2_ws/src
cd ros2_ws
git clone https://github.com/janmacro/weed-control-ros src/ofa
```

## Deployment
For deployment you only need to build and run the docker container.

```bash
cd ~/ros2_ws/src
docker build --target=prod -t ofa_container .
docker run ofa_container COMMAND
```

<!-- TODO: explain different commands -->

If you have installed ROS 2 natively you can also try running the packages without docker. You will have to look at the Dockerfile to reproduce the installation.


## Development
For development we use [Dev Containers](https://code.visualstudio.com/docs/devcontainers/containers). Just open the `src` folder in VS Code and use **Dev Containers: Rebuild and Reopen in Container**.