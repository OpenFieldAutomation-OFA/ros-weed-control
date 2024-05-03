# ofa-weed-control-ros
This repo contains all ROS 2 packages that were developed for the OFA Weed Control Unit.

<!-- TODO: detailed explanation, some images -->

## Hardware
<!-- TODO: describe hardware and wiring -->

## Docker
We use Docker to simplify deployment and development of our application.

### Host Setup
Even though Docker handles most of our setup, certain things still have to be configured on the host OS directly.

The following steps describe the setup on a reComputer Industrial J40 flashed with JetPack 5.1.1. If you use different hardware you will have to figure out how to reproduce the setup yourself.

1. [Maximize Performance](https://wiki.seeedstudio.com/reComputer_Industrial_J40_J30_Hardware_Interfaces_Usage/#max-performance-on-recomputer-industrial) of the Jetson.
    ```bash
    sudo nvpmodel -m 0
    sudo jetson_clocks
    ```
2. Install [Docker Engine](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository).
    ```bash
    # Add Docker's official GPG key:
    sudo apt-get update
    sudo apt-get install ca-certificates curl
    sudo install -m 0755 -d /etc/apt/keyrings
    sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
    sudo chmod a+r /etc/apt/keyrings/docker.asc

    # Add the repository to Apt sources:
    echo \
    "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
    $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
    sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
    sudo apt-get update

    sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
    ```
3. Add your user to the `docker` group.
    ```bash
    sudo usermod -aG docker $USER
    su - $USER
    ```
4. Install the udev rules for the Orbbec Femto Bolt.
    ```bash
    echo 'SUBSYSTEMS=="usb", ATTRS{idVendor}=="2bc5", ATTRS{idProduct}=="066b", MODE:="0666",  OWNER:="root", GROUP:="video", SYMLINK+="Femto Bolt"' | sudo tee /etc/udev/rules.d/99-obsensor-libusb.rules
    sudo udevadm control --reload-rules && sudo udevadm trigger
    ```
5. Enable the [CAN interface](https://wiki.seeedstudio.com/reComputer_Industrial_J40_J30_Hardware_Interfaces_Usage/#can).
    ```bash
    sudo modprobe mttcan
    sudo ip link set can0 type can bitrate 1000000
    sudo ip link set can0 up
    ```
10. Finally, clone this repo.
    ```bash
    git clone https://github.com/janmacro/ofa-weed-control-ros.git
    ```


### Deployment
To deploy the application you only need to build and run the `prod` docker container.
```bash
cd ofa-weed-control-ros
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

## Non-Docker
TODO