# ros-weed-control
This repo contains all ROS 2 packages that were developed for the OFA Weed Control Unit.

<!-- TODO: detailed explanation, some images -->

## Hardware
<!-- TODO: describe hardware and wiring -->

## Setup
### Host
Even though Docker handles most of our setup, certain things still have to be configured on the host OS directly.

The following steps describe the setup on a reComputer Industrial J40 flashed with JetPack 6.0. If you use different hardware you will have to figure out how to reproduce the setup yourself.

1. Enable the [maximum power mode](https://wiki.seeedstudio.com/reComputer_Industrial_J40_J30_Hardware_Interfaces_Usage/#max-performance-on-recomputer-industrial).
    ```bash
    sudo nvpmodel -m 0
    ```
2. Install [jetson-stats](https://rnext.it/jetson_stats/) (for monitoring).
    ```bash
    sudo pip3 install -U jetson-stats
    ```
3. Install [Docker Engine](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository).
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
4. Add your user to the `docker` group.
    ```bash
    sudo usermod -aG docker $USER
    ```
5. Install the udev rules for the Orbbec Femto Bolt.
    ```bash
    echo 'SUBSYSTEMS=="usb", ATTRS{idVendor}=="2bc5", ATTRS{idProduct}=="066b", MODE:="0666",  OWNER:="root", GROUP:="video", SYMLINK+="Femto Bolt"' | sudo tee /etc/udev/rules.d/99-obsensor-libusb.rules
    sudo udevadm control --reload-rules && sudo udevadm trigger
    ```
6. Enable the SocketCAN interface on boot.
    ```bash
    sudo systemctl enable systemd-networkd
    echo -e '[Match]\nName=can0\n[CAN]\nBitRate=1M' | sudo tee /etc/systemd/network/80-can.network
    ```
7. Clone this repo.
    ```bash
    git clone https://github.com/OpenFieldAutomation-OFA/ros-weed-control-ros.git
    ```
8. Install the SC4-Hub USB Driver.
    ```bash
    # Fix linux headers symlink
    sudo ln -sf /usr/src/linux-headers-5.15.136-tegra-ubuntu22.04_aarch64/3rdparty/canonical/linux-jammy/kernel-source /lib/modules/5.15.136-tegra/build
    # Install driver
    cd ros-weed-control/teknic_hardware/ExarKernelDriver
    sudo ./Install_DRV_SCRIPT.sh
    ```
9. Reboot.
    ```bash
    sudo reboot
    ```

### Docker
We use Docker to simplify deployment and development of our application.

#### Deployment
To deploy the application you only need to build and run the `prod` docker container.
```bash
cd ofa-weed-control-ros
docker build --target=prod -t weed_control .
docker run weed_control COMMAND
```
<!-- TODO: explain different commands, e. g.
docker run --net=host -e "DISPLAY=$DISPLAY" weed_control ros2 launch ofa_description display1.launch.py
-->

#### Development
For development we use [Dev Containers](https://code.visualstudio.com/docs/devcontainers/containers).

To get started just open the `ros-weed-control` folder in VS Code and run **Dev Containers: Rebuild and Reopen in Container**. This will automatically build and run the container with `--target=dev` (specified in the `.devcontainer/devcontainer.json` file). Then open two new terminals, one for running `colcon build --symlink-install` and one for sourcing the workspace with `source install/setup.bash`.

After that you can run all the same commands as specified above.

## URDF
The URDF description of the robot is stored in `ofa_moveit_config/urdf/ofa_robot_description.urdf.xacro`. Everytime the URDF is changed, you need to update the IKFast plugin and regenerate the SRDF file of the MoveIt config. Details about these two steps can be found in the README of `ofa_ikfast_plugin` and `ofa_bringup`.