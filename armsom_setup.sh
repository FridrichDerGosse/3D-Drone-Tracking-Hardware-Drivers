#! /usr/bin/bash
# --> flash debian to armsom SD card

function update_system
{
    # update system
    sudo apt update && sudo apt upgrade -y
}

function setup_camera
{
    # enable device tree overlay to support CSI-cameas
    echo "Initial armbianEnv.txt:\n"
    cat /boot/armbianEnv.txt

    # append required overlays to `overlays` key
    CAM_OVERLAY="armsom-sige7-camera-imx415-4k"

    echo "\nModified armbianEnv.txt:\n"
    cat /boot/armbianEnv.txt | sed "s/^\(overlays=.*\)/\1 $CAM_OVERLAY/"

    # ask user to confirm changes
    while true; do
        read -p "Do you wish to apply those changes? " yn

        case $yn in
            [Yy]* ) cat /boot/armbianEnv.txt | sed "s/^\(overlays=.*\)/\1 $CAM_OVERLAY/" | sudo tee /boot/armbianEnv.txt && echo "succesfully written to /boot/armbianEnv.txt"; break;;
            [Nn]* ) echo "changes were NOT applied, please manually configure the required OVERLAYS"; break;;
            * ) echo "Please answer [y/n].";;
        esac
    done

    # update the system
    sudo apt update
    sudo apt install v4l-utils -y

    # download camera driver package and install
    cd ~/Downloads
    wget https://cdn.discordapp.com/attachments/1280594948848357376/1283307783902728192/camera_engine_rkisp_v2.3.0_arm64.deb?ex=672c5902&is=672b0782&hm=2aa29b18bd7c080634deb1e6ce3553c228a84b9c0fa8475981888424f47892c6&
    sudo dpkg -i --force-overwrite camera_engine_rkisp_v2.3.0_arm64.deb
    cd
}

function setup_gpio
{
    # setup gpiod for c++ and python
    sudo apt install gpiod libgpiod-dev python3-dev -y
    pip install gpiod
}

function setup_opencv
{
    sudo apt install python3-opencv
    echo "installed opencv version: "
    python3 -c "import cv2; print(cv2.__version__)"
}

function cofigure_device_tree
{
    dtc -I dtb -O dts -o output.dts /boot/dtb/rockchip/rk3588-armsom-sige7.dtb

    # edit file
    dtc -I dts -O dtb -o new.dtb orangepi.dts

    # backup old dtb file
    sudo cp /boot/dtb/rockchip/rk3588-armsom-sige7.dtb /boot/dtb/rockchip/rk3588-armsom-sige7.dtb.bak

    # copy new file
    sudo cp new.dtb /boot/dtb/rockchip/rk3588-armsom-sige7.dtb

}

function device_tree_overlay
{
    dtc -I dts -O dtb -o camera_overlay.dtbo camera_overlay.dts
    sudo cp camera_overlay.dtbo /boot/overlays/
}

# default setup
update_system
setup_gpio
setup_camera
setup_opencv
reboot
