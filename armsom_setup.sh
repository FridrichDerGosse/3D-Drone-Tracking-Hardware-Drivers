# --> flash debian to armsom SD card

function setup_camera()
{
    # download camera driver package and install
    cd Downloads
    wget https://cdn.discordapp.com/attachments/1280594948848357376/1283307783902728192/camera_engine_rkisp_v2.3.0_arm64.deb?ex=672c5902&is=672b0782&hm=2aa29b18bd7c080634deb1e6ce3553c228a84b9c0fa8475981888424f47892c6&
    sudo dpkg -i --force-overwrite camera_engine_rkisp_v2.3.0_arm64.deb
    reboot
}

function setup_gpio()
{
    # setup gpiod for c++ and python
    sudo apt install gpiod libgpiod-dev python3-dev -y
    pip install gpiod
}


setup_gpio
setup_camera
