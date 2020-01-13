# frc-2020-vision

Official repository for Team Arctos 6135's vision code in FRC 2020 Infinite Recharge! Runs on a NVIDIA Jetson TX1 with ROS.

~~We're trying hard to not rely on anyone, but this project wouldn't be possible without [@mincrmatt12](https://github.com/mincrmatt12).~~

## Building

### Installing Dependencies

This project natively uses ROS Kinetic on Ubuntu Xenial, but any distribution should work.
This project is powered by ROS. First install ROS from http://wiki.ros.org/ROS/Installation.

1. Source the setup file from `/opt/ros/<distro>/setup.bash`.
2. Run `wstool update -t src` to clone the necessary packages from source.
3. Run `rosdep install --from-paths src -i` to install all package dependencies.

To install the dependencies for the Python tests, run `pip install -r python-tests/requirements.txt`.

### Building the Project

1. Run `catkin build` to build the project.

## Running

Again, you must have ROS installed to run this project.

1. Source the local setup file from `devel/setup.bash`.
2. Run `roslaunch bot bot.launch` to launch the entire robot.

To run the Python tests, first `cd python-tests`, and then run `python3 vision_test.py`.

## Thank you to our generous sponsors:
### Platinum
<img alt="TDSB" src="https://upload.wikimedia.org/wikipedia/en/thumb/6/60/Toronto_District_School_Board_Logo.svg/1200px-Toronto_District_School_Board_Logo.svg.png" height="400"/>

### Gold
<img alt="The Intuitive Foundation" src="https://images.squarespace-cdn.com/content/v1/575036b345bf2183563cd328/1564584203054-4XAQJMKZAM1FZKPP71ST/ke17ZwdGBToddI8pDm48kElPbZlriv4ByvPLLYTs3rRZw-zPPgdn4jUwVcJE1ZvWhcwhEtWJXoshNdA9f1qD7XxG-9FZQiNMT_ZdcQnlMXbFYWqAe63cqij5R0iA9W7XX4KjGb09mhyQhiOJiRgdGQ/Intuitive+Foundation+Logo.png"/>
<br/>
<img alt="SNC-Lavalin" src="https://upload.wikimedia.org/wikipedia/en/thumb/5/50/SNC-Lavalin_logo.svg/470px-SNC-Lavalin_logo.svg.png"/>
<br/>
<img alt="Ryver" src="https://ryver.com/wp-content/themes/bridge-child/images/logo-dark-2017.svg" width="500"/>

### Silver
<img alt="The Maker Bean Cafe" src="https://user-images.githubusercontent.com/32781310/52224389-eaf94480-2875-11e9-82ba-78ec58cd20cd.png" width="300">

### Bronze
<img alt="Arbor Memorial" src="https://www.cbc.ca/marketplace/content/images/Arbor_Logo.jpg" height="100"/>

