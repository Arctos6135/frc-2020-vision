# frc-2020-vision

Official repository for Team Arctos 6135's vision code in FRC 2020 Infinite Recharge! Runs on a NVIDIA Jetson TX1 with ROS.

Special thanks for ROS help from [@mincrmatt12](https://github.com/mincrmatt12).

The following images were generated with the Python tests:
![Target position tracking (Python)](https://user-images.githubusercontent.com/32781310/107910212-f0d79500-6f27-11eb-8368-efb82aa2f47d.png)
![Target detection (Python)](https://user-images.githubusercontent.com/32781310/107910131-bcfc6f80-6f27-11eb-8e10-57aa3d1b706a.png)

**Unfortunately, because we switched to a Limelight midway through (*not because it's better, but because it's easier to use for future programming members*), the code was never finished. However, the Python tests are fully functional. You can run `png_vision_test.py` to test on the local static images, or `vision_test.py` if you have a real camera and target available.**

## Building

### Installing Dependencies

#### Main Project

This project natively uses ROS Kinetic on Ubuntu Xenial, but any distribution should work.
This project is powered by ROS. First install ROS from http://wiki.ros.org/ROS/Installation.

1. Source the setup file from `/opt/ros/<distro>/setup.bash`.
2. Run `wstool update -t src` to clone the necessary packages from source.
3. Run `rosdep install --from-paths src -i` to install all package dependencies.

#### Python Tests

To install the dependencies for the Python tests, run `pip install -r python-tests/requirements.txt`.

### Building the Project

1. Run `catkin build` to build the project.

## Running

### Main Project

A USB camera (Microsoft Lifecam HD-3000) should be at `/dev/video0`, with a green LED ring (am-3597) around it.

Again, you must have ROS installed to run this project.

1. Source the local setup file from `devel/setup.bash`.
2. Run `roslaunch bot bot.launch` to launch the entire robot.

### Python Tests

For live vision tests, a USB camera (Microsoft Lifecam HD-3000) should be at `/dev/video1` (since usually `/dev/video0` is the built-in webcam), with a green LED ring (am-3597) around it.

To run the Python tests, first `cd python-tests`.
To run the live vision tests, run `python3 vision_test.py`.
To run the PNG vision tests, first extract `test_imgs.tar.gz`, and then run `python3 png_vision_test.py`.

When running PNG vision tests, press 'q' to quit, press 'p' to go to the previous image, or press any key to move on to the next image.

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

