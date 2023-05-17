# zomby_startup
This will be a package of a node that can be run by the zomby robot to listen for cmd_vel on the ros network and send messages over usb cord to the arduino

## Installation

Ensure ros2 is downloaded and installed on your computer. If not, follow the instructions here: https://index.ros.org/doc/ros2/Installation/

install git
```
sudo apt install git
```

make a directory if not already done
```
mkdir -p ~/ros2_ws/src
```

clone this repository into the src folder
```
cd ~/ros2_ws/src
git clone https://github.com/marksoulier/zomby_startup.git
```

build the package
```
cd ~/ros2_ws
colcon build --packages-select zomby_startup
```

## Usage

To run the node, open a terminal and run the following command:
```
ros2 run zomby_startup zomby_startup
```

To control the zomby robot with a keyboard, open a new terminal and run the following command:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```


