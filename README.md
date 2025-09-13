### Install y1_sdk
cd y1_sdk/
pip install .

### use y1_sdk  
cd y1_ros/
bash build.sh

source devel/setup.bash
roslaunch y1_controller single_arm.launch