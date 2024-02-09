# perception

```bash
mkdir -p ~/perception_ws/src
cd ~/perception_ws/src
git clone https://github.com/jmguerreroh/perception.git
sudo apt update
sudo apt install python3-vcstool python3-pip python3-rosdep python3-colcon-common-extensions -y
vcs import < perception/thirdparty.repos
pip3 install -r yolov8_ros/requirements.txt
cd ~/perception_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
```
