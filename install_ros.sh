sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' -y
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 -y
sudo apt-get update -y
sudo apt-get install ros-kinetic-desktop-full -y

sudo rosdep init -y
rosdep update -y

source /opt/ros/kinetic/setup.bash -y
sudo apt-get install python-rosinstall -y

source /opt/ros/kinetic/setup.bash -y
mkdir -p ~/catkin_ws/src -y
cd ~/catkin_ws/src -y
catkin_init_workspace -y
cd ~/catkin_ws/ -y
catkin_make -y
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc -y
source ~/.bashrc -y