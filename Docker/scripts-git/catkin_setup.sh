echo Starting catkin workspace setup

cd ~/
mkdir catkin_ws
mkdir catkin_ws/src
cd catkin_ws

echo Running catkin_make in workspace...
source /opt/ros/melodic/setup.sh
catkin_make >> /dev/null


echo Cloning into workspace...
git clone --branch $GIT_BRANCH $WORKSPACE_GIT
cp -r $WORKSPACE_GIT_NAME/board_interface src/board_interface
cp -r $WORKSPACE_GIT_NAME/watchdog src/watchdog

echo Cleaning workspace...
catkin_make clean >> /dev/null

echo Finished catkin workspace setup