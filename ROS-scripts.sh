# Run this command on every new shell you open to have access to the ROS 2 commands
source /opt/ros/humble/setup.bash

# If you don’t want to have to source the setup file every time you open a new shell (skipping task 1), then you can add the command to your shell startup script:
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Check environment variables
printenv | grep -i ROS
