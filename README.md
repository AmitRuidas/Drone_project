# Drone_project
It is the Complete Drone Project in Gazebo Simulation using ROS1 and PX4 Driver & Mavros
i uploaded it in the whole src file 
you just need to create a workspace and paste my src folder 
chnage the path name in launch file 
and then install the px4 through px4 site 
copy and paste my model like differential_drive/camera_model etc and make the px4 on the command "make px4_sitl gazebo"
finally you are in root directory and launch the whole nodes [roslaunch package name in my case "drone_pkg" you write the same and the file name start_offb.launch]
[roslaunch pkg name file name]
roslaunch drone_pkg start_offb.launch
your drone on gazebo operate on keyboard and also have a vehicle in simulation to follow the vehicle through fvp_camera and rplidar
