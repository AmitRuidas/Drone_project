# Drone_project
It is the Complete Drone Project in Gazebo Simulation using ROS1 and PX4 (keyboard Control simulation with autonomous mood)


1st you created a work_space in src folder you clone the entire program

after that you download px4 accroding to your ros version then make/compile the px4 on this command "make px4_sitl gazebo" and you can also got it on there site

after successful complition of compile you can change two things one is  "iris_rplidar.sdf" i also paste it on src folder you copy and paste it and another file is "mavros_posix_sitl.launch" paste it from my src folder [sdf file you can find in px4 models folder, and the px4-launch file you can find in px4-launch folder copy and paste those two file 

afte completing all of those you go to catkin_ws(root) then finally compile on catkin_make command

all of those you just run "roslaunch drone_pkg start_offb.launch" command in you terminal on the root directory then you can find the boom things.............
