# Main_Project
This repo is the main project where all source codes in relation to project 1- Spot Search n rescue will come to. This repo is for group "Robodawgs" 2024Spr Industrial Robotics



For the model

1. gedit ~/.bashrc
2. Add: export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/pathto/Main_Project/models
3. Close the text editor 
4. source ~/.bashrc









For navigation 

1. Launch environment: export TURTLEBOT3_MODEL=waffle_pi 
                       ros2 launch turtlebot3_gazebo turtlebot3_warehouse.launch.py 
2. Run cylinder detector: ros2 run main_project circle_detector
3. Run environment navigation: ros2 run Main_Project EnvNavigation
4. Launch Rviz: launch rviz turtlebot3_navigation2 navigation2.launch.py map:=$HOME/git/src/Main_Project/maps/warehouse.yaml
5. Localise bot in rviz (2D pose estimate)
6. Add injured path topic in rviz
7. Set buffer length to 1000 in rviz





For navigation condensed a launch file

ros2 launch main_project project.launch.py

Instructions for SPOT
1. Clone and install all dependencies

	sudo apt install -y python3-rosdep
	rosdep update

	cd ros2_ws/src
	git clone --recursive https://github.com/chvmp/champ -b ros2
	git clone https://github.com/chvmp/champ_teleop -b ros2
	cd ..
	rosdep install --from-paths src --ignore-src -r -y

2. Build your workspace

	cd ros2_ws
	colcon build
	source install/setup.bash
	
3. We need to change the following files. Please find the full version of the files in the file_mods folder. Copy and paste from these files to make the changes. The files we need to change are:

	gazebo.launch.py (The launch file for gazebo - /home/student/ros2_ws/src/champ/champ_config/launch)
	outdoor.world (The outdoor world file - /home/student/ros2_ws/src/champ/champ_config/worlds)
	navigation.rviz (The pre-configured rviz setup - /home/student/ros2_ws/src/champ/champ_navigation/rviz)
	champ.urdf.xacro (Describes the physical properties of the robot - /home/student/ros2_ws/src/champ/champ_description/urdf)
	
4. Once these files have been changed, make sure to build and source the workspace
	
	cd ~/ros2_ws
	colcon build
	source install/setup.bash
	
NOTE. If you havent already, symbolically link Main_Project to your ros2_ws/src:
	
	cd ~/ros2_ws/src
	ln -s ~/git/Main_Project
	
	
5. The launch file for the SPOT is included in the Main_Project git. Call it with:

	ros2 launch main_project project_spot.launch.py

6. Once Rviz has launched, perform the 2D Pose estimate. There should be a red arrow from the base of the robot, do the pose estimate in this direction.

7. The robot will then begin navigating to the pre-defined route.

8. If this doesn't work, or you want to launch everything seperately, do it in this order. Please note that if using the luanch file, it will only work once (restarting will solve this issue).

	ros2 launch champ_config gazebo.launch.py gui:=False headless:=True (Launches Gazebo)
	ros2 run main_project spot_the_circles (Runs the object detection)
	ros2 launch champ_config navigate.launch.py rviz:=true (Launches Rviz)
	ros2 run main_project spot_navigation (Publishes the route and starts the navigation)
	(You then need to do the 2D pose estimate)
	
9. Enjoy and get full marks baby
	
	















