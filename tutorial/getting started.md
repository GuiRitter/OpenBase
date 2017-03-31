1. Copy the folders ````model/abstract```` and ````models/omni_wheel```` to Gazebo's default model folder.

2. Copy the plugin folder ````plugin/demo```` to somewhere.

3. Since the STL files now have a new path, open both folders' ````model.sdf````, search for occurrences of ````.stl```` and replace the paths for the corresponding path of each STL file.

4. Create a new world in Gazebo (paused) and insert the model. It should be available in the **Insert** tab since you placed the files in Gazebo's default model folder.

5. Place the robot at **(x, y, z) = (0; 0; 0,01905)**.

6. Save the world and close Gazebo.

7. Compile the plugin, following either Gazebo's tutorial or my tutorial using NetBeans.

8. Open the world file with a text editor. Search for ````<plugin name=```` and replace the path after ````filename=```` by the new plugin path.

9. Open the world in Gazebo and hit play. The robot should start moving: it will repeatedly decide on a random **(x, y, θ)** and move to it.

10. If you want to change the robot's behaviour, follow Gazebo's tutorials as explained in the **README** and set the plugin path like mentioned above as necessary.

10. If you want to change the robot itself, like adding actuators and sensors to it, follow my tutorial on how to make a new model. How to work on the rest of the robot is up to you, but you can copy my plugin's code and paste it in your plugin's code and just call the movement methods to move the robot.
