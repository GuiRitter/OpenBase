1. I adapted [this tutorial](http://gazebosim.org/tutorials?tut=inertia&cat=) for this specific scenario, so it may be a good idea to read it before doing this, since it's illustrated among other things.

2. Also read my tutorial on how to compile the plugin to control this robot.

3. Draw the body on the modelling tool of your choice. Follow these constraints:
  1. The geometric center of the body must lie on the center of the coordinate system.
  2. The wheels' axes must lie on the **XY** plane and be angled **120°** between each other.
  3. The geometric center of each wheel must have the same distance to the geometric center of the body.
  4. The up side of the body should be towards the positive **Z** axis.
  5. To match the already implemented kinematics, the axis of one wheel must be colinear with the **Y** axis, and this whell must lie on the negative side of the same axis.

4. Note down the **XYZ** coordinates of each wheel center and their angle.
  1. Measure on the **XY** plane the angle between the **X** axis and the wheel rotation axis.

5. If your modelling tool is capable of measuring mass, volume and moment of inertia, do so and note down the values (in meters and kilograms). I'll show later how do to that with [FOSS](https://en.wikipedia.org/wiki/Free_and_open-source_software) tools, but you can probably get more precise results and/or compare both results for validation.
  1. If you use [SOLIDWORKS](https://en.wikipedia.org/wiki/SolidWorks) (it probably works with other tools too), you can first change the options to show all values with as much decimal places as possible, then get the values in meters and kilograms (the actual values) and then get the values in millimeters and grams to get even more decimal places (I'm a bit paranoid about precision, so feel free to ignore that).

6. Export the model to [STL](https://en.wikipedia.org/wiki/STL_(file_format)).

7. Import the STL file in [Blender](https://www.blender.org/).

8. **Set Geometry to Origin** (default **Shift + Ctrl + Alt + C**).

9. Make sure the scale is right. Gazebo works in meters and kilograms. For example, if you made your model in SOLIDWORKS in the MMGS system (millimeter, gram, second), you must divide the entire model by **1&thinsp;000**.

10. Also make sure it's properly aligned in all axes (**Z** is up).

11. Export the model as STL and close Blender.

12. Import the model in [MeshLab](http://meshlab.sourceforge.net/).

13. Click in **View** > **Show Layer Dialog**, so you can see a terminal output on the lower right of the screen.

14. Click on **Filters** > **Quality Measure and Computations** > **Compute Geometric Measures**.

15. In the terminal, look for a 3×3 matrix below ````Inertia Tensor is :````. These are the values for the model's moment of inertia. Note them down.

16. If the values are too small, you can scale the model to see them.
  1. Choose a value for **s** that is a power of **10**. **100** should be enough.
  2. Click on **Filters** > **Normals, Curvatures and Orientation** > **Transform: Scale**.
  3. In **X|Y|Z Axis**, input **s** and click **Apply** then **Close**.
  4. Click on **Compute Geometric Measures** (see above).
  5. Repeat these steps if the values still are too small. Remember that, for example, if you initially used **s = 10** and used the same value again, **s** is now **100**.
  6. After getting values with enough precision, divide them by **s<sup>5</sup>**.
  7. Multiply the result by the body's mass. You'll have to either measure it or compute it from the volume (see below) and a desired density.
  8. Divide them by the volume. You can get the volume from the terminal, after **Mesh Volume is** . Remember that, if you scaled the model, you have to divide the volume by **s<sup>3</sup>** before dividing the moment of inertia by the volume.

17. Every element of the moment of inertia's matrix corresponds to the element in the same position in the following matrix:
    ```
    ixx  ixy  ixz
    ixy  iyy  iyz
    ixz  iyz  izz
    ```
    Note down which value is which variable.

18. Close MeshLab, open it again and import the file again, to reset any changes made to it.

19. Export the file. This is done to fix a rendering error in Gazebo. Close MeshLab again.

20. Open Gazebo (new world). I like to use the parameters ````-u```` (start paused) and ````--verbose```` (more numerous and detailed messages).

21. Enter **Model Editor**.

22. On the **Insert** tab, below **Custom Shapes**, click **Add**. Find the latest body STL file and name the link if you wish. Click **Import**.
  1. The model will follow the mouse pointer until you click to set its position. The model for this robot will probably be too small to see, so zoom in before clicking to place it in the world.

23. Right click the body link and click **Open Link Inspector**, or double click the link.

24. In the **Link** tab, in the **Pose** folder (not **Inertial > Pose**), set the link's position to **(0, 0, 0)**.

25. Click OK.

26. Add 3 of the omni wheels that I provided close to their mounting points. They should be appearing in the **Insert** tab, in the **Model Database** panel, in Gazebo's standard model folder.

27. Note down the wheels' link names.

28. Create 3 joints. Nevermind that the wheels are not positioned yet.
  1. The parents are the body model and each child is each wheel rim (make sure to select the rim, not the rollers). 
  2. In **Joint Axis**, select **Z** in the drop down menu.
  3. In **Joint Pose**, in **Pitch**, enter ````-1,570796```` **(−π/2)**;

29. Note down which joint corresponds to which wheel, as it will be necessary to make the correct references for the kinematics to work. You'll be able to change their names soon.

30. In the **Model** tab, in **Model Plugin**, click add, write anything in **Plugin name** and **Filename** and click ok.

31. Add whatever else you want to be in your robot. How these will work is up to you.

32. Exit **Model Editor** and save the model in Gazebo's standard model folder.

33. Delete the model.

34. Browse to Gazebo's standard model folder, open the folder for the model you just saved and open ````model.sdf```` with a text editor.

35. Two of the first things you'll see are the body link's mass and moment of inertia. The reason you'll input the values here is because you can use higher precision here than in Gazebo's GUI. The values will be truncated to a lower precision once a world is saved with this model (still higher than in the GUI), but at least very small values will be translated to scientific notation, which also can't be used in the GUI. Pay attention that the GUI only accepts values with a comma for a decimal mark, while the [SDF](http://sdformat.org/) file only accepts a period, like most programming languages.

36. Search for ````<model name=```` and look for those corresponding to each wheel link name. The pose for each wheel is located inside a ````<pose frame=```` element that is the last thing inside each model element. This means that, to find a wheel's ````<pose frame=````, you must first search for the wheel's ````<model name=````, then search for ````</model>```` from that position, and the ````<pose frame=```` will be right above it.

37. Replace the values by those you noted down when you modelled the body. They are ordered: x, y, z, rotation around the x, y, and z axis.

38. If you want to change the names of the joints as mentioned earlier, search for ````<joint name=```` without quotes. In the next line, there's a ````<parent>```` element that references the wheel link's name, so you know which are the links we want.

39. Save the file and insert the model (since you saved in the right folder, it should appear in the **Insert** tab). Check if the body and the wheels are correctly positioned and rotated. 
  1. For example, if you failed to set the body's position in the Model Editor (actually, when I made this tutorial my changes in the GUI were always ignored), you can change it in the SDF in the first ````<pose frame=''>```` element that you see, and set all 6 values to zero.
  2. If the wheels are rotated wrongly, or seeminly right but inside out, just play around with the angle values (the hexagonal hole is the inside of the wheel and should face the body).
  3. Delete the model and insert it again to check everything. Keep repeating these steps until it's as you want it.
  4. Keep in mind that STL changes requires a Gazebo restart.

40. When everything is ok with the model, build the world as it will be necessary for your simulations, with the model you just built, save this world and close Gazebo.
  1. The plugin I developed assumes the robot starts from **(x, y, θ) = (0, 0, 0)**. If, in the world you developed, the robot starts elsewhere, you'll need to edit the source code and alter ````xw```` and ````yw````'s initial value. Otherwise, you need to place the robot at  **(x, y) = (0, 0)**.
  2. Also, there's a distance of **0,01905 m** between the wheel's center (also the robot's center if the body is symmetrical) and the point where it touchs the ground. If the robot will start in the Gazebo's default ground plane and the body is symmetrical like the one I provided, then that value must be the robot's **z** in Gazebo.

41. Open the plugin's source code go to the ````Load()```` method. Replace the parameters of the ````GetJoint()```` method calls to the joint names you noted down before. The same lines have the variables that represent each wheel, and I've named them as I named the wheels: ````left````, ````back```` and ````right````. Rename these as you need.

42. If the geometry of your robot does not match mine, change the value of the distance between the robot's center and each wheel's center in the ````#define L```` macro.

43. Also go to the ````forwardKinematicsMobile()```` and ````inverseKinematicsMobile()```` methods and change the kinematics equations. Obtaining these equations is left as an exercise to the reader.

44. Compile the plugin. If necessary, call some movement methods in the ````Load()```` method to test the robot before compiling.

45. Open the world SDF file you created earlier with a text editor.

46. Search for ````<plugin name=```` and replace the filename by the path to the ````*.so```` file generated from the plugin.

47. Open Gazebo with the path to the world file passed as another parameter.

48. Test the robot.
