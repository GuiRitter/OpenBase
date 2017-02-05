You can build a robot using my robot as a base. It comes with commands that you can use to move it without having to worry on how it will be done. I'll explain the commands I implemented:

````fireMovementDirectWheel(double Vleft, double Vback, double Vright)````

Direct control of each wheel's speed, in meters per second. Don't use a value higher than Vmax, or Gazebo will not work correctly.

````fireMovementDirectWorld(double Vxw, double Vyw, double omegap)````

Sets the translation and rotation (radians per second) speeds, according to the world frame. For example, if the positive side of the Y axis is north, passing Vyw = 1 will always make the robot move north, regardless of it's current angle. Again, don't use values higher than Vmax for translation and omegamax for rotation speeds.

````fireMovementDirectMobile(double Vxm, double Vym, double omegap)````

Sets the translation and rotation speeds, according to the robot's frame. For example, passing Vym = 1 will always make the robot move away from its back wheel, regardless of it's current angle.

````fireMovementDirectHybrid(double Vxm, double Vym, double omegap)````

Transforms the translation speeds from the robot's frame to the world's frame, then sets the translation and rotation speeds, according to the world's frame. For example, fireMovementDirectHybrid(0, 1, 10) with θ = 0 will be the same movement as fireMovementDirectWorld(0, 1, 10), but with other θ values it will be the same movement rotated according to the initial angle between reference frames.

````fireMovementAbsoluteW(double x, double y, double theta)````

Makes the robot move towards a fixed position in the world frame, with the given angle. For example, passing (0, 0, 0) will always make the robot return to the starting pose, regardless of it's current pose.

````fireMovementAbsoluteM(double x, double y, double theta)````

Makes the robot move towards a fixed position in the robot's frame. Actually translates the desired pose to the world frame, and moves according to the world frame.

````fireMovementAbsoluteMRaw(double x, double y, double theta)````

Makes the robot move towards a fixed position in the robot's frame. Because the position is not translated to the world frame, you can't predict where the robot will end in the world frame, because it depends on how fast the robot rotates from its current angle to the desired angle. I'm not sure how useful is this, but I've left it anyway because it's a natural development from the world frame method.

````fireMovementRelativeW(double x, double y, double theta)````

Makes the robot move the given distance and rotate the given angle in the world frame. For example, if the positive side of the Y axis is north, passing y = 1 will always make the robot move 1 meter north, regardless of it's current angle.

````fireMovementRelativeM(double x, double y, double theta)````

Makes the robot move the given distance and rotate the given angle in the robot's frame. For example, passing y = 1 will always make the robot move away from its back wheel, regardless of it's current angle.

````fireMovementRelativeMRaw(double x, double y, double theta)````

Same thing as fireMovementAbsoluteMRaw(), but relative.

````
fireMovementBezierW(std::vector<Point> * points, std::vector<double> * angles, double step, bool offsetT, bool offsetR)
fireMovementBezierM(std::vector<Point> * points, std::vector<double> * angles, double step, bool offsetT, bool offsetR)
````

Makes the robot travel a path described by a Bézier curve [[1]](https://en.wikipedia.org/wiki/B%C3%A9zier_curve)[2]. The control points can be treated like absolute (offsetT = false) or relative (offsetT = true) positions in any frame (W for world frame, M for robot's frame). For relative poses, the first point is translated to the robot's current position, and the remaining points are translated (and rotated, when using the robot's frame) in order to maintain the curve's original design.

The robot's angle can also be controlled by a Bézier curve. It works like an unidimentional Bézier curve. So, instead of a list of control points, you pass a list of control angles. The angles can also be relative (offsetR = true). In this case, the first control angle is considered the robot's current angle. A list must be passed with at least one value in it. Use a zero with offsetR = true for no rotation.

The robot will follow the curve starting from the first point. How fast it will go is determined by the step parameter, although it won't go faster than Vmax. If step is too high, the curve generation speed will be faster than the robot can move, so it will not be able to traverse the curve correctly. The slower the value, the more closely the robot will follow the curve. This parameter also indirectly controls how much time it should take to traverse the curve, and that equals step divided by the plugin's iteration time, which in Gazebo is 1 millisecond. Play around with this value until the robot can perform the movement that you desire.

## References

[1] https://en.wikipedia.org/wiki/B%C3%A9zier_curve

[2] D. F. Rogers and J. A. Adams, Mathematical Elements for Computer Graphics, 2nd ed. New York City: McGraw-Hill, 1990.
