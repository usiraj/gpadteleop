gpadteleop
==========

GamePad teleoperation for robots using ROS

Use gpadteleop_sameaxis.launch to launch gamepad teleoperation node.
Before starting make sure that robot_cmd_vel argument of launch file is pointing to the right topic for your robot.
Before moving your robot through gamepad, first enable robot using the appropriate button
on your game pad( Button 4 by default )
If you lock your robot ( by pressing lock button or hard brake button ) you need to unlock
the robot (Button 4 by default) before commanding your robot.
Locking the robot ( Button 4 by default ) starts sending zero robot velocity commands to
robot, unless the robot is unlocked by pressing Lock/Unlock Button (Button 4 by default)
GPadTeleop2 uses dynamic reconfigure, so parameters can be reset according to your needs when the node is running.

More information can be accessed at the corresponding project wiki.
For feature requests and bug reporting feel free to contact osama@smme.edu.pk

Note:
	If rosmake fails. There is an issue with rosmake that requires messages to be generated
	first before generating cfg file. A way to bypass this problem is to comment out 
	gencfg() and in CMakeLists.txt and do rosmake. Then redo rosmake after uncommenting
	out the lines you commented. This will resolve the issue and rosmake will succeed.
