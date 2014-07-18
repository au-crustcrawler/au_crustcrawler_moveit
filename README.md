au_crustcrawler_moveit
======================

Moveit configuration for the crustcrawler ax12 depends on the au_crustcrawler_base package

This package provides a default configuration of moveit for the crustcrawler robot


Play with robot in rviz
-----------

`roslaunch au_crustcrawler_moveit demo.launch`

If it does not succeed check that you have the *au_crustcrawler_base* package also.

Use it with the real robot
-----------
`roslaunch au_crustcrawler_base base.launch`

Wait for the motor to be initialised

In another terminal window

`roslaunch au_crustcrawler_base meta.launch`

and then

`roslaunch au_crustcrawler_moveit real_robot.launch`

