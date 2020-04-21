This file represents the complete project of my thesis.

At first the global path planner, which uses RRT* method,
generates some way points. The job of global path planner
is to find any possible path that reaches the goal.
In global path planner a single ackerman model of 
the robot is used to have a fast path planner.

Then the local planner uses the generated way points and
produces a high quality path for the robot. In this module
a more sophisticated model, four wheel steering bicycle model, 
is used for generating a high quality path for our robot.

The final module, which is named Animation Builder, builds 
a gif of the robot movement along the path.