# RobotPlatooning

Software package developed using ROS libaries to control Pioneer 3DX. The package features two programs. Program one controls the master robot which works by maintaining a distance to the left wall and always making left turns when available. The second program controlls the follower robot which tracks the location of the leader robot and attempts to maintain a following distance using a PD controller.

Video Example of Execution

<a href="https://www.youtube.com/watch?v=oRKcFJu4yQA" target="_blank"><img src="https://www.youtube.com/watch?v=oRKcFJu4yQA" alt="CLICK ME TO WATCH VIDEO" width="240" height="180" border="10" /></a>

Contributors:

Follower Robot:

  Bart Kerfeld (responsible for processing of laser data to detect leader)
  
  Lucas Kramer (responsible for implementing PD controller to set speeds a
  
Master Robot:

  Gillian Mcdonald (responsible to coming up with sector idea)
  
  John Harwell (implemented idea)
