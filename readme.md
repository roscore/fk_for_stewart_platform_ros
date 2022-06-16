## Summary
This package is Forward kinematics for Stewart Platform on ROS Noetic

## Usage
```
rosrun fk-for-stewart-platform-ros fk.py
```

The period of the node is 100 Hz, and when all the motor values are updated, the current position is calculated and broadcast to the topic.

*input topic
  ```
  /motor_{number}/joint_states ## JointState type
  ```
  
*output topic
  ```
  /stewart/curr_pos           ## Twist type
  ```


## Origin Repo
fork at https://jak-o-shadows.github.io/electronics/stewart-gough/stewart-gough.html
