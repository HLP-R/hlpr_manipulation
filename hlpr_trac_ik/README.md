Note: IK is performed with linear_actuator_link as the BASE LINK!

All goal positions must be w.r.t. the linear actuator, not base_link.

# Service Message
float32[] origin
  - Starting pose in joint space

geometry_msgs/Pose[] goals
  - Goal positions in ee space
  
float32[] tolerance
  - Tolerances for each joint
