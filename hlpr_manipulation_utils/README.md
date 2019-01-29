# HLPR Manipulation Utilities

This package contains several helpful utilities for moving robots and manipulating objects.

Currently, the best-supported scripts in this package are ArmMoveIt 2, Gripper, and PickPlace.

## ArmMoveIt 2


There is more complete documentation on the [GitHub Pages Site](https://hlp-r.github.io/hlpr_documentation/).

ArmMoveIt 2 is a wrapper around MoveIt specifically for our Kinova arms. Here is a minimal working example of how to use ArmMoveIt 2 with the Kinova 7-DOF arm:


```python
import math
import rospy
from arm_moveit2 import ArmMoveIt
import hlpr_manipulation_utils.transformations as Transform
from geometry_msgs.msg import Pose

arm = ArmMoveIt(planning_frame="j2s7s300_link_base")
target = Pose()
target.position.x = 0.2
target.position.y = 0.2
target.position.z = 0.2
target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w = \
    Transform.quaternion_from_euler(0.0, math.pi/2, math.pi/2)

arm.move_to_ee_pose(target)
```

ArmMoveIt 2 has several capabilities, such as producing plans without moving, doing joint motions instead of eef motions, etc. It is well documented, see arm_moveit2.py for more details.


## Gripper

Control a Robotiq 85 2-finger gripper. A minimal example is:

```python
import time
from hlpr_manipulation_utils.manipulator import Gripper

gripper = Gripper()

gripper.open()
time.sleep(2.0)

gripper.close()
time.sleep(2.0)
```

Note that this class **does not block**. You must add a sleep after each call if you want to wait for the motion to finish. 2 seconds is enough for any motion.


## PickPlace

### Overview
An easy way to pick and place objects with a robot. This class requires several other things to be running to work correctly. Most notably, it requires frames to be published at the position of any objects that you want to manipulate. These frames are automatically provided if you are using the ORP object detector (contact Adam Allevato for more details).

PickPlace uses ArmMoveIt2 and Gripper to control the robot. The data flow is therefore as follows:

```
Obj Detector => Vision Frames
                      |
                      v
     Your code => PickPlace => ArmMoveIt 2 => MoveIt => Kinova Driver => Kinova
                      |
                      v
                   Gripper => Robotiq Driver => Robotiq
```

### Video
[This video](https://youtu.be/mn2rNa-sdmI) shows this class in action.

### Test Via Command Line
You can use the class in an interactive command line (as in the video) by running `rosrun hlpr_manipulation_utils pick_place.py`. This could be useful for quick-and-dirty demos.


### Minimal Example

```python
from hlpr_manipulation_utils.pick_place import PickPlace

pick_place = PickPlace()
pick_place.pick_center("obj_frame")
pick_place.place()
```

### More Complete Example with Notes

```
from hlpr_manipulation_utils import PickPlace

# PickPlace has several arguments that modify its behavior. See pick_place.py for more details.
pick_place = PickPlace()

# Open the gripper, approach the object, move to it, close the gripper, and move back up. This assumes that obj_frame is the frame locating the object you want to pick.
pick_place.pick_center("obj_frame")

# this will put the object down at the same place from which it was picked.
pick_place.place()

# this command will fail, because you can't pick until you have placed. This behavior can be modified via the require_place argument.
pick_place.place()

# This does the same thing as pick
pick_place.pick_rim("obj_frame")

# You can access the ArmMoveIt instance via pick_place.arm. After moving, place() will
# respect the new XY position of the arm.
pick_place.place()
```
