# Robot Arm Simulator

In this project a 2 DoF robot arm is simulated in a 2D space. The environment has some obstacles in it, which might interfere with robot's movement. The project includes three part:

1. Inverse Kinematics
2. Obstacle Avoidance and Path Planning
3. Motion Actuator

## Inverse Kinematics
The inverse kinematics task is to build in order to transform position from cartesian coordination into robots joint angle coordination. The corresponding code in JavaScripts is demonstrated below:

```javascript
// x and y is the input value in cartesian coordination
let dx = x*x + y*y;
let dl = arm_1.len*arm_1.len + arm_2.len*arm_2.len;

if (dx > dl)
    return;

var q2 = Math.acos((dx - dl) / (2*arm_1.len*arm_2.len));
var k1 = arm_1.len + arm_2.len * Math.cos(q2);
var k2 = arm_2.len * Math.sin(q2);
var q1 = Math.atan2(y, -x) - Math.atan2(k2, k1);

theta_1 = -(q1 - Math.PI);
theta_2 = -(q2 - Math.PI);
```

## Path Planning
In order to avoid obstacles in this code, a RRT path planning is employed. In this method, robot tries to avoid obstacles in joint space. Hence, we can make sure it will never hit any obstacles in its way.

![obstacle view](https://github.com/zaraanry/robotArmSimulator/blob/master/robotArmSimulator.html/obstacle_view.png)

## Motion Actuator
In order to transform robot position from joint space to cartesian space we just simply calculate each joint position node-by-node. The code example is demonstrated below:

```javascript
// Normalizing angles
theta_1 = 6.0 * cursor.x / 400.0 + .07;
theta_2 = 6.0 * cursor.y / 400.0 + .07;

// Calculating joint #1
let eef_x = Math.cos(interest_point.t1) * arm_1.len + arm_1.x;
let eef_y = Math.sin(interest_point.t1) * arm_1.len + arm_1.y;

// Calculating joint #2
let eef_x2 = Math.cos(interest_point.t1 + interest_point.t2 + Math.PI) * arm_2.len + eef_x;
let eef_y2 = Math.sin(interest_point.t1 + interest_point.t2 + Math.PI) * arm_2.len + eef_y;

end_effector_x = eef_x2;
end_effector_y = eef_y2;
```
