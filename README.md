## Project: Kinematics Pick & Place
---
[//]: # (Image References)

[image0]: ./images/DH_parameters.png
[image1]: ./images/DH_Diagram.png
[image2]: ./images/theta1.png
[image3]: ./images/law_cosines.png
[image4]: ./images/theta2_3.png
[image5]: ./images/theta2_3_equations.png
[image6]: ./images/R3_6.png
[image7]: ./images/theta4_6.png
[image8]: ./images/joint2_standard.png
[image9]: ./images/joint2_reversed.png
[image10]: ./images/first_success.png
[scara]:  ./images/scara.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view)

---

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

![alt text][image1]

I used videos 11 and 12 on the Kuka KR210 Forward Kinematics to obtain the DH parameter values.

1. twist angle - alpha(i-1) - angle between Z(i-1) and Z(i) about X(i-1) axis
2. link length - a(i-1) - distance between Z(i-1) and Z(i) along X(i-1) axis
3. link offset - d(i) - distance between X(i-1) and X(i) along Z(i) axis
4. joint angle - theta(i) - angle between X(i-1) and X(i) about Z(i) axis

#### Axis (i-1) -> Axis (i) / (Rotation | Translation) Axis
![alt text][image0]

#### DH Parameter Table
|Joint | twist angle | link length | link offset | joint angle |
|----- | ----------- | ----------- | ----------- | ----------- |
|    1 |           0 |           0 |        0.75 |          q1 |
|    2 |       -pi/2 |        0.35 |           0 |   q2 - pi/2 |
|    3 |           0 |        1.25 |           0 |          q3 |
|    4 |       -pi/2 |      -0.054 |        1.50 |          q4 |
|    5 |        pi/2 |           0 |           0 |          q5 |
|    6 |       -pi/2 |           0 |           0 |          q6 |
|    G |           0 |           0 |       0.303 |           0 |

### SCARA manipulator - DH Kinematic Analysis - Practice
![alt text][scara]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

For each individual transformation matrix, I defined the basic DH transformation matrix in Sympy and then simplified the matrix with the DH parameter values.
The generalized homogenous transform is composed of two primary components - Euler rotation matrix and the End-Effector pose. The Euler rotation matrix is generated from the roll, pitch, and yaw orientation angles and the tf.transformations.euler_matrix function. I defined the symbolic matrix using the standard X, Y, Z rotation matrices in Sympy for the report.

#### Base Link to Joint 1

| cos(q1) | -sin(q1) | 0 | 0    |
| ------- | -------- | - | ---- |
| sin(q1) |  cos(q1) | 1 | 0    |
|      0  |        0 | 1 | 0.75 |
|      0  |        0 | 0 | 1    |

#### Joint 1 to Joint 2

| sin(q2) |  cos(q2) | 0 | 0.35 |
| ------- | -------- | - | ---- |
|      0  |        0 | 1 | 0    |
| cos(q2) | -sin(q2) | 1 | 0    |
|      0  |        0 | 0 | 1    |

#### Joint 2 to Joint 3

| cos(q3) | -sin(q3) | 0 | 1.25 |
| ------  | -------- | - | ---- |
| sin(q3) |  cos(q3) | 1 | 0    |
|      0  |        0 | 1 | 0    |
|      0  |        0 | 0 | 1    |

#### Joint 3 to Joint 4

| cos(q4)  | -sin(q4) | 0 | -0.054 |
| -------  | -------- | - | ------ |
|       0  |        0 | 1 | 1.5    |
| -sin(q4) | -cos(q4) | 0 | 0.75   |
|       0  |        0 | 0 | 1      |

#### Joint 4 to Joint 5

| cos(q5) | -sin(q5) |  0  | 0 |
| ------  | -------- | --  | - |
|       0 |        0 | -1  | 0 |
| sin(q5) |  cos(q5) |  0  | 0 |
|       0 |        0 |  0  | 1 |

#### Joint 5 to Joint 6

|  cos(q6) | -sin(q6) | 0 | 0 |
|   -----  | -------- | - | - |
|        0 |        0 | 1 | 0 |
| -sin(q6) | -cos(q6) | 0 | 0 |
|        0 |        0 | 0 | 1 |

#### Joint 6 to Gripper Link

| 1 | 0 | 0 | 0     | 
| - | - | - | ---   |
| 0 | 1 | 0 | 0     |
| 0 | 0 | 1 | 0.303 |
| 0 | 0 | 0 | 1     |

#### Homogeneous Transformation Base Link to Gripper Link

* R_RPY - Rotation Matrix defined by Euler Angles (roll, pitch, yaw)
* P - Gripper Link current position

| R_RPY | P  |
|  ---  | ---|
|    0  | 1  |

|c2*c3 | s1*s2*s3 - s3*c1 |  s1*s3 + s2*c1*c3 | px |
| ---  | ---------------- |  ---------------- | -- |
|s3*c2 | s1*s2*s3 + c1*c3 | -s1*c3 + s2*s3*c1 | py |
|  -s2 |            s1*c2 |             c1*c2 | pz |
|    0 |                0 |                 0 | 1  |

* c - cos
* s - sin
1. Roll
2. Pitch
3. Yaw

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and Inverse Orientation Kinematics

#### Inverse Position
##### Theta 1
![alt text][image2]

##### Law of Cosines
![alt text][image3]

##### Theta 2 + 3
Joints 2 and 3 rotate in a clockwise manner starting approximately from a 90 angle. The initial angle between Joint 3 and Joint 5 (Wrist Center) is shifted by a small factor because of Joint 4.
I compensate for this small shift using a correction factor - C calculated from the initial pose. I relied heavily on the law of cosines to calculate the angles for the triangle formed between Joint 2,3,5. The angles for joints 2+3 are defined using an Elbow-Up two link configuration.

![alt text][image4]
![alt text][image5]

#### Inverse Orientation
There are two primary scenarios for the spherical wrist - (Singular OR Not-Singular). The rotation matrix R3_6 is singular, when theta5 = 0. Therefor, the robot is in a singular condition when R10 = 1, R00 = 0, R20 = 0, R11 = 0, R12 = 0. In this scenario, we can derive the sum of theta 4 and 6 from the rotation matrix R3_6. I store the previous set of orientation angles for theta 4,5,6. Using the previous orientation angle and theta 4+6, I generate a reasonable configuration for the spherical wrist that avoids unneccessary movement. If the spherical wrist is not in a singular condition, the joint configuration is more straight-forward to derive from the rotation matrix R3_6. The main choice in this scenario is whether theta5 is positive or negative. I generate configurations for both choices, and then I choose the one that minimizes the difference between the previous joint state.

##### R3_6 with Gripper Correction
![alt text][image6]

##### Theta 4 - 6
![alt text][image7]

### Project Implementation

Succcess: Storing the previous joint angles for the spherical wrist reduced the ambiguity in defining joint angles for the singular and non-singular scenarios.

Failure: 
 * The inverse kinematics can generate inefficient motion paths 
 * The robot cannot determine whether joint 2 is inline with the base link and end-effector.
 
### Standard Configuration - Subtract a1
 ![alt text][image8]
 
### Reversed Configuration - Add a1
 ![alt text][image9]
 
Improvements: 
 * Better coordination between the inverse kinematics and motion planning services could improve the effiency of the robot's movement.
 * Tracking the positions of all the robot's joints could determine if the joint 2 is inline with the rest of the robot.

Here is an image of my first successful pick-and-place operation:

![alt text][image10]