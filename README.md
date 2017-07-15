## Project: Kinematics Pick & Place
---
[//]: # (Image References)

[image1]: ./images/DH_Diagram.png
[image2]: ./images/theta1.png
[image3]: ./images/law_cosines.png
[image4]: ./images/theta2_3.png
[image5]: ./images/theta2_3_equations.png
[image6]: ./images/R3_6.png
[image7]: ./images/theta4_6.png
[image8]: ./images/first_success.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

![alt text][image1]

1. twist angle
2. link length
3. link offset
4. joint angle

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

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

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

Apply URDF to DH convention correction to homogeneous transformation

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and Inverse Orientation Kinematics

#### Inverse Position
##### Theta 1
![alt text][image2]

##### Law of Cosines
![alt text][image3]

##### Theta 2 + 3
![alt text][image4]
![alt text][image5]

#### Inverse Orientation
##### R3_6 with Gripper Correction
![alt text][image6]

##### Theta 4 - 6
![alt text][image7]

### Project Implementation

Succcess:

Failure:

Improvements:


Here is an image of my first successful pick-and-place operation:
![alt text][image8]


