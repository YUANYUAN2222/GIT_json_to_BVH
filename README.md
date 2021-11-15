# GIT_json_to_BVH

## Introduction

The transform code was based on the original work 'Video to 3DPose and BVH motion file'(https://github.com/HW140701/VideoTo3dPoseAndBvh), which only could convert 17 body joints coordinates into the BVH motion file.
We rewrote part of code making the original code could transform the number of 15 or 20 body joints into BVH files.

### Construction Dataset structure
#### 15 joints Skeleton data structure           
Frame #, P(1),P(2),P(3),...,P(15)，<br/>
 P(i)=>(x,y,z) position of ith joint，values are in meters <br/>
Joint number -> Joint name <br/>
     1 -> HEAD <br/>
     2 -> NECK <br/>
     3 -> TORSO <br/>
     4 -> LEFT_SHOULDER <br/>
     5 -> LEFT_ELBOW <br/>
     6 -> RIGHT_SHOULDER <br/>
     7 -> RIGHT_ELBOW <br/>
     8 -> LEFT_HIP <br/>
     9 -> LEFT_KNEE <br/>
    10 -> RIGHT_HIP <br/>
    11 -> RIGHT_KNEE <br/>
    12 -> LEFT_HAND <br/>
    13 -> RIGHT_HAND <br/>
    14 -> LEFT_FOOT <br/>
    15 -> RIGHT_FOOT <br/>

#### 20 joints Skeleton data structure   
Frame #, P(1),P(2),P(3),...,P(20)，<br/>
 P(i) =>(x,y,z) position of ith joint，values are in meters <br/>
Joint number -> Joint name <br/>
     1 -> HEAD <br/>
     2 -> SHOULDER CENTER (NECK) <br/>
     3 -> SIPNE(TORSO) <br/>
     4 -> LEFT_SHOULDER <br/>
     5 -> LEFT_ELBOW <br/>
     6 -> LEFT_HAND <br/>
     7 -> RIGHT_ SHOULDER <br/>
     8 -> RIGHTLEFT_ELBOW <br/>
     9 -> RIGHT_HAND <br/>
    10 -> LEFT_HIP <br/>
    11 -> LEFT_KNEE <br/>
    12 -> LEFT_FOOT <br/>
    13 -> RIGHT_HIP <br/>
    14 -> RIGHT_KNEE <br/>
    15 -> RIGHT_FOOT <br/>
    16 -> HIP CENTER <br/>
    17 -> LEFT_WRIST <br/>
    18 -> LEFT_ANKLE <br/>
    19 -> RIGHT_WRIST <br/>
    20 -> RIGHT_ANKLE <br/>
# Environment
* windows 10
* anaconda 
* python 3.7
# visualization Case 
## Integrated skeleton data come from different dataset source 
### CMU dataset subject 01-01, label: forward jump turn around.
left gif subject 01-01<br/>
right gif Jump1：1-248,Turn1:240-295,Jump2:295-480,Turn2:480-530,Jump3:530-668 <br/>

<img src="https://github.com/YUANYUAN2222/GIT_json_to_BVH/blob/main/%E5%9B%BE%E7%89%874.gif" width="400" height="220" >    <img src="https://github.com/YUANYUAN2222/GIT_json_to_BVH/blob/main/%E5%9B%BE%E7%89%873.gif" width="400" height="220">
## UTKA dataset 
<img src="https://github.com/YUANYUAN2222/GIT_json_to_BVH/blob/main/%E5%9B%BE%E7%89%8713.png" width="400" height="220" >    <img src="https://github.com/YUANYUAN2222/GIT_json_to_BVH/blob/main/%E5%9B%BE%E7%89%875.gif" width="400" height="220">



## HDM05 dataset 
<img src="https://github.com/YUANYUAN2222/GIT_json_to_BVH/blob/main/%E5%9B%BE%E7%89%879.png" width="400" height="220" >    <img src="https://github.com/YUANYUAN2222/GIT_json_to_BVH/blob/main/%E5%9B%BE%E7%89%8710.gif" width="400" height="220">






 


