# GIT_json_to_BVH
introduction

The transform code was based on the original work “Video to 3DPose and BVH motion file”, which only could convert 17 body joints coordinates into the BVH motion file.
We rewrote part of code making the original code could transform the number of 15 or 20 body joints into BVH files.

== Construction Dataset ==
1) 15 joints Skeleton data files.txt                
Frame#, P(1),P(2),P(3),...,P(15)，
P(i)   =>(x,y,z) position of ith joint，values are in meters
Joint number -> Joint name
     1 -> HEAD
     2 -> NECK
     3 -> TORSO
     4 -> LEFT_SHOULDER
     5 -> LEFT_ELBOW
     6 -> RIGHT_SHOULDER
     7 -> RIGHT_ELBOW
     8 -> LEFT_HIP
     9 -> LEFT_KNEE
    10 -> RIGHT_HIP
    11 -> RIGHT_KNEE
    12 -> LEFT_HAND
    13 -> RIGHT_HAND
    14 -> LEFT_FOOT
    15 -> RIGHT_FOOT

2 ) 20 joints Skeleton data files.txt    
Frame#, P(1),P(2),P(3),...,P(20)，
P(i)   =>(x,y,z) position of ith joint，values are in meters
Joint number -> Joint name
     1 -> HEAD
     2 -> SHOULDER CENTER (NECK)  
     3 -> SIPNE(TORSO) 
     4 -> LEFT_SHOULDER
     5 -> LEFT_ELBOW
     6 -> LEFT_HAND
     7 -> RIGHT_ SHOULDER
     8 -> RIGHTLEFT_ELBOW
     9 -> RIGHT_HAND
    10 -> LEFT_HIP
    11 -> LEFT_KNEE
    12 -> LEFT_FOOT
    13 -> RIGHT_HIP
    14 -> RIGHT_KNEE
    15 -> RIGHT_FOOT
    16 -> HIP CENTER
    17 -> LEFT_WRIST
    18 -> LEFT_ANKLE
    19 -> RIGHT_WRIST
    20 -> RIGHT_ANKLE
![image](https://github.com/YUANYUAN2222/GIT_json_to_BVH/blob/main/%E5%9B%BE%E7%89%872.gif)
