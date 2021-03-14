# Monocular Visual Odometry


## 1.1 Steps
* Map Init
  * feature detection
  * feature tracking
  * motion estimation wtih 2d-2d correspondence
  * triangulation
  

* Pose Update
  * feature detection with mask
  * feature tracking
  * motion estimation with 3d-2d correspondence
  * traingulation for new detected features 
  * local bundle adjustment
  * update triangulated points to map


![kitti00](https://github.com/libing64/mono_vo/blob/mono_vo_se3_devel/image/mono_vo_se3.png)


![kitti00](https://github.com/libing64/mono_vo/blob/mono_vo_se3_devel/image/mono_vo_se3.gif)

## 1.2 Building
```
cd catkin_ws/src
git clone git@github.com:libing64/mono_vo.git
cd ..
catkin_make -DCATKIN_WHITELIST_PACKAGES="mono_vo"
```
## 1.3 Running with kitti dataset
modify the dataset_folder in mono_vo_kitti.launch 
```
soure devel/setup.bash
roslaunch mono_vo mono_vo_kitti.launch
```

## 1.5 Test Environment
Ubuntu 20.04 + ros noetic


# 2. TODO
## 2.1 How to make the estimator more robust and accurate?
- [ ] local bundle adjustment
- [x] record screen to gif
- [ ] mapping
- [ ] fusing imu 
- [ ] sliding window 
- [ ] clean outliers


# 3. Stereo-VO vs Mono-VO
单目VO比双目VO难在哪？
双目VO通过单帧的一对图像可以直接获取深度信息， 前后两帧之间可以通过3d-2d或者3d-3d的方法估计运动，然后进行积分。

但是单目没法直接获取深度信息，前后帧之间虽然可以估计的R和t，但是平移信息只有方向没有尺度，需要sliding window保存多帧的信息，通过多帧之间的约束关系才能保证尺度一致。 