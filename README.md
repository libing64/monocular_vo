# 1. monocular visual odometry


## 1.1 steps
* get image sequence
* feature detection
* feature matching or tracking
* motion estimation(3d-2d correspondence)
* Local Optimiation(Bundle adjustment)


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
- [ ] local sparse bundle adjustment
- [ ] record screen to gif
- [ ] mapping
- [ ] fusing imu 


# 3. OpenCV 升级