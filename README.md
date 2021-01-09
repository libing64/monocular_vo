# 1. monocular visual odometry


## 1.1 steps
* get image sequence
* feature detection
* feature matching or tracking
* motion estimation(3d-2d correspondence)
* Local Optimiation(Bundle adjustment)


![kitti00](https://github.com/libing64/mono_vo/blob/master/image/mono_vo_so3.png)

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

# 4. SO3-VO
只是计算姿态信息，不管位置信息，但是但考虑姿态如果是积分的化的漂移速度也挺快
是feature 有outlier吗？

如果单姿态都不够准的化，那么linear sliding window的方式进行优化还是不行，只有姿态很准这事才靠谱
那个mono-vo它怎么处理的？为何姿态很准？只是补充了速度就可以了？

ORB + tracker不是很好的选择，FAST + tracker才比较合理 