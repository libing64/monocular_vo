# 1. monocular visual odometry


## 1.1 steps
* get image sequence
* feature detection
* feature matching or tracking
* motion estimation(3d-2d correspondence)
* Local Optimiation(Bundle adjustment)


![kitti00](https://github.com/libing64/mono_vo/blob/mono_vo_so3/image/mono_vo_so3.png)

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

# 5. trianglatePoints
注意数据类型一定要一致

# triangulate之后，每次keyframe到来又需要addKeyframe和features
这样又变成了一个序列的管理, 总不能把feature投影到上一个frame吧? 理论位置和理论feature?
如果是投影过来的位置，是不是信息没有丢失？ feats -> feats_proj

只要keyframe不更新，那么feats就不更新，根据R t进行投影过来的位置，以及track到的位置

第一步只考虑keyframe，别的帧先不考虑 

始终要维护三帧信息，当然更多帧就更好了，是不是就变成sliding window了？ 然后考虑丢掉哪一帧？
是丢弃最新的还是最老的？

多帧之间的信息，orb-slam以及ptam怎么维护的啊？

# 假设有sliding window内部的各种信息， features 旋转已知
怎么很好的求出t内？
一种就是两帧-> 3d pose, 然后3d-2d更新pose, 但是中间还是要不断的insert keyframe，其实不如直接进行 最小二乘法，然后最后二乘法，还是需要维护历史尺度，就需要marginalization，维护prior info

# 要不再仔细研究下orb slam的代码？

# insert new keyframe and new features to map
1. feature detect
2. feature matching
3. two view map init
4. insert new frame; 3d-2d correspondence; 
   add new feature to the map
   detect new and tracing, + Rt => new map points





