## SimpleSLAM

A lidar-centric simple slam integrated with sensors like imu and wheel odometer, with lots of interfaces for easily re-developing


### Modules

* dataproxy

data interface compatible with various comm protocols. Adapt to another data is easy, you should inherit Base class *DataProxy*, which is templated and ensure thread-safty with block queue(see `common/utils/SafeDeque.hpp`)

* frontend

SLAM frontend providing real-time odometry. Our frontend module is not depend on backend. Frontend is testable individually as locating mode(see unit test: `test/loc.cpp`). Our frontend is lidar-centric, Lidar odometry is the most important part applying the key algorthm of loose-coupled schem.

* PCR(PointCloudRegister)

Frontend heavily depends on PCR. PCR abstract common interface of lidar register and adapt with some mature lidar register algorithms. LOAM-like pc register implemented by [1] is reimplemented with Lie-group update schem. More good algorithms like Ndt[2], Vgicp[3] are available.

* filter

Filter module serves to provide locally accurate odometry like fusion of dead-reckoning, imu or wheel encoder. Extendtable light-weight header-only Kalman library[4] is included. Current version is just simple fusion of wheel, imu and static model. 2D case performance is totally the same as [5].(see test for comparison: `filter/test`)

* backend

SLAM backend for optimization stuff. Backend depends on frontend, which sends keyframe to it. Then pose graph is constructed and optimize all keyframe pose based on gtsam library[6]. Loop closure detection is also included. I want to treat Loop closure detect as Place Recognition problem for scalerable application. So one Keyframe should be able to convert to generalizable context(or say descriptor). Place Recognition problem is just querying context from context database. If you want to use your own context, inherit *ContextBase*(see `backend/include`). For now, context based on 2D-distance and ScanContext[7] is implemented.

* unit test

There are lots of unit test to check each modules working properly. Some are listed below:

1. `benchmark/kdtree.cpp` some benchmark showing efficiency of kdtree implement by PCL or Nanoflann[8].
2. `benchmark/rospub.cpp` showing that the time consumeing of publishing ros message which is not subscribed or is subscibed. They are different.
3. `comp/liorf_scan2map.cpp` compare my Lie-group update LOAM register with original Euler-angle update LOAM[9] register  
4. `dataproxy.cpp` test worker-consumer case with blocked or non-blocked deque.
5. `lcm.cpp` test Loop Closure Manager function with visualization.
6. `logger.cpp` test logger and progress bar.
7. `thread.cpp` test resident thread which means the thread's life extend utils whole program exit.
8. `loc.cpp` minimal lidar odometry case, if you has static point cloud maps, you can test localization performance with mouse relocalization on rviz.

* app

The main entry of the project! It depends on configuration file at `config/params.json`.


### Ref.

[1] https://github.com/TixiaoShan/LIO-SAM

[2] https://github.com/koide3/ndt_omp

[3] https://github.com/SMRT-AIST/fast_gicp

[4] https://github.com/mherb/kalman

[5] http://wiki.ros.org/robot_pose_ekf

[6] https://github.com/borglab/gtsam

[7] https://github.com/irapkaist/scancontext

[8] https://github.com/jlblancoc/nanoflann

[9] https://github.com/YJZLuckyBoy/liorf