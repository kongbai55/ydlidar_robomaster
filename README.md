# ydlidar_robomaster

This project implements mobile robot localization and navigation using the ROS framework. The system consists of the following components:

- LiDAR-Based Mapping and Navigation: A YDLIDAR X3 LiDAR is used to collect environmental data and construct a 2D occupancy grid map, providing environmental information for path planning and navigation.

- Localization Module: The AMCL (Adaptive Monte Carlo Localization) algorithm is employed to estimate the robot's pose in real time within a prebuilt map, enabling accurate and reliable localization.

- Motion Control: The RoboMaster chassis handles low-level motion control and trajectory tracking. It supports differential-drive control and path execution to enable autonomous navigation.

The system supports autonomous mapping, accurate localization, and path planning and navigation, making it suitable for research and practical applications involving indoor mobile robots.

本项目基于 ROS 框架实现移动机器人定位与导航功能。系统主要由以下部分构成：

激光雷达建图与导航：采用 YDLIDAR X3 激光雷达进行环境信息采集，构建二维栅格地图，并提供路径规划所需的环境感知。

定位模块：使用 AMCL (Adaptive Monte Carlo Localization) 算法实现自适应蒙特卡洛定位，在已有地图中进行实时位姿估计，确保机器人具备高精度定位能力。

运动控制：通过 RoboMaster 底盘实现底层运动驱动与轨迹跟踪，支持差速控制与路径执行，完成自主导航任务。

该系统能够实现 自主建图、精确定位与路径规划导航，适用于室内移动机器人相关研究与应用场景。
