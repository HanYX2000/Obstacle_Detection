/perception/lidar_perception 中为主要文件

--> /perception/lidar_perception/src/SFA3D  检测网络(torch=1.5 python=3.6)
    订阅/rslidar_points 
    发布/obstacle_poses  检测到的车/行人/自行车位姿
--> /perception/lidar_perception/src/ogm.cpp  绘制OGM
    订阅/obstacle_poses
    发布/all_lidars      三个雷达点云融合
       /target_man      反光衣对应的点云（供验证）
       /OGM_Final       最终处理后的栅格地图   

接口: /OGM_Final 栅格地图:data=0:   无障碍物
                         data=100: 障碍物，无类型
                         data=-128:人
                         data=-32: 车辆
                         data=-64: 反光条
                         data=127: 自身车辆

