# point_cloud_process
Point cloud pre-process from Kinect v2

## config/prepocess_prarm.txt
该文件指定了与处理过程中的各项参数，程序将在处理每一帧点云之前先读取该文本参数，因此可以直接修改，实时观察预处理点云的变化。
```
pass_through_x=1    设置是否允许相机坐标系x轴方向点云直通滤波
pass_through_y=1    y轴直通滤波
pass_through_z=1    z轴直通滤波
table_remove=1         是否允许剔除支撑桌面
outlier_remove=1      是否允许剔除离群点   
surface_smooth=0      是否允许表面平滑
subsample=0                是否允许降采样
z_min= 0.3                     设置z轴方向直通滤波的远近距离
z_max=0.9                      
x_left= -0.4                    设置x轴方向直通滤波的左右距离
x_right= 0.3                  
y_up= -0.25                 设置y轴方向直通滤波的上下距离
y_down= 0.2             
remain_rate=0.4             桌面剔除率
remain_points=500       桌面剩余最少点数
k_points= 50                   用于剔除离群点
thresh=0.9                        用于剔除离群点
radius= 0.03                   用于剔除离群点
subsampling_leaf_size=0.004              设置降采样voxel尺寸
```