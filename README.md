# 本仓库是ros的一个package(C++)，通过[USB串口设备](https://detail.tmall.com/item.htm?id=41323941056&ali_refid=a3_430583_1006:1109983619:N:fR9qc9ZbC3viF8dvT6rOSA==:4fa281c00790e7961dd7428bb1e08ed4&ali_trackid=1_4fa281c00790e7961dd7428bb1e08ed4&spm=a230r.1.14.1)读取[该款IMU传感器](https://item.taobao.com/item.htm?spm=a230r.7195193.1997079397.8.7b4c2f01nN8OQq&id=16837036760&abbucket=12)数据,然后转换为ros的topic发布出去
topic名称为`turtle1/imu_angle`，数据类型为`geometry_msgs::Vector3`，依次分别是yaw, pitch, roll角度(有磁力计数据)，单位是度（传感器支持原始角速度和加速度，可以修改为标准IMU数据）

# 1.编译安装
将此仓库clone到`catkin_ws/src`后，回到`catkin_ws`工作路径下，执行编译命令即可


注意：可能需要安装ros的serial库，`sudo apt-get install ros-kinetic-serial`
# 2.运行
1.首先需要给串口权限进行读写操作,需要修改`simple_car/script/setup_device.bash`中的`wanghuohuo`为本机的用户名

2.在终端输入`./setup_device.bash`，运行，给USB串口权限

3.运行：`rosrun simple_car simple_car_node`

