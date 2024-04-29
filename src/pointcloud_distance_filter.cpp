#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <osqp/osqp.h>

// 回调函数处理接收到的LaserScan消息
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // 定义距离阈值
    float distance_threshold = 15;

    // 创建一个新的LaserScan消息
    sensor_msgs::LaserScan filtered_scan;
    filtered_scan.header = msg->header; // 使用原始的头信息
    filtered_scan.angle_min = msg->angle_min;
    filtered_scan.angle_max = msg->angle_max;
    filtered_scan.angle_increment = msg->angle_increment;
    filtered_scan.time_increment = msg->time_increment;
    filtered_scan.scan_time = msg->scan_time;
    filtered_scan.range_min = msg->range_min;
    filtered_scan.range_max = msg->range_max;

    // 计数器，用于统计有效测量的数量
    int valid_count = 0;

    // 遍历原始LaserScan中的每个测量值
    
    
    for (unsigned int i = 0; i < msg->ranges.size(); ++i) {
        // 检查距离是否小于阈值
        if (msg->ranges[i] < distance_threshold ) {
            // 将有效的测量值添加到新的LaserScan中
            filtered_scan.ranges.push_back(msg->ranges[i]);
            filtered_scan.intensities.push_back(msg->intensities[i]);
            valid_count++;
        } else {
            // 对于无效的测量，可以设置为NaN或者其他约定值
            filtered_scan.ranges.push_back(std::numeric_limits<float>::quiet_NaN());
        }
    }
    
    // 更新有效测量的数量
    filtered_scan.ranges.resize(valid_count);
    filtered_scan.intensities.resize(valid_count);
    
    // 发布过滤后的LaserScan消息
    ros::Publisher pub = ros::NodeHandle().advertise<sensor_msgs::LaserScan>("/scan_near", 10);
    pub.publish(filtered_scan);
}

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "pointcloud_distance_filter");
    ros::NodeHandle nh;

    // 订阅原始LaserScan数据
    ros::Subscriber sub = nh.subscribe("/scan_filtered", 10, laserCallback);
    printf("pointcloud_distance_filter node started.\n");
    ros::Publisher pub = ros::NodeHandle().advertise<sensor_msgs::LaserScan>("/scan_near", 10);

    // 进入ROS循环
    ros::spin();

    return 0;
}