#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <cmath>

// 全局变量
double sampling_distance; // 采样间隔
geometry_msgs::PoseStamped current_pose; // 当前机器人位置

// 回调函数处理全局路径数据
void globalPlanCallback(const nav_msgs::Path::ConstPtr& msg) {
    // 创建一个新的Path消息用于存放采样后的数据
    nav_msgs::Path sampled_path;
    sampled_path.header = msg->header;

    // 计算需要采样的点的数量
    double total_distance = 5.0; // 采样的总距离为5.0米
    int num_samples = std::max(1, static_cast<int>(std::round(total_distance / sampling_distance)));

    // 遍历原始路径中的所有点
    for (int i = 0; i < msg->poses.size() && i < num_samples; ++i) {
        // 计算当前点到起始点的直线距离
        double dx = msg->poses[i].pose.position.x - current_pose.pose.position.x;
        double dy = msg->poses[i].pose.position.y - current_pose.pose.position.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        // 如果距离小于采样间隔，则跳过该点
        if (distance < sampling_distance) continue;

        // 将点添加到采样后的路径中
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = msg->header;
        pose_stamped.pose = msg->poses[i].pose;
        sampled_path.poses.push_back(pose_stamped);
    }

    // 发布采样后的路径
    ros::Publisher pub = ros::NodeHandle().advertise<nav_msgs::Path>("globalplan_sampled", 10);
    pub.publish(sampled_path);
}

// 回调函数处理当前机器人位置数据
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose = *msg;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_sampler");
    ros::NodeHandle nh1;

    tf::TransformListener listener;

    // 订阅全局路径和当前机器人位置
    ros::Subscriber sub_plan = nh1.subscribe("/move_base/TebLocalPlannerROS/global_plan", 10, globalPlanCallback);
    ros::Subscriber sub_pose = nh1.subscribe("/robot_posestamped", 10, poseCallback);

    // 设置采样间隔为1米
    sampling_distance = 0.01;
    
    ros::Publisher pub = ros::NodeHandle().advertise<nav_msgs::Path>("globalplan_sampled", 10);
    // 启动ROS循环
    ros::spin();

    return 0;
}