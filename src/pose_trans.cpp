#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

nav_msgs::Odometry latestOdom;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) 
{
    latestOdom = *msg;
}


// 定义一个回调函数来处理接收到的Pose消息 and odem info
void poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{  // 创建一个PoseStamped消息
    geometry_msgs::PoseStamped stampedPose;
    
    // 将接收到的Pose消息的时间戳和帧ID复制到PoseStamped消息中
    stampedPose.header.seq = latestOdom.header.seq;


    stampedPose.header.stamp = latestOdom.header.stamp; // 或者使用msg->header.stamp，如果需要保留原始时间戳
    //将接收到的Pose消息的时间戳和帧ID复制到PoseStamped消息中


    stampedPose.header.frame_id = "map";

    // 将Pose消息的数据复制到PoseStamped消息中
    stampedPose.pose = *msg;

    // 发布PoseStamped消息
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("/robot_posestamped", 10);
    pub.publish(stampedPose);
}

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "robot_pose_to_pose_stamped");
    
    // 创建一个订阅者，订阅原始的/robot_pose话题
    ros::NodeHandle nh;

    ros::Subscriber odomsub = nh.subscribe("/odom", 10, odomCallback);
    ros::Subscriber sub = nh.subscribe("/robot_pose", 10, poseCallback);
    
    // 创建一个发布者，用于发布新的/robot_posestamped话题
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("/robot_posestamped", 10);

    ros::spin(); // 保持节点运行，直到ROS关闭

    return 0;
}
