/*
 * Copyright (c) 2020, Robobrain.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Konstantinos Konstantinidis */

#include "ros/ros.h"
#include "datmo.hpp"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

//static int nCount = 0;
nav_msgs::Odometry latestOdom;
sensor_msgs::LaserScan last_laser_scan;
static int n_callback = 0;
geometry_msgs::Twist vel_cmd;
static double vx_odom;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) 
{
        if (n_callback == 0) {
        latestOdom = *msg;
    }

    // 计算位置差
    double dx = msg->pose.pose.position.x - latestOdom.pose.pose.position.x;

    // 计算时间差
    ros::Duration time_diff = msg->header.stamp - latestOdom.header.stamp;

    // 计算线速度
    vx_odom = dx / time_diff.toSec();
    
}

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
        
    ros::Publisher pub_scan_near = ros::NodeHandle().advertise<sensor_msgs::LaserScan>("/scan_near", 10);
    pub_scan_near.publish(filtered_scan);    


}

void laserCallback_vel(const sensor_msgs::LaserScan::ConstPtr& msg) {
    
    int nNum = msg->ranges.size();
    int nMid = nNum/2;
    float fMidDist = msg->ranges[nMid];

    if (n_callback == 0) {
        last_laser_scan = *msg;
    }
    bool obstacle_in_motion = false;
    double dis_diff = std::abs(msg->ranges[nMid] - last_laser_scan.ranges[nMid]);
     if (std::abs(dis_diff - vx_odom) > 0.1) {
            // 如果相邻两次扫描中的距离变化超过0.1米，我们认为障碍物可能在移动
            obstacle_in_motion = true;
            
        }
    n_callback++ ;
    last_laser_scan = *msg;
    double Kp = 0.5;  // 比例系数
    double Ki = 0.1;  // 积分系数

    // PI控制器的变量
    double integral_term = 0.0;
    double setpoint = 5.0;

    // 减速到停止的最大减速度
    double max_deacceleration = 0.0;


        // 如果距离小于阈值，则使用PI控制器计算速度
    if (fMidDist < 10.5 && obstacle_in_motion) {
        // 计算误差
        double error = setpoint - fMidDist;

        // 更新积分项
        integral_term += error * ros::Duration(1.0 / 10).toSec();  // 假设10Hz的回调频率
        // 计算PI控制器的输出
        double output = Kp * error + Ki * integral_term;
        // 限制输出，以确保不超过最大减速度
        double velocity = std::max(std::min(output, max_deacceleration), -max_deacceleration);

        vel_cmd.linear.x = velocity;
    }
    else
    {
        vel_cmd.linear.x = 5.0;
    }
    // 发布速度消息
    ros::Publisher vel_pub = ros::NodeHandle().advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    vel_pub.publish(vel_cmd);
}
int main(int argc, char **argv)
{
  //Initiate ROS                         
  ros::init(argc, argv, "datmo_node");
  ros::NodeHandle nh;

  ros::Subscriber odomsub = nh.subscribe("/odom", 10, odomCallback);
  ros::Subscriber sub_pose = nh.subscribe("/robot_pose", 10, poseCallback);

  ros::Subscriber sub_scan = nh.subscribe("/scan_filtered", 10, laserCallback);
  ros::Subscriber sub_scan1 = nh.subscribe("/scan_filtered", 10, laserCallback_vel);
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("/robot_posestamped", 10);

ros::Publisher pub_scan_near = nh.advertise<sensor_msgs::LaserScan>("/scan_near", 10);
   ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  //Create an object of class datmo 
  Datmo  datmo_object;

  ros::spin();

  return 0;
}
