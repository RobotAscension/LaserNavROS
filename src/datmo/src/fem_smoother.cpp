#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <cmath>
#include <osqp/osqp.h>
#include "fem_smoother.h"
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>

// #include "cyber/common/log.h"
/**/
typedef std::pair<double, double> Point2D;
bool is_obstacle = false;
double Bias = 0.0;

struct Config {
    double weight_fem_pos_deviation = 1.0e10;
    double weight_ref_deviation = 1.0;
    double weight_path_length = 1.0;
    bool apply_curvature_constraint = false;
    double weight_curvature_constraint_slack_var = 1.0e2;
    double curvature_constraint = 0.2;

    int max_iter = 500;
  // time_limit set to be 0.0 meaning no time limit
    double time_limit = 0.0;
    bool verbose = false;
    bool scaled_termination = true;
    bool warm_start = true;

};

std::vector<Point2D> corner_points;

void cornerPointsCallback(const geometry_msgs::PoseArray::ConstPtr& msg){
    for (int i = 0; i < msg->poses.size();  i++) {
        // msg has no member named points
        
        double x = msg->poses[i].position.x;
        double y = msg->poses[i].position.y;
        corner_points.push_back(std::make_pair(x,y));
}
}

double calculateDistance(const std::pair<double, double>& point1,
                         const std::pair<double, double>& point2) {
    return std::sqrt((point2.first - point1.first) * (point2.first - point1.first) +
                     (point2.second - point1.second) * (point2.second - point1.second));
}

// 找到最近的点并计算距离的函数
double findNearestDistance(const std::pair<double, double> & points,
                           const std::vector<std::pair<double, double>>& target_points) 
    {
    double min_distance = 10;
    int index;
    for (int i = 0; i < target_points.size(); i++) {
        
        // calculate distance of y-direction
        double distance = calculateDistance(points, target_points[i]);

        if (distance < min_distance) {
            min_distance = distance;
            index = i;
        }

    }

    return index;
}

void LaserNearCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    int nNum = msg->ranges.size();
    int nMid = nNum/2;
    float fMidDist = msg->ranges[nMid];
    if (fMidDist < 10.0) {
        is_obstacle = true;
        Bias = 1.5;
    }

}
// 回调函数处理全局路径数据
void globalPlanCallback(const nav_msgs::Path::ConstPtr& msg) {
    std::vector<Point2D> raw_point2d;
    // 只取前50个点
    int num_d = 15;
    for ( int i = 0; i < msg->poses.size() && i < 20 * num_d; i+=num_d) {
        // 提取x和y坐标
        double x = msg->poses[i].pose.position.x;
        double y = msg->poses[i].pose.position.y;
        if(is_obstacle){
          double delta_x = msg->poses[i+num_d].pose.position.x-x;
          double delta_y = msg->poses[i+num_d].pose.position.y-y;
        
          x = msg->poses[i].pose.position.x + 0.2*delta_y*i/num_d;
          y = msg->poses[i].pose.position.y - 0.2*delta_x*i/num_d;
        }
        // 将坐标点添加到raw_point2d中
        raw_point2d.push_back(std::make_pair(x, y));
    }

    // set bounds around raw_point2d, length = 20
    
    std::vector<double> bounds;
    //bounds.reserve(raw_point2d.size());
    for (int i = 0; i < raw_point2d.size(); i++) {

        double index = findNearestDistance(raw_point2d[i], corner_points);
        
        double x = std::abs(corner_points[index].first - raw_point2d[i].first); 
        double y = std::abs(corner_points[index].second - raw_point2d[i].second); 

        double distance = std::min(x, y);
        
        std::cout << "distance: " << distance << std::endl;

        distance = std::min(distance, 5.0);
        
        
         bounds.push_back(distance);
        

         //bounds.push_back(0.3);
    }
    
   /*
    std::vector<double>* opt_x;
    std::vector<double>* opt_y;
  if (opt_x == nullptr || opt_y == nullptr) {
    std::cerr << "  opt_x or opt_y is nullptr" << std::endl;

  }
*/ 
    FemPosDeviationOsqpInterface solver;
    Config config;
    solver.set_weight_fem_pos_deviation(config.weight_fem_pos_deviation);
    solver.set_weight_path_length(config.weight_path_length);
    solver.set_weight_ref_deviation(config.weight_ref_deviation);
 
    solver.set_max_iter(config.max_iter);
    solver.set_time_limit(config.time_limit);
    solver.set_verbose(config.verbose);
    solver.set_scaled_termination(config.scaled_termination);
    solver.set_warm_start(config.warm_start);
    
    solver.set_ref_points(raw_point2d);
    solver.set_bounds_around_refs(bounds);
    if (!solver.Solve()) {
        std::cerr << "  Failed to find solution." << std::endl;
    }
    const std::vector<double>& result_x = solver.opt_x();
    const std::vector<double>& result_y = solver.opt_y();

   //for (int i = 0; i < result_x.size(); ++i) {
        //std::cout << "  point " << i << ": (" << result_x[i] << ", " << result_y[i] << ")" << std::endl;
  //  }
        nav_msgs::Path smooth_path;
    smooth_path.header.frame_id = "map";
    smooth_path.header.stamp = ros::Time::now();
    for (int i = 0; i < result_x.size(); i++) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = smooth_path.header.stamp;
        pose_stamped.header.frame_id = smooth_path.header.frame_id;
        pose_stamped.pose.position.x = result_x[i];
        pose_stamped.pose.position.y = result_y[i];
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation.x = 0.0;
        pose_stamped.pose.orientation.y = 0.0;
        pose_stamped.pose.orientation.z = 0.0;
        pose_stamped.pose.orientation.w = 1.0;
        smooth_path.poses.push_back(pose_stamped);
    }
    
    ros::Publisher smooth_path_pub =
        ros::NodeHandle().advertise<nav_msgs::Path>("smooth_path", 10);
    smooth_path_pub.publish(smooth_path);
}
bool FemPosDeviationOsqpInterface::Solve() {
  // Sanity Check
  if (ref_points_.empty()) {
    //AERROR << "reference points empty, solver early terminates";
    return false;
  }

  if (ref_points_.size() != bounds_around_refs_.size()) {
    //AERROR << "ref_points and bounds size not equal, solver early terminates";
    return false;
  }

  if (ref_points_.size() < 3) {
    //AERROR << "ref_points size smaller than 3, solver early terminates";
    return false;
  }

  if (ref_points_.size() > std::numeric_limits<int>::max()) {
    //AERROR << "ref_points size too large, solver early terminates";
    return false;
  }

  // Calculate optimization states definitions
  num_of_points_ = static_cast<int>(ref_points_.size());
  // printf("num_of_points_ = %d\n", num_of_points_);
  num_of_variables_ = num_of_points_ * 2;
  num_of_constraints_ = num_of_variables_;

  // Calculate kernel
  std::vector<c_float> P_data;
  std::vector<c_int> P_indices;
  std::vector<c_int> P_indptr;
  CalculateKernel(&P_data, &P_indices, &P_indptr);

  // Calculate affine constraints
  std::vector<c_float> A_data;
  std::vector<c_int> A_indices;
  std::vector<c_int> A_indptr;
  std::vector<c_float> lower_bounds;
  std::vector<c_float> upper_bounds;
  CalculateAffineConstraint(&A_data, &A_indices, &A_indptr, &lower_bounds,
                            &upper_bounds);

  // Calculate offset
  std::vector<c_float> q;
  CalculateOffset(&q);

  // Set primal warm start
  std::vector<c_float> primal_warm_start;
  SetPrimalWarmStart(&primal_warm_start);

  OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
  OSQPSettings* settings =
      reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));

  // Define Solver settings
  osqp_set_default_settings(settings);
  settings->max_iter = max_iter_;
  settings->time_limit = time_limit_;
  settings->verbose = verbose_;
  settings->scaled_termination = scaled_termination_;
  settings->warm_start = warm_start_;

  OSQPWorkspace* work = nullptr;

  bool res = OptimizeWithOsqp(num_of_variables_, lower_bounds.size(), &P_data,
                              &P_indices, &P_indptr, &A_data, &A_indices,
                              &A_indptr, &lower_bounds, &upper_bounds, &q,
                              &primal_warm_start, data, &work, settings);
  if (res == false || work == nullptr || work->solution == nullptr) {
  //  AERROR << "Failed to find solution.";
    // Cleanup
    osqp_cleanup(work);
    c_free(data->A);
    c_free(data->P);
    c_free(data);
    c_free(settings);

    return false;
  }

  // Extract primal results
  x_.resize(num_of_points_);
  y_.resize(num_of_points_);
  for (int i = 0; i < num_of_points_; ++i) {
    int index = i * 2;
    x_.at(i) = work->solution->x[index];
    y_.at(i) = work->solution->x[index + 1];
  }

  // Cleanup
  osqp_cleanup(work);
  c_free(data->A);
  c_free(data->P);
  c_free(data);
  c_free(settings);

  return true;
}

void FemPosDeviationOsqpInterface::CalculateKernel(
    std::vector<c_float>* P_data, std::vector<c_int>* P_indices,
    std::vector<c_int>* P_indptr) {
  // CHECK_GT(num_of_variables_, 4);

  // Three quadratic penalties are involved:
  // 1. Penalty x on distance between middle point and point by finite element
  // estimate;
  // 2. Penalty y on path length;
  // 3. Penalty z on difference between points and reference points

  // General formulation of P matrix is as below(with 6 points as an example):
  // I is a two by two identity matrix, X, Y, Z represents x * I, y * I, z * I
  // 0 is a two by two zero matrix
  // |X+Y+Z, -2X-Y,   X,       0,       0,       0    |
  // |0,     5X+2Y+Z, -4X-Y,   X,       0,       0    |
  // |0,     0,       6X+2Y+Z, -4X-Y,   X,       0    |
  // |0,     0,       0,       6X+2Y+Z, -4X-Y,   X    |
  // |0,     0,       0,       0,       5X+2Y+Z, -2X-Y|
  // |0,     0,       0,       0,       0,       X+Y+Z|

  // Only upper triangle needs to be filled
  std::vector<std::vector<std::pair<c_int, c_float>>> columns;
  columns.resize(num_of_variables_);
  int col_num = 0;

  for (int col = 0; col < 2; ++col) {
    columns[col].emplace_back(col, weight_fem_pos_deviation_ +
                                       weight_path_length_ +
                                       weight_ref_deviation_);
    ++col_num;
  }

  for (int col = 2; col < 4; ++col) {
    columns[col].emplace_back(
        col - 2, -2.0 * weight_fem_pos_deviation_ - weight_path_length_);
    columns[col].emplace_back(col, 5.0 * weight_fem_pos_deviation_ +
                                       2.0 * weight_path_length_ +
                                       weight_ref_deviation_);
    ++col_num;
  }

  int second_point_from_last_index = num_of_points_ - 2;
  for (int point_index = 2; point_index < second_point_from_last_index;
       ++point_index) {
    int col_index = point_index * 2;
    for (int col = 0; col < 2; ++col) {
      col_index += col;
      columns[col_index].emplace_back(col_index - 4, weight_fem_pos_deviation_);
      columns[col_index].emplace_back(
          col_index - 2,
          -4.0 * weight_fem_pos_deviation_ - weight_path_length_);
      columns[col_index].emplace_back(
          col_index, 6.0 * weight_fem_pos_deviation_ +
                         2.0 * weight_path_length_ + weight_ref_deviation_);
      ++col_num;
    }
  }

  int second_point_col_from_last_col = num_of_variables_ - 4;
  int last_point_col_from_last_col = num_of_variables_ - 2;
  for (int col = second_point_col_from_last_col;
       col < last_point_col_from_last_col; ++col) {
    columns[col].emplace_back(col - 4, weight_fem_pos_deviation_);
    columns[col].emplace_back(
        col - 2, -4.0 * weight_fem_pos_deviation_ - weight_path_length_);
    columns[col].emplace_back(col, 5.0 * weight_fem_pos_deviation_ +
                                       2.0 * weight_path_length_ +
                                       weight_ref_deviation_);
    ++col_num;
  }

  for (int col = last_point_col_from_last_col; col < num_of_variables_; ++col) {
    columns[col].emplace_back(col - 4, weight_fem_pos_deviation_);
    columns[col].emplace_back(
        col - 2, -2.0 * weight_fem_pos_deviation_ - weight_path_length_);
    columns[col].emplace_back(col, weight_fem_pos_deviation_ +
                                       weight_path_length_ +
                                       weight_ref_deviation_);
    ++col_num;
  }

  // CHECK_EQ(col_num, num_of_variables_);

  int ind_p = 0;
  for (int i = 0; i < col_num; ++i) {
    P_indptr->push_back(ind_p);
    for (const auto& row_data_pair : columns[i]) {
      // Rescale by 2.0 as the quadratic term in osqp default qp problem setup
      // is set as (1/2) * x' * P * x
      P_data->push_back(row_data_pair.second * 2.0);
      P_indices->push_back(row_data_pair.first);
      ++ind_p;
    }
  }
  P_indptr->push_back(ind_p);
}

void FemPosDeviationOsqpInterface::CalculateOffset(std::vector<c_float>* q) {
  for (int i = 0; i < num_of_points_; ++i) {
    const auto& ref_point_xy = ref_points_[i];
    q->push_back(-2.0 * weight_ref_deviation_ * ref_point_xy.first);
    q->push_back(-2.0 * weight_ref_deviation_ * ref_point_xy.second);
  }
}

void FemPosDeviationOsqpInterface::CalculateAffineConstraint(
    std::vector<c_float>* A_data, std::vector<c_int>* A_indices,
    std::vector<c_int>* A_indptr, std::vector<c_float>* lower_bounds,
    std::vector<c_float>* upper_bounds) {
  int ind_A = 0;
  for (int i = 0; i < num_of_variables_; ++i) {
    A_data->push_back(1.0);
    A_indices->push_back(i);
    A_indptr->push_back(ind_A);
    ++ind_A;
  }
  A_indptr->push_back(ind_A);

  for (int i = 0; i < num_of_points_; ++i) {
    const auto& ref_point_xy = ref_points_[i];
    upper_bounds->push_back(ref_point_xy.first + bounds_around_refs_[i]);
    upper_bounds->push_back(ref_point_xy.second + bounds_around_refs_[i]);
    lower_bounds->push_back(ref_point_xy.first - bounds_around_refs_[i]);
    lower_bounds->push_back(ref_point_xy.second - bounds_around_refs_[i]);
  }
}

void FemPosDeviationOsqpInterface::SetPrimalWarmStart(
    std::vector<c_float>* primal_warm_start) {
  // CHECK_EQ(ref_points_.size(), static_cast<size_t>(num_of_points_));
  for (const auto& ref_point_xy : ref_points_) {
    primal_warm_start->push_back(ref_point_xy.first);
    primal_warm_start->push_back(ref_point_xy.second);
  }
}

bool FemPosDeviationOsqpInterface::OptimizeWithOsqp(
    const size_t kernel_dim, const size_t num_affine_constraint,
    std::vector<c_float>* P_data, std::vector<c_int>* P_indices,
    std::vector<c_int>* P_indptr, std::vector<c_float>* A_data,
    std::vector<c_int>* A_indices, std::vector<c_int>* A_indptr,
    std::vector<c_float>* lower_bounds, std::vector<c_float>* upper_bounds,
    std::vector<c_float>* q, std::vector<c_float>* primal_warm_start,
    OSQPData* data, OSQPWorkspace** work, OSQPSettings* settings) {
  // CHECK_EQ(lower_bounds->size(), upper_bounds->size());

  data->n = kernel_dim;
  data->m = num_affine_constraint;
  data->P = csc_matrix(data->n, data->n, P_data->size(), P_data->data(),
                       P_indices->data(), P_indptr->data());
  data->q = q->data();
  data->A = csc_matrix(data->m, data->n, A_data->size(), A_data->data(),
                       A_indices->data(), A_indptr->data());
  data->l = lower_bounds->data();
  data->u = upper_bounds->data();

  //*work = osqp_setup( data, settings);
   osqp_setup(work, data, settings);

  osqp_warm_start_x(*work, primal_warm_start->data());

  // Solve Problem
  osqp_solve(*work);

  auto status = (*work)->info->status_val;

  if (status < 0) {
    // AERROR << "failed optimization status:\t" << (*work)->info->status;
    return false;
  }

  if (status != 1 && status != 2) {
    // AERROR << "failed optimization status:\t" << (*work)->info->status;
    return false;
  }

  return true;
}

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "fem_smoother");
    ros::NodeHandle nh;
    ros::Subscriber sub_scan = nh.subscribe("/scan_near", 10, LaserNearCallback);

    ros::Subscriber sub_corner_points = nh.subscribe("/datmo/corner_points", 10, cornerPointsCallback);
    // 订阅全局路径话题
    ros::Subscriber sub_plan = nh.subscribe("/move_base/TebLocalPlannerROS/global_plan", 10, globalPlanCallback);
    // subscribe geometry_msgs::Point topic for the data of corner points
    

    ros::Publisher smooth_path_pub =
        ros::NodeHandle().advertise<nav_msgs::Path>("smooth_path", 10);
    // 进入ROS循环
    ros::spin();

    return 0;
}