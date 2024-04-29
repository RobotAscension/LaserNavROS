#include <osqp/osqp.h>
#include <Eigen/Dense>
#include <iostream>
#include <random>
#include <OsqpEigen/OsqpEigen.h>

int main() {
    // 问题数据
    int n = 10; // 变量数量
    int m = 5;  // 约束数量

    // 随机生成P矩阵和q向量
    Eigen::MatrixXd P = Eigen::MatrixXd::Random(n, n);
    Eigen::VectorXd q = Eigen::VectorXd::Random(n);

    // 随机生成A矩阵和l向量、u向量
    Eigen::MatrixXd A = Eigen::MatrixXd::Random(m, n);
    Eigen::VectorXd l = Eigen::VectorXd::Random(m);
    Eigen::VectorXd u = Eigen::VectorXd::Random(m);

    // 初始化OSQP设置
    OSQPSettings *settings = (OSQPSettings *)malloc(sizeof(OSQPSettings));
    osqp_set_default_settings(settings);
    settings->alpha = 1.6; // 设置alpha参数

    // 创建OSQP工作空间和数据对象
    OSQPWorkspace *work;
    OSQPData *data = (OSQPData *)c_malloc(sizeof(OSQPData));

    data->n = n;
    data->m = m;
    data->P = P.data();
    data->q = q.data();
    data->A = A.data();
    data->l = l.data();
    data->u = u.data();

    // 设置工作空间
    osqp_setup(&work, data, settings);

    // 清理设置和数据对象
    c_free(data);
    osqp_cleanup(work);
    c_free(settings);

    std::cout << "OSQP test completed successfully." << std::endl;

    return 0;
}