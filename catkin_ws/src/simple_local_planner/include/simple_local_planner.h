#ifndef SIMPLE_LOCAL_PLANNER_H_
#define SIMPLE_LOCAL_PLANNER_H_

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h> 
#include <vector>
#include <string>

namespace simple_local_planner {

// 定义导航状态
enum class PlannerState {
    INITIAL_ROTATION,
    FOLLOWING_PATH,
    FINAL_ROTATION,
    GOAL_REACHED
};

class SimpleLocalPlannerROS : public nav_core::BaseLocalPlanner {
public:
    /**
     * @brief 构造函数
     */
    SimpleLocalPlannerROS();

    /**
     * @brief 析构函数
     */
    ~SimpleLocalPlannerROS();

    /**
     * @brief 初始化函数，由move_base调用
     * @param name 规划器实例的名称
     * @param tf tf2的buffer指针
     * @param costmap_ros 代价地图的指针
     */
    void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

    /**
     * @brief 设置全局路径
     * @param plan 要跟随的全局路径
     * @return 如果路径被接受则为true
     */
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

    /**
     * @brief 核心函数，根据当前状态计算并返回速度指令
     * @param cmd_vel 计算出的速度指令将被填充到此变量
     * @return 如果成功计算出速度则为true
     */
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    /**
     * @brief 判断是否已到达最终目标
     * @return 如果已到达目标则为true
     */
    bool isGoalReached();

private:
    /**
     * @brief 将角度归一化到[-PI, PI]范围
     */
    double normalizeAngle(double angle);

    /**
     * @brief 获取机器人在世界坐标系下的当前位姿
     */
    bool getCurrentPose(geometry_msgs::PoseStamped& robot_pose);
    
    /**
     * @brief 发布路径以供可视化
     */
    void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path); // <-- 添加函数声明

    // 指针和ROS句柄
    tf2_ros::Buffer* tf_buffer_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher plan_pub_; // <-- 添加Publisher成员变量
    
    // 状态和数据
    bool initialized_; // <-- 添加初始化标志位
    bool goal_reached_;
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    PlannerState current_state_;
    std::string base_frame_;

    // 参数
    double yaw_tolerance_;
    double xy_goal_tolerance_;
    double carrot_distance_;
    double linear_speed_factor_;
    double angular_speed_factor_;
    double max_linear_speed_;
    double max_angular_speed_;
};

} // namespace simple_local_planner

#endif // SIMPLE_LOCAL_PLANNER_H_

