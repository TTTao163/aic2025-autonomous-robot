#include <simple_local_planner.h>

#include <pluginlib/class_list_macros.h>

#include <tf2/utils.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>

#include <nav_msgs/Path.h> // <-- 添加头文件

// 注册插件
PLUGINLIB_EXPORT_CLASS(simple_local_planner::SimpleLocalPlannerROS, nav_core::BaseLocalPlanner)

namespace simple_local_planner {

SimpleLocalPlannerROS::SimpleLocalPlannerROS() : 
    tf_buffer_(NULL),
    costmap_ros_(NULL),
    initialized_(false), 
    goal_reached_(false),
    current_state_(PlannerState::GOAL_REACHED) {}

SimpleLocalPlannerROS::~SimpleLocalPlannerROS() {}

void SimpleLocalPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) {
    setlocale(LC_ALL,"");
    ros::NodeHandle private_nh("~/" + name);

    tf_buffer_ = tf;
    costmap_ros_ = costmap_ros;
    
    base_frame_ = costmap_ros_->getBaseFrameID();

    // 从参数服务器加载参数
    private_nh.param("yaw_tolerance", yaw_tolerance_, 0.05);
    private_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.1);
    private_nh.param("carrot_distance", carrot_distance_, 0.1);
    private_nh.param("linear_speed_factor", linear_speed_factor_, 0.5);
    private_nh.param("angular_speed_factor", angular_speed_factor_, 0.8);
    private_nh.param("max_linear_speed", max_linear_speed_, 0.3);
    private_nh.param("max_angular_speed", max_angular_speed_, 0.5);

    // <-- 初始化路径发布者 -->
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
    
    initialized_ = true; // <-- 在末尾设置初始化标志位

    ROS_INFO("Simple Local Planner initialized.");
}

bool SimpleLocalPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized_) {
        ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
        return false;
    }
    if (plan.empty()) {
        ROS_ERROR("Received an empty plan.");
        return false;
    }
    global_plan_ = plan;//用于存储机器人的全局路径。路径由一系列带有时间戳和坐标系信息的位姿 (PoseStamped) 组成。
    goal_reached_ = false;
    current_state_ = PlannerState::INITIAL_ROTATION; // 设置初始状态
    ROS_INFO("New global plan received, size: %zu. Starting initial rotation.", global_plan_.size());

    // <-- 在收到新路径后立即发布它 -->
    publishPlan(global_plan_);

    return true;
}

bool SimpleLocalPlannerROS::isGoalReached() {
    return goal_reached_;
}

double SimpleLocalPlannerROS::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

bool SimpleLocalPlannerROS::getCurrentPose(geometry_msgs::PoseStamped& robot_pose){
    geometry_msgs::PoseStamped stamped_ident;
    stamped_ident.header.frame_id = base_frame_;
    stamped_ident.header.stamp = ros::Time(0);
    tf2::toMsg(tf2::Transform::getIdentity(), stamped_ident.pose);
    
    try {
        robot_pose = tf_buffer_->transform(stamped_ident, costmap_ros_->getGlobalFrameID());
    } catch (tf2::TransformException& ex) {
        ROS_ERROR("Failed to get robot pose: %s", ex.what());
        return false;
    }
    return true;
}

void SimpleLocalPlannerROS::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
    if (!initialized_) {
        ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    // 创建一个 nav_msgs::Path 消息
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    // 确保消息头中有正确的坐标系和时间戳
    gui_path.header.frame_id = costmap_ros_->getGlobalFrameID();
    gui_path.header.stamp = ros::Time::now();

    // 填充路径点
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
}

bool SimpleLocalPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    if (!initialized_) {
        ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
        return false;
    }
    if (goal_reached_ || global_plan_.empty()) {
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
        return true;
    }

    geometry_msgs::PoseStamped current_pose;
    if(!getCurrentPose(current_pose)) {
        ROS_ERROR("Could not get robot pose");
        return false;
    }

    geometry_msgs::PoseStamped final_goal_pose = global_plan_.back();
    double current_yaw = tf2::getYaw(current_pose.pose.orientation);
    
    double dist_to_final_goal = std::hypot(
        final_goal_pose.pose.position.x - current_pose.pose.position.x,
        final_goal_pose.pose.position.y - current_pose.pose.position.y
    );
    
    // --- 状态机逻辑 ---
    switch (current_state_) {
        case PlannerState::INITIAL_ROTATION:
        {
            // ROS_INFO_ONCE("状态: 初始旋转");
            double angle_to_goal = atan2(
                final_goal_pose.pose.position.y - current_pose.pose.position.y,
                final_goal_pose.pose.position.x - current_pose.pose.position.x
            );
            double angle_error = normalizeAngle(angle_to_goal - current_yaw);

            if (std::abs(angle_error) > yaw_tolerance_) {
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = angular_speed_factor_ * angle_error;
                // 限制最大角速度
                cmd_vel.angular.z = std::max(-max_angular_speed_, std::min(max_angular_speed_, cmd_vel.angular.z));
            } else {
                // ROS_INFO("初始旋转完成，开始路径跟随。");
                current_state_ = PlannerState::FOLLOWING_PATH;
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;
            }
            break;
        }

        case PlannerState::FOLLOWING_PATH:
        {
            // ROS_INFO_ONCE("状态: 路径跟随");
            if (dist_to_final_goal < xy_goal_tolerance_) {
                current_state_ = PlannerState::FINAL_ROTATION;
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;
                break;
            }
            
            // 寻找前瞻目标点(Carrot Point)
            geometry_msgs::PoseStamped target_pose_map;//用于存储最终选定的局部目标点位姿。
            bool target_found = false;//用于指示是否找到了符合前瞻距离要求的点。

            for (int i = global_plan_.size() - 1; i >= 0; --i) {//逆向遍历：从路径的最后一个点开始，向前（索引减小）遍历整个全局路径。
                double dist_to_point = std::hypot(//开始计算当前机器人位置到路径点的 2D 平面距离。
                    global_plan_[i].pose.position.x - current_pose.pose.position.x,
                    global_plan_[i].pose.position.y - current_pose.pose.position.y
                );
                if (dist_to_point > carrot_distance_) {//检查距离是否大于设定的前瞻距离
                    target_pose_map = global_plan_[i];
                    target_found = true;//找到了第一个（离机器人最远）符合前瞻距离要求的点。
                    break;
                }
            }
            // 如果所有点都太近，就选择路径上离当前位置最远的点
            if (!target_found) {
                target_pose_map = final_goal_pose;
            }

            // 将目标点转换到机器人基座标系下
            geometry_msgs::PoseStamped target_pose_base;
            try {
                target_pose_map.header.stamp = ros::Time(0); //使用最新的坐标变换。
                target_pose_base = tf_buffer_->transform(target_pose_map, base_frame_);//将全局坐标系中的前瞻点点转换到机器人基坐标系 
            } catch (tf2::TransformException& ex) {
                ROS_ERROR("Failed to transform carrot pose: %s", ex.what());
                return false;
            }

            // 根据前瞻点的位置计算速度
            double target_dist = std::hypot(target_pose_base.pose.position.x, target_pose_base.pose.position.y);///计算前瞻点在机器人局部坐标系中的距离
            double lookahead_angle = atan2(target_pose_base.pose.position.y, target_pose_base.pose.position.x);//计算前瞻点点相对于机器人朝向的夹角。

            cmd_vel.linear.x = linear_speed_factor_ * target_dist;
            cmd_vel.angular.z = angular_speed_factor_ * lookahead_angle;

            // 限制最大速度
            cmd_vel.linear.x = std::min(max_linear_speed_, cmd_vel.linear.x);
            cmd_vel.angular.z = std::max(-max_angular_speed_, std::min(max_angular_speed_, cmd_vel.angular.z));

            break;
        }

        case PlannerState::FINAL_ROTATION:
        {
            // ROS_INFO_ONCE("状态: 最终旋转");
            double final_yaw = tf2::getYaw(final_goal_pose.pose.orientation);
            double angle_error = normalizeAngle(final_yaw - current_yaw);

            if (std::abs(angle_error) > yaw_tolerance_) {
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = angular_speed_factor_ * angle_error;
                // 限制最大角速度
                cmd_vel.angular.z = std::max(-max_angular_speed_, std::min(max_angular_speed_, cmd_vel.angular.z));
            } else {
                ROS_INFO("goal reached!");
                current_state_ = PlannerState::GOAL_REACHED;
                goal_reached_ = true;
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;
            }
            break;
        }
        
        case PlannerState::GOAL_REACHED:
        {
            goal_reached_ = true;
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
            break;
        }
    }
    
    return true;
}

} // namespace simple_local_planner