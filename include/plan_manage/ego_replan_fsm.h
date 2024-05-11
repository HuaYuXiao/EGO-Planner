#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <bspline_opt/bspline_optimizer_ego.h>
#include <plan_env/grid_map.h>
#include <ego_planner/Bspline.h>
#include <ego_planner/DataDisp.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization_ego.h>

using std::vector;

namespace ego_planner{
  class EGOReplanFSM{

  private:
    /* ---------- flag 状态机FSM状态 ---------- */
    enum FSM_EXEC_STATE{
        INIT,			//初始状态
        WAIT_TARGET,  //等待触发
        GEN_NEW_TRAJ, //生成轨迹
        REPLAN_TRAJ,  //重规划轨迹
        EXEC_TRAJ,    //执行轨迹
        EMERGENCY_STOP //紧急停止
    };
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1, //手动选择目标点
        PRESET_TARGET = 2, //预设目标点
      REFENCE_PATH = 3
    };

    /* planning utils */
    EGOPlannerManager::Ptr planner_manager_; //ego过程管理
      PlanningVisualization::Ptr visualization_;	//可视化
    ego_planner::DataDisp data_disp_;

    /* parameters */
    double no_replan_thresh_, replan_thresh_;
    double waypoints_[50][3]; //目标点三轴位置
    double planning_horizen_; //局部规划的范围
    double emergency_time_; //如果距离碰撞的时间小于该值，立刻切换到停止模式

    /* planning data */
    bool trigger_, have_target_, have_odom_, have_new_target_;
    FSM_EXEC_STATE exec_state_;
    int continously_called_times_{0};

    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
    Eigen::Quaterniond odom_orient_;

    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // start state
    Eigen::Vector3d end_pt_, end_vel_;                                       // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_;                     // local target state
    int current_wp_;

    bool flag_escape_emergency_; //如果无人机停止或者得到新的无碰撞路径就设为true代表安全了

    /* ROS utils */
    ros::NodeHandle node_;
    ros::Timer exec_timer_, safety_timer_;
    ros::Subscriber waypoint_sub_, odom_sub_;
    ros::Publisher replan_pub_, new_pub_, bspline_pub_, data_disp_pub_;

    /* helper functions */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj); // front-end and back-end method
    bool callEmergencyStop(Eigen::Vector3d stop_pos);                          // front-end and back-end method
    bool planFromCurrentTraj();

    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    void printFSMExecState();

    void getLocalTarget();

    /* ROS functions */
    void execFSMCallback(const ros::TimerEvent &e);
    void checkCollisionCallback(const ros::TimerEvent &e);
    void waypointCallback(const nav_msgs::PathConstPtr &msg);
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);

    bool checkCollision();

  public:
    EGOReplanFSM(/* args */){}
    ~EGOReplanFSM(){}

    void init(ros::NodeHandle &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
} // namespace ego_planner

#endif
