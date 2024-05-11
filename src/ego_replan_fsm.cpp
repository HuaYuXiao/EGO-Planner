#include <plan_manage/ego_replan_fsm.h>

namespace ego_planner
{
  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;

    /*  fsm param  */
    nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/emergency_time_", emergency_time_, 1.0);

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);

    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::checkCollisionCallback, this);

    odom_sub_ = nh.subscribe("/prometheus/drone_odom", 1, &EGOReplanFSM::odometryCallback, this);
      waypoint_sub_ = nh.subscribe("/waypoint_generator/waypoints", 1, &EGOReplanFSM::waypointCallback, this);

    bspline_pub_ = nh.advertise<ego_planner::Bspline>("/prometheus/planning/bspline", 10);
    data_disp_pub_ = nh.advertise<ego_planner::DataDisp>("/planning/data_display", 100);
  }

  // 初始位置设为当前位置，调用planNextWaypoint规划到msg的路径并执行
  void EGOReplanFSM::waypointCallback(const nav_msgs::PathConstPtr &msg)
  {
    if (msg->poses[0].pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;
    trigger_ = true;
    init_pt_ = odom_pos_;

    bool success = false;
    // TODO: height of end_pt_ should be set to goal height
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, odom_pos_(2);
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {
      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

       visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 1, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

    // 保存无人机当前里程计信息，包括位置、速度和姿态
  void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call){
    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::printFSMExecState(){
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e){
//      printFSMExecState();

    switch (exec_state_)
    {
        // 如果没程计信息就直接结束，否则切换到WAIT_TARGET
    case INIT:{
      if (!have_odom_){
          cout << "[planner] no odom." << endl;
        return;
      }
      if (!trigger_){
          cout << "[planner] wait for goal." << endl;
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

            // 如果没有目标位置和启动信号就直接结束，否则切换到GEN_NEW_TRAJ
    case WAIT_TARGET:{
      if (!have_target_)
        return;
      else{
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:{
            // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
            // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
            // start_yaw_(1) = start_yaw_(2) = 0.0;

            bool success = planFromGlobalTraj();
            if (success){
                // 如果成功，则切换到EXEC_TRAJ, 并flag_escape_emergency_=true
                changeFSMExecState(EXEC_TRAJ, "FSM");
                flag_escape_emergency_ = true;
            }
            else{
                changeFSMExecState(GEN_NEW_TRAJ, "FSM");
            }
            break;
        }

    case REPLAN_TRAJ:{
      if (planFromCurrentTraj())
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }else{
          // 否则切换到REPLAN_TRAJ
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:{
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - info->start_time_).toSec();
      t_cur = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

        // 局部规划的终点与全局终点接近
      /* && (end_pt_ - pos).norm() < 0.5 */
      // TODO: 当前局部轨迹的执行时间已经超过了局部轨迹预计的执行时间
      if (t_cur > info->duration_ - 1e-2){
        have_target_ = false;

        changeFSMExecState(WAIT_TARGET, "FSM");
        return;
      }
      // 当前位置与当前路标点的位置小于no_replan_thresh_
      else if ((end_pt_ - pos).norm() < no_replan_thresh_){
//         cout << "[planner] near end" << endl;

        return;
      }
      else if ((info->start_pos_ - pos).norm() < replan_thresh_){
//         cout << "[planner] near start" << endl;

        return;
      }
          // 当前位置与全局目标点距离大于no_replan_thresh_并且当前局部轨迹的执行时间超过了replan_thresh_
      else{
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case EMERGENCY_STOP:{
        // Avoiding repeated calls
      if (flag_escape_emergency_){
      // 在当前位置停下
        callEmergencyStop(odom_pos_);
      }
      else{
        if (odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);
  }

    bool EGOReplanFSM::planFromGlobalTraj(){
        start_pt_ = odom_pos_;
        start_vel_ = odom_vel_;
        start_acc_.setZero();

        bool flag_random_poly_init;
        if (timesOfConsecutiveStateCalls().first == 1)
            flag_random_poly_init = false;
        else
            flag_random_poly_init = true;

        if (callReboundReplan(true, flag_random_poly_init)){
            return true;
        }
        return false;
    }

  bool EGOReplanFSM::planFromCurrentTraj(){
    LocalTrajData *info = &planner_manager_->local_data_;
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - info->start_time_).toSec();

    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    // 设置起始位置速度为当前时刻局部轨迹计算出的位置和速度
    start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    bool success = callReboundReplan(false, false);

    if (!success){
      success = callReboundReplan(true, false);
      //changeFSMExecState(EXEC_TRAJ, "FSM");
      if (!success){
        success = callReboundReplan(true, true);
        if (!success){
          return false;
        }
      }
    }

    return true;
  }

  // 检查轨迹是否会和自己小于安全距离，如果小于安全距离，则切换到REPLAN_TRAJ状态
  void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e){
    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    if (exec_state_ == WAIT_TARGET || info->start_time_.toSec() < 1e-5)
      return;

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = (ros::Time::now() - info->start_time_).toSec();
    double t_2_3 = info->duration_ * 2 / 3;
    for (double t = t_cur; t < info->duration_; t += time_step){
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      // t时刻轨迹上为障碍物
      if (map->getInflateOccupancy(info->position_traj_.evaluateDeBoorT(t))){
        if (planFromCurrentTraj()) // Make a chance
        {
            // 切换到EXEC_TRAJ 执行新的安全路径
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        }
        else{
            // 将要碰撞的时间与现在的时间小于安全时间emergency_time_
          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
              // 切换到EMERGENCY_STOP 紧急停止
            ROS_WARN("Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else{
              // 切换到REPLAN_TRAJ 重新规划轨迹
            //ROS_WARN("current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj){
    // 调用getLocalTarget() 得到局部轨迹的目标点和到达该点时的速度
      getLocalTarget();

    bool plan_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    have_new_target_ = false;

    cout << "final_plan_success=" << plan_success << endl;

    if (plan_success){
      auto info = &planner_manager_->local_data_;

      /* publish traj */
      ego_planner::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i){
        geometry_msgs::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i){
        bspline.knots.push_back(knots(i));
      }

      // 发布新规划出的局部轨迹
      bspline_pub_.publish(bspline);

      visualization_->displayOptimalList(info->position_traj_.get_control_points(), 0);
    }

    return plan_success;
  }

  // 规划出到stop_pos 的B样条路径
  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos){
    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    ego_planner::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i){
      geometry_msgs::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i){
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_.publish(bspline);

    return true;
  }

  void EGOReplanFSM::getLocalTarget(){
    double t;

    double t_step = planning_horizen_ / 20 / planner_manager_->pp_.max_vel_;
    double dist_min = 9999, dist_min_t = 0.0;
    for (t = planner_manager_->global_data_.last_progress_time_;
        t < planner_manager_->global_data_.global_duration_;
        t += t_step){
        // 在全局轨迹planner_manager_->global_data_ 找到距离当前无人机位置planning_horizen_处的点作为局部轨迹的目标点
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist = (pos_t - start_pt_).norm();

      if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizen_){
        // todo
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");

        return;
      }
      if (dist < dist_min){
        dist_min = dist;
        dist_min_t = t;
      }
      if (dist >= planning_horizen_){
        local_target_pt_ = pos_t;
        // 将全局轨迹上离当前位置最近的点对应的时间赋值给planner_manager_->global_data_.last_progress_time_
        planner_manager_->global_data_.last_progress_time_ = dist_min_t;
        break;
      }
    }

    // Last global point 局部轨迹的目标点对应的时间大于了全局轨迹的时长
    if (t > planner_manager_->global_data_.global_duration_){
        // 将全局终点作为局部轨迹的目标点
      local_target_pt_ = end_pt_;
    }

    // 全局终点到局部轨迹的目标点的距离小于无人机以做大速度和加速度可飞行的距离
    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_)){
      // local_target_vel_ = (end_pt_ - init_pt_).normalized() * planner_manager_->pp_.max_vel_ * (( end_pt_ - local_target_pt_ ).norm() / ((planner_manager_->pp_.max_vel_*planner_manager_->pp_.max_vel_)/(2*planner_manager_->pp_.max_acc_)));
      // cout << "A" << endl;
      // 局部轨迹的目标点速度设为0
      local_target_vel_ = Eigen::Vector3d::Zero();
    }

    else{
        // 局部轨迹的目标点速度为全局轨迹在该点的速度
      local_target_vel_ = planner_manager_->global_data_.getVelocity(t);
      // cout << "AA" << endl;
    }
  }
} // namespace ego_planner
