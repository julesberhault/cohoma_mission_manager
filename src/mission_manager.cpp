#include "mission_manager.h"

MissionManager::MissionManager(ros::NodeHandle& n)
{
    // Subscribers
    m_missionPlanSubscriber = n.subscribe("/mission/mission_plan", 10, &MissionManager::missionPlanCallback, this);
    m_missionContextSubscriber = n.subscribe("/mission/mission_context", 10, &MissionManager::missionContextCallback, this);
    // m_moveBaseStatusSubscriber = n.subscribe("/move_base/status", 10, &MissionManager::moveBaseStatusCallback, this);
    m_moveBaseResultSubscriber = n.subscribe("/move_base/result", 10, &MissionManager::moveBaseResultCallback, this);

    // Publishers
    m_moveBaseCancelPublisher = n.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
    m_moveBaseGoalPublisher = n.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1);

    // Service Clients
    m_pushMissionServer = n.advertiseService("/mission/push_mission", &MissionManager::pushMissionService, this);
    m_launchMissionServer = n.advertiseService("/mission/launch_mission", &MissionManager::launchMissionService, this);
    m_abortMissionServer = n.advertiseService("/mission/abort_mission", &MissionManager::abortMissionService, this);

    // Variables
    m_sequence = 0;
};

// -------------------- Callbacks --------------------

void MissionManager::missionPlanCallback(const cohoma_msgs::MissionPlan& _mission_plan)
{
    m_waypoints = _mission_plan.waypoints;
    m_cur_waypoint_seq = _mission_plan.current_seq;
    ROS_INFO("Mission received");
}

void MissionManager::missionContextCallback(const cohoma_msgs::MissionContext& _mission_context)
{
    m_strategic_points = _mission_context.strategic_points;
}

void MissionManager::moveBaseStatusCallback(const actionlib_msgs::GoalStatusArray& _goal_status_array)
{
    
}

void MissionManager::moveBaseResultCallback(const move_base_msgs::MoveBaseActionResult& _move_base_action_result)
{
    if (_move_base_action_result.status.goal_id == m_cur_goal_id)
    {
        if (_move_base_action_result.status.status == actionlib_msgs::GoalStatus::SUCCEEDED)
        {
            ROS_INFO_STREAM(_move_base_action_result.status.text);
            m_cur_waypoint_seq++;
            setNextGoal();
        }
        else if (_move_base_action_result.status.status == actionlib_msgs::GoalStatus::ABORTED || _move_base_action_result.status.status == actionlib_msgs::GoalStatus::REJECTED || _move_base_action_result.status.status == actionlib_msgs::GoalStatus::LOST)
        {
            ROS_ERROR_STREAM(_move_base_action_result.status.text);
            m_cur_waypoint_seq++;
            setNextGoal();
        }
        else
        {
            ROS_INFO_STREAM(_move_base_action_result.status.text);
        }
    }
}

// -------------------- Service servers --------------------

bool MissionManager::pushMissionService(cohoma_msgs::PushMission::Request& req, cohoma_msgs::PushMission::Request& res)
{
    m_waypoints = req.waypoints;
    m_cur_waypoint_seq = req.current_seq;
    return true;
}

bool MissionManager::launchMissionService(std_srvs::Empty::Request& req, std_srvs::Empty::Request& res)
{
    if (m_waypoints.size())
    {
        ROS_INFO("Launch mission");
        setNextGoal();
        return true;
    }
    return false;
}

bool MissionManager::abortMissionService(std_srvs::Empty::Request& req, std_srvs::Empty::Request& res)
{
    m_moveBaseCancelPublisher.publish(m_cur_goal_id);
    return true;
}

// -------------------- Functions --------------------

void MissionManager::setNextGoal()
{
    if (m_cur_waypoint_seq < m_waypoints.size())
    {
        geographic_msgs::GeoPoint cur_geo_point = m_waypoints.at(m_cur_waypoint_seq).position;
        m_cur_goal_id = generateID();
        setMoveBaseGoal(getTargetPose(cur_geo_point), m_cur_goal_id);
    }
}

geometry_msgs::PoseStamped MissionManager::getTargetPose(geographic_msgs::GeoPoint& _geo_point)
{
    geometry_msgs::PoseStamped target_pose_stamped_utm;
    geometry_msgs::PoseStamped target_pose_stamped_odom;

    int zone;
    bool northp;
    GeographicLib::UTMUPS::Forward(_geo_point.latitude, _geo_point.longitude, zone, northp, target_pose_stamped_utm.pose.position.x, target_pose_stamped_utm.pose.position.y);
    target_pose_stamped_utm.pose.orientation.w = 1.0;
    
    ros::Time time_now = ros::Time::now();
    target_pose_stamped_utm.header.stamp = time_now;
    target_pose_stamped_utm.header.frame_id = "utm";
    bool notDone = true;
    tf::TransformListener listener;
    while(notDone)
    {
        try
        {
            target_pose_stamped_utm.header.stamp = ros::Time::now();
            listener.waitForTransform("odom", "utm", time_now, ros::Duration(3.0));
            listener.transformPose("odom", target_pose_stamped_utm, target_pose_stamped_odom);
            notDone = false;
        }
        catch (tf::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(0.01).sleep();
        }
    }
    target_pose_stamped_odom.header.stamp = time_now;
    target_pose_stamped_odom.header.frame_id = "odom";
    m_sequence++;
    target_pose_stamped_odom.header.seq = m_sequence;

    ROS_INFO_STREAM("Goal setted to :"<<"\n"<<"    latitude: "<<_geo_point.latitude<<"\n"<<"   longitude: "<<_geo_point.longitude <<"\n"<<"           x: "<<target_pose_stamped_odom.pose.position.x <<"\n"<< "           y: "<<target_pose_stamped_odom.pose.position.y);
    ROS_INFO_STREAM("Goal setted to :"<<"\n"<<"       utm_x: "<<target_pose_stamped_utm.pose.position.x <<"\n"<< "       utm_y: "<<target_pose_stamped_utm.pose.position.y);
    
    return target_pose_stamped_odom;
}

void MissionManager::setMoveBaseGoal(const geometry_msgs::PoseStamped& _target_pose, actionlib_msgs::GoalID& _goal_id)
{
    move_base_msgs::MoveBaseActionGoal goal;
    goal.goal.target_pose = _target_pose;
    goal.goal_id = _goal_id;
    goal.header = _target_pose.header;
    m_moveBaseGoalPublisher.publish(goal);
}

actionlib_msgs::GoalID MissionManager::generateID()
{
    actionlib_msgs::GoalID goal_id;
    ros::Time time_now = ros::Time::now();
    std::stringstream ss;
    ss << "mission_manager-";
    ss << time_now.sec << "." << time_now.nsec;
    goal_id.id = ss.str();
    goal_id.stamp = time_now;
    return goal_id;
}