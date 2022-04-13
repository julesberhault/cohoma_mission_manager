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

    // Parameters
    std::string utm_grid_zone;
    n.param<std::string>("utm_zone", utm_grid_zone, "31n");
    GeographicLib::UTMUPS::DecodeZone(utm_grid_zone, m_utm_zone, m_northp);

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

bool MissionManager::pushMissionService(cohoma_msgs::PushMission::Request& req, cohoma_msgs::PushMission::Response& res)
{
    m_waypoints = req.waypoints;
    m_cur_waypoint_seq = req.current_seq;
    ROS_INFO("Mission received");
    res.success = true;
    return true;
}

bool MissionManager::launchMissionService(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    if (m_waypoints.size())
    {
        ROS_INFO("Mission launched");
        setNextGoal();
        res.success = true;
        res.message = "Mission launched with success";
        return true;
    }
    res.success = false;
    res.message = "No waypoint received. Unable to launch mission";
    return false;
}

bool MissionManager::abortMissionService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    m_moveBaseCancelPublisher.publish(m_cur_goal_id);
    if (m_cur_waypoint_seq)
    {
        m_waypoints.erase(m_waypoints.begin()+m_cur_waypoint_seq-1);
    }
    m_cur_waypoint_seq = 0;
    ROS_INFO_STREAM(m_waypoints.size());
    ROS_INFO("Mission aborted");
    return true;
}

// -------------------- Functions --------------------

void MissionManager::setNextGoal()
{
    if (m_cur_waypoint_seq < m_waypoints.size())
    {
        buildNextGoal();
        setMoveBaseGoal(m_next_goal, m_cur_goal_id);
    }
}

void MissionManager::buildNextGoal()
{
    geographic_msgs::GeoPoint cur_geo_point = m_waypoints.at(m_cur_waypoint_seq).position;
    m_cur_goal_id = generateID();
    m_next_goal = getTargetPose(cur_geo_point);
}

geometry_msgs::PointStamped MissionManager::latLongToUtm(geographic_msgs::GeoPoint& _geo_point)
{
    geometry_msgs::PointStamped utm_point;
    utm_point.header.frame_id = "utm";
    utm_point.header.stamp = ros::Time::now();
    GeographicLib::UTMUPS::Forward(_geo_point.latitude, _geo_point.longitude, m_utm_zone, m_northp, utm_point.point.x, utm_point.point.y);
    utm_point.point.z = 0;

    return utm_point;
}

geographic_msgs::GeoPoint MissionManager::utmToLatLong(geometry_msgs::PointStamped& _utm_point)
{
    geographic_msgs::GeoPoint geo_point;
    GeographicLib::UTMUPS::Reverse(m_utm_zone, m_northp, _utm_point.point.x, _utm_point.point.y, geo_point.latitude, geo_point.longitude);
    geo_point.altitude = 0;

    return geo_point;
}

geometry_msgs::PointStamped MissionManager::utmToOdom(geometry_msgs::PointStamped& _utm_point)
{
    geometry_msgs::PointStamped odom_point;
    bool notDone = true;
    tf::TransformListener listener;
    ros::Time time_now = ros::Time::now();
    while(notDone)
    {
        try
        {
            _utm_point.header.stamp = ros::Time::now();
            listener.waitForTransform("odom", "utm", time_now, ros::Duration(0.5));
            listener.transformPoint("odom", _utm_point, odom_point);
            notDone = false;
        }
        catch (tf::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(0.1).sleep();
        }
    }
    return odom_point;
}

geometry_msgs::PointStamped MissionManager::odomToUtm(geometry_msgs::PointStamped& _odom_point)
{
    geometry_msgs::PointStamped utm_point;
    bool notDone = true;
    tf::TransformListener listener;
    ros::Time time_now = ros::Time::now();
    while(notDone)
    {
        try
        {
            _odom_point.header.stamp = ros::Time::now();
            listener.waitForTransform("utm", "odom", time_now, ros::Duration(0.5));
            listener.transformPoint("utm", _odom_point, utm_point);
            notDone = false;
        }
        catch (tf::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(0.1).sleep();
        }
    }
    return utm_point;
}

geometry_msgs::PoseStamped MissionManager::getTargetPose(geographic_msgs::GeoPoint& _geo_point)
{
    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id = "odom";
    target_pose.header.stamp = ros::Time::now();
    geometry_msgs::PointStamped utm_point = latLongToUtm(_geo_point);
    geometry_msgs::PointStamped target_point = utmToOdom(utm_point);
    target_pose.pose.position.x = target_point.point.x;
    target_pose.pose.position.y = target_point.point.y;
    target_pose.pose.orientation.w = 1.0;
    m_sequence++;
    target_pose.header.seq = m_sequence;
    ROS_INFO_STREAM("Goal setted to :"<<"\n"<<"    latitude: "<<_geo_point.latitude<<"\n"<<"   longitude: "<<_geo_point.longitude <<"\n"<<"           x: "<<target_pose.pose.position.x <<"\n"<< "           y: "<<target_pose.pose.position.y);
    return target_pose;
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