#pragma once

#include "ros/ros.h"

// Standard
#include <sstream>
#include <string>

// ROS
#include "geometry_msgs/PoseStamped.h"
#include "geographic_msgs/GeoPoint.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/GetPlan.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "actionlib_msgs/GoalID.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "move_base_msgs/MoveBaseGoal.h"
#include "cohoma_msgs/MissionPlan.h"
#include "cohoma_msgs/WayPoint.h"
#include "cohoma_msgs/MissionContext.h"
#include "cohoma_msgs/StrategicPoint.h"
#include "cohoma_msgs/PushMission.h"
#include "std_srvs/Empty.h"
#include "tf2/LinearMath/Quaternion.h"

// Geographic Lib
#include <GeographicLib/LocalCartesian.hpp>

class MissionManager
{
    
    public:
        MissionManager(ros::NodeHandle& n);

        // -------------------- Callbacks --------------------

        // Message containing mission plan sent from CoHoMa HMI
        void missionPlanCallback(const cohoma_msgs::MissionPlan& mission_plan);
        // Message containing mission context sent from anyone
        void missionContextCallback(const cohoma_msgs::MissionContext& mission_context);
        // Message containing move base goal status
        void moveBaseStatusCallback(const actionlib_msgs::GoalStatusArray& goal_status_array);
        // Message containing move base goal results
        void moveBaseResultCallback(const move_base_msgs::MoveBaseActionResult& move_base_action_result);

        void latLonRefCallback(const sensor_msgs::NavSatFix& lat_lon_ref);

        // -------------------- Service servers --------------------

        bool pushMissionService(cohoma_msgs::PushMission::Request& req, cohoma_msgs::PushMission::Request& res);
        bool launchMissionService(std_srvs::Empty::Request& req, std_srvs::Empty::Request& res);
        bool abortMissionService(std_srvs::Empty::Request& req, std_srvs::Empty::Request& res);

        // -------------------- Functions --------------------

        
        void setNextGoal();

        void setMoveBaseGoal(const geometry_msgs::PoseStamped& target_pose, actionlib_msgs::GoalID& _goal_id);

        geometry_msgs::PoseStamped getTargetPose(geographic_msgs::GeoPoint& geo_point);

        actionlib_msgs::GoalID generateID();

    private:
        // Subscribers
        ros::Subscriber m_missionPlanSubscriber;
        ros::Subscriber m_missionContextSubscriber;
        ros::Subscriber m_moveBaseStatusSubscriber;
        ros::Subscriber m_moveBaseResultSubscriber;
        
        // Publishers
        ros::Publisher m_moveBaseCancelPublisher;
        ros::Publisher m_moveBaseGoalPublisher;

        // Service server
        ros::ServiceServer m_pushMissionServer;
        ros::ServiceServer m_abortMissionServer;
        ros::ServiceServer m_launchMissionServer;

        // Mission data
        std::vector<cohoma_msgs::WayPoint> m_waypoints;
        std::vector<cohoma_msgs::StrategicPoint> m_strategic_points;
        int m_cur_waypoint_seq;
        actionlib_msgs::GoalID m_cur_goal_id;
        std::string m_frame_id;
        int m_sequence;

        // Projection
        float m_lat_ref;
        float m_lon_ref;
        GeographicLib::LocalCartesian m_local;
};