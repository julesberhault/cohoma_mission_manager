#include "mission_manager.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cohoma_mission_manager_node");
    ros::NodeHandle n("~");
    MissionManager MissionManager(n);
    ros::spin();
    return 0;
}