#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

void GoToStartPosition()
{
    ROS_INFO("-- Goint to start position\n");

    geometry_msgs::Pose move_target;
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");

    move_target.position.x = 2;
    move_target.position.y = 0;
    move_target.position.z = 1;
    tf::Quaternion q;
    q.setRPY(0, 1.57, 0);
    move_target.orientation.x = q.x();
    move_target.orientation.y = q.y();
    move_target.orientation.z = q.z();
    move_target.orientation.w = q.w();

    move_group.setPoseTarget(move_target);
    move_group.move();

    ROS_INFO("-- Done\n");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "descartes_planner");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    GoToStartPosition();

    return 0;
}
