#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

//==========
// SETTINGS
//==========
static float xOffset = 1.5;
static float center = 1.25;
static float radius = 0.25;
static float step = 0.2;
//==========



void GoToStartPosition(moveit::planning_interface::MoveGroupInterface &move_group)
{
    ROS_INFO("-- Going to start position");

    geometry_msgs::Pose pose;

    pose.position.x = xOffset;
    pose.position.y = radius / 2 + radius;
    pose.position.z = center + radius * 2;
    tf::Quaternion q;
    q.setRPY(0, M_PI/2, 0);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    move_group.setPoseTarget(pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
    {
        move_group.move();
        ROS_INFO("-- done");
    } else {
        ROS_WARN("-- Unable to plan");
    }
}



// Начальная инициализация маркеров
void MarkerSetup(visualization_msgs::Marker &marker)
{
    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "points";
    marker.id = 0;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = 0;
    marker.pose.position.z = 0;

    marker.type = visualization_msgs::Marker::POINTS;

    marker.scale.x = 0.025;
    marker.scale.y = 0.025;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 0.75;
}



void MakePathS(std::vector<geometry_msgs::Pose> &waypoints, visualization_msgs::Marker &marker)
{
    geometry_msgs::Point point;
    geometry_msgs::Pose pose;
    tf::Quaternion q;
    q.setRPY(0, M_PI/2, 0);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    // Верхняя прямая
    for (float y = radius / 2 + radius; y > - radius / 2; y -= radius * step)
    {
        pose.position.x = point.x = xOffset;
        pose.position.y = point.y = y;
        pose.position.z = point.z = center + radius * 2;
        waypoints.push_back(pose);
        marker.points.push_back(point);
    }

    // Верхняя дуга
    for (float x = M_PI / 2; x < M_PI * 1.5; x += step)
    {
        pose.position.x = point.x = xOffset;
        pose.position.y = point.y = - radius / 2 + cos(x) * radius;
        pose.position.z = point.z = sin(x) * radius + center + radius;
        waypoints.push_back(pose);
        marker.points.push_back(point);
    }

    // Центральная прямая
    for (float y = - radius / 2; y < radius / 2; y += radius * step)
    {
        pose.position.x = point.x = xOffset;
        pose.position.y = point.y = y;
        pose.position.z = point.z = center;
        waypoints.push_back(pose);
        marker.points.push_back(point);
    }

    // Нижняя дуга
    for (float x = M_PI / 2; x > - M_PI / 2; x -= step)
    {
        pose.position.x = point.x = xOffset;
        pose.position.y = point.y = radius / 2 + cos(x) * radius;
        pose.position.z = point.z = sin(x) * radius + center - radius;
        waypoints.push_back(pose);
        marker.points.push_back(point);
    }

    // Нижняя прямая
    for (float y = radius / 2; y > - radius / 2 - radius; y -= radius * step)
    {
        pose.position.x = point.x = xOffset;
        pose.position.y = point.y = y;
        pose.position.z = point.z = center - radius * 2;
        waypoints.push_back(pose);
        marker.points.push_back(point);
    }
}



int main (int argc, char **argv)
{
    ros::init(argc, argv, "cs_moveit_planner");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10, true);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    GoToStartPosition(move_group);

    visualization_msgs::Marker marker;
    MarkerSetup(marker);

    std::vector<geometry_msgs::Pose> waypoints;

    MakePathS(waypoints, marker);

    move_group.setMaxVelocityScalingFactor(0.1);

    moveit_msgs::RobotTrajectory trajectory;

    const double jump_treshold = 0.0;
    const double eef_step = 0.01;
    ROS_INFO("-- Calculating trajectory");
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_treshold, trajectory);

    ROS_INFO("-- Publishing markers");
    pub.publish(marker);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    ROS_INFO("-- Execution of planned trajectory");
    move_group.execute(plan);
    ROS_INFO("-- DONE");

    return 0;
}
