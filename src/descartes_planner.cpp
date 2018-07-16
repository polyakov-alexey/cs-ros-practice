#include <cmath>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <descartes_trajectory/cart_trajectory_pt.h>
#include <descartes_moveit/moveit_state_adapter.h>
#include <descartes_planner/dense_planner.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

// Создание синонимов чисто для удобства
typedef std::vector<descartes_core::TrajectoryPtPtr> TrajectoryVec;
typedef TrajectoryVec::const_iterator TrajectoryIter;

// Функция из туториала
trajectory_msgs::JointTrajectory resultToJointTrajectory (const TrajectoryVec &trajectory,
                                                          const descartes_core::RobotModel &model,
                                                          const std::vector<std::string> &joint_names,
                                                          double time_delay)
{
    trajectory_msgs::JointTrajectory result;
    result.header.stamp = ros::Time::now();
    result.header.frame_id = "world";
    result.joint_names = joint_names;

    double time_offset = 0.0;
    for (TrajectoryIter it = trajectory.begin(); it != trajectory.end(); ++it)
    {
        std::vector<double> joints;
        it->get()->getNominalJointPose(std::vector<double>(), model, joints);

        trajectory_msgs::JointTrajectoryPoint pt;
        pt.positions = joints;

        pt.velocities.resize(joints.size(), 0.0);
        pt.accelerations.resize(joints.size(), 0.0);
        pt.effort.resize(joints.size(), 0.0);

        pt.time_from_start = ros::Duration(time_offset);

        time_offset += time_delay;

        result.points.push_back(pt);
    }

    return result;
}

// Функция из туториала
bool executeTrajectory(const trajectory_msgs::JointTrajectory &trajectory)
{
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("joint_trajectory_action", true);

    if (!ac.waitForServer(ros::Duration(10.0)))
    {
        ROS_ERROR("\n-- executeTrajectory() -> Server error\n\n");
        return false;
    }

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = trajectory;
    goal.goal_time_tolerance = ros::Duration(1.0);

    ac.sendGoal(goal);

    if (ac.waitForResult(goal.trajectory.points[goal.trajectory.points.size() - 1].time_from_start + ros::Duration(5)))
    {
        ROS_INFO("-- Success\n");
        return true;
    } else {
        ROS_WARN("\n-- Trajectory error\n\n");
        return false;
    }
}

// Переход в начальное положение (2, 0, 1)
// НЕ ИСПОЛЬЗУЕТСЯ
void GoToStartPosition()
{
    ROS_INFO("-- Goint to start position\n");

    geometry_msgs::Pose move_target;
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");

    //move_target.position.x = 2;
    move_target.position.x = 1;
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

// Создание массива точек-положений - два круга
TrajectoryVec makePointsCircle()
{
    TrajectoryVec points;

    for (float x = 0.5; x < M_PI * 4.5; x += 0.25)
    {
        Eigen::Affine3d pose;
        pose = Eigen::Translation3d(1.25, cos(x) * 0.25, sin(x) * 0.25 + 1); // for abb x=1.25
        pose *= Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY());

        descartes_core::TrajectoryPtPtr pt = descartes_core::TrajectoryPtPtr(new descartes_trajectory::CartTrajectoryPt(descartes_trajectory::TolerancedFrame(pose)));
        points.push_back(pt);
    }

    return points;
}

void MarkerSetup(visualization_msgs::Marker &mrk)
{
    mrk.header.frame_id = "/world";
    mrk.header.stamp = ros::Time::now();
    mrk.ns = "there_there_is_point";
    mrk.id = 0;
    mrk.action = visualization_msgs::Marker::ADD;

    mrk.pose.position.x = 0;
    mrk.pose.position.z = 0;

    mrk.type = visualization_msgs::Marker::POINTS;

    mrk.scale.x = 0.025;
    mrk.scale.y = 0.025;
    mrk.color.r = 0.0;
    mrk.color.g = 0.0;
    mrk.color.b = 1.0;
    mrk.color.a = 0.75;
}

// Создание массива точек-положений - буква S
TrajectoryVec makePointsS(visualization_msgs::Marker &mrk)
{
    TrajectoryVec points;

    float xOffset = 1.5;
    float center = 1.25;
    float radius = 0.25;
    float step = 0.2; // rad

    // Верхняя прямая
    for (float y = radius / 2 + radius; y > - radius / 2; y -= radius * step)
    {
        Eigen::Affine3d pose;
        pose = Eigen::Translation3d(xOffset, y, center + radius * 2);
        pose *= Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY());

        descartes_core::TrajectoryPtPtr pt = descartes_core::TrajectoryPtPtr(new descartes_trajectory::CartTrajectoryPt(descartes_trajectory::TolerancedFrame(pose)));
        points.push_back(pt);

        geometry_msgs::Point p;
        p.x = xOffset;
        p.y = y;
        p.z = center + radius * 2;
        mrk.points.push_back(p);
    }

    // Верхняя дуга
    for (float x = M_PI / 2; x < M_PI * 1.5; x += step)
    {
        Eigen::Affine3d pose;
        pose = Eigen::Translation3d(xOffset, - radius / 2 + cos(x) * radius, sin(x) * radius + center + radius);
        pose *= Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY());

        descartes_core::TrajectoryPtPtr pt = descartes_core::TrajectoryPtPtr(new descartes_trajectory::CartTrajectoryPt(descartes_trajectory::TolerancedFrame(pose)));
        points.push_back(pt);

        geometry_msgs::Point p;
        p.x = xOffset;
        p.y = - radius / 2 + cos(x) * radius;
        p.z = sin(x) * radius + center + radius;
        mrk.points.push_back(p);
    }

    // Центральная прямая
    for (float y = - radius / 2; y < radius / 2; y += radius * step)
    {
        Eigen::Affine3d pose;
        pose = Eigen::Translation3d(xOffset, y, center);
        pose *= Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY());

        descartes_core::TrajectoryPtPtr pt = descartes_core::TrajectoryPtPtr(new descartes_trajectory::CartTrajectoryPt(descartes_trajectory::TolerancedFrame(pose)));
        points.push_back(pt);

        geometry_msgs::Point p;
        p.x = xOffset;
        p.y = y;
        p.z = center;
        mrk.points.push_back(p);
    }

    // Нижняя дуга
    for (float x = M_PI / 2; x > - M_PI / 2; x -= step)
    {
        Eigen::Affine3d pose;
        pose = Eigen::Translation3d(xOffset, radius / 2 + cos(x) * radius, sin(x) * radius + center - radius);
        pose *= Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY());

        descartes_core::TrajectoryPtPtr pt = descartes_core::TrajectoryPtPtr(new descartes_trajectory::CartTrajectoryPt(descartes_trajectory::TolerancedFrame(pose)));
        points.push_back(pt);

        geometry_msgs::Point p;
        p.x = xOffset;
        p.y = radius / 2 + cos(x) * radius;
        p.z = sin(x) * radius + center - radius;
        mrk.points.push_back(p);
    }

    // Нижняя прямая
    for (float y = radius / 2; y > - radius / 2 - radius; y -= radius * step)
    {
        Eigen::Affine3d pose;
        pose = Eigen::Translation3d(xOffset, y, center - radius * 2);
        pose *= Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY());

        descartes_core::TrajectoryPtPtr pt = descartes_core::TrajectoryPtPtr(new descartes_trajectory::CartTrajectoryPt(descartes_trajectory::TolerancedFrame(pose)));
        points.push_back(pt);

        geometry_msgs::Point p;
        p.x = xOffset;
        p.y = y;
        p.z = center - radius * 2;
        mrk.points.push_back(p);
    }

    return points;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "descartes_planner");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10, true);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    visualization_msgs::Marker mrk;
    MarkerSetup(mrk);

    // Переход в начальное положение (2, 0, 1)
    //GoToStartPosition();

    // Создание массива точек-положений
    //TrajectoryVec points = makePointsCircle();
    TrajectoryVec points = makePointsS(mrk);

    // Подключение робота в планировщик
    descartes_core::RobotModelPtr model (new descartes_moveit::MoveitStateAdapter);
    if (!model->initialize("robot_description", "manipulator", "base_link", "tool0"))
    {
        ROS_ERROR("\n-- Error during model.initialize()\n\n");
        return -1;
    }

    // Создание планнера и подключение в него модели робота
    descartes_planner::DensePlanner planner;
    planner.initialize(model);

    // Передача пути планировщику
    if (!planner.planPath(points))
    {
        ROS_ERROR("\n-- Error during planner.planPath()\n\n");
        return -1;
    }

    // Получение пути
    TrajectoryVec result;
    if (!planner.getPath(result))
    {
        ROS_ERROR("\n-- Error during planner.getPath()\n\n");
        return -1;
    }

    // Перевод данных из формата descartes в формат, который понимает ros
    std::vector<std::string> names;
    n.getParam("controller_joint_names", names);
    trajectory_msgs::JointTrajectory joint_solution = resultToJointTrajectory(result, *model, names, 0.25);

    // Отрисовка маркеров
    pub.publish(mrk);

    // Выполнение пути роботом
    if (!executeTrajectory(joint_solution))
    {
        ROS_ERROR("\n-- Error during executeTrajectory()\n\n");
        return -1;
    }

    ROS_INFO("\n-- DONE\n\n");

    return 0;
}
