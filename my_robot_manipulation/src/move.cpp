#include <string>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_fk_demo");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("robot_arm");

    // Устанавливаем допустимую ошибку
	arm.setGoalJointTolerance(0.001);

    // Устанавливаем максимально допустимую скорость и ускорение
    arm.setMaxAccelerationScalingFactor(0.5);
    arm.setMaxVelocityScalingFactor(0.5);

	double targetPose[5] = {0.01, -2.02, 1.53, -0.01, 0.49};
	std::vector<double> joint_group_positions(5);
	joint_group_positions[0] = targetPose[0];
	joint_group_positions[1] = targetPose[1];
	joint_group_positions[2] = targetPose[2];
	joint_group_positions[3] = targetPose[3];
	joint_group_positions[4] = targetPose[4];

	arm.setJointValueTarget(joint_group_positions);

    // Выполняем планирование движения и рассчитываем траекторию движения робота, движущегося к цели. В настоящее время он только рассчитывает траекторию и не контролирует движение робота
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::planning_interface::MoveItErrorCode success = arm.plan(plan);

    ROS_INFO("Plan (pose goal) %s",success?"":"FAILED");   

    // Дать роботу-манипулятору начать движение по запланированной траектории.

    arm.execute(plan);

	arm.move();

    ros::shutdown(); 

    return 0;
}

