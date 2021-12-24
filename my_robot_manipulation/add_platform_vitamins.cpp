#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "robot_arm";

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


  //platform
  moveit_msgs::CollisionObject collision_platform;
  collision_platform.header.frame_id = move_group.getPlanningFrame();
 
  collision_platform.id = "platform";


  // Здесь меняем размер платформы
  shape_msgs::SolidPrimitive primitive_platform;
  primitive_platform.type = primitive_platform.BOX;
  primitive_platform.dimensions.resize(3);
  primitive_platform.dimensions[0] = 0.21;
  primitive_platform.dimensions[1] = 0.21;
  primitive_platform.dimensions[2] = 0.13;


  // Здесь меняем положение платформы
  geometry_msgs::Pose platform_pose;
  platform_pose.orientation.w = 1.0;
  platform_pose.position.x = -0.4;
  platform_pose.position.y = 0;
  platform_pose.position.z = 0.0625;

  collision_platform.primitives.push_back(primitive_platform);
  collision_platform.primitive_poses.push_back(platform_pose);
  collision_platform.operation = collision_platform.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_object1;
  collision_object1.push_back(collision_platform);

  planning_scene_interface.addCollisionObjects(collision_object1);

  //vitamins box
  moveit_msgs::CollisionObject collision_vitamins;
  collision_vitamins.header.frame_id = move_group.getPlanningFrame();
 
  collision_vitamins.id = "vitamins";

  // Здесь меняем размер баночки
  shape_msgs::SolidPrimitive primitive_vitamins;
  primitive_vitamins.type = primitive_vitamins.CYLINDER;
  primitive_vitamins.dimensions.resize(2);
  primitive_vitamins.dimensions[primitive_vitamins.CYLINDER_HEIGHT] = 0.07;
  primitive_vitamins.dimensions[primitive_vitamins.CYLINDER_RADIUS] = 0.0225;

  // Здесь меняем положение баночки
  geometry_msgs::Pose vitamins_pose;
  vitamins_pose.orientation.w = 1.0;
  vitamins_pose.position.x = -0.4;
  vitamins_pose.position.y = 0;
  vitamins_pose.position.z = 0.1625;

  collision_vitamins.primitives.push_back(primitive_vitamins);
  collision_vitamins.primitive_poses.push_back(vitamins_pose);
  collision_vitamins.operation = collision_vitamins.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_object3;
  collision_object3.push_back(collision_vitamins);
  
  planning_scene_interface.addCollisionObjects(collision_object3);

  ros::shutdown();
  return 0;
}
