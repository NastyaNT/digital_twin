Gazebo: 
roslaunch model_gazebo model_world.launch

Загрузка контроллеров:
roslaunch model_control model_control.launch

Пример передвижения звеньев: 
rostopic pub /robot/joint1_position_controller/command std_msgs/Float64 "data: -1"
