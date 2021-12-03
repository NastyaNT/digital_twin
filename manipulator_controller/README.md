Gazebo: 
roslaunch gazebo world.launch

Загрузка контроллеров:
roslaunch control control.launch

Пример передвижения звеньев: 
rostopic pub /robot/joint1_position_controller/command std_msgs/Float64 "data: -1"
