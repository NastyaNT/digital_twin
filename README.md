# description
Модель робота с датчиками и контроллерами.
===========================================
Управление роботом
------------
1. 1й терминал: roslaunch description gazebo.launch - загрузка модели робота с датчиками и контроллерами в окне gazebo. 

Вариант 1
------------
2. 2й терминал: rosrun rqt_gui rqt_gui 
3. После открытия переходим в Plugins -> Topics -> Message Publisher. В Topic вводим /robot/*название звена*/command (пример: /robot/joint1/command). В Freq ставим 50. Нажимаем +. После добавлеия топика в поле, раскрываем его, нажимая на треугольник. В поле expression вводим значение в радианах. И ставим галочку в квадратике перед названием топика.

Вриант 2
------------
2. Управление через консоль (2й терминал): rostopic pub /robot/*название звена*/command std_msgs/Float64 "data: *значение в радианах*" (пример: rostopic pub /robot/joint1/command std_msgs/Float64 "data: -1")

Сбор данных с датчиков
------------
1) 1й терминал: roslaunch description gazebo.launch - загрузка модели робота с датчиками и контроллерами в окне gazebo. 
2) 2й терминал: rostopic echo *название* -n1 > *имя файла, куда запишется вывод* - единоразовая запись данных в файл (пример: rostopic echo imu1/data -n1 > imu1_experiment1_1_before)

# moveit_config
Пакет с моделью робота для MoveIt.
------------
roslaunch moveit_config demo_gazebo.launch - запуск Moveit и Gazebo, загрузка датчиков и контроллеров.

# my_robot_manipulation
box_capture_rise.cpp:
===========================================
Проведение экспериментов по захвату и поднятию коробки.
------------ 
Программа осществляет добавление платформы размером 0.21х0.21х0.13м и объекта 
для манипулирования (коробка) размером 0.065х0.008х0.03м перед моделью робота. 
Осуществение приклепления объекта к роботу, открепления объекта от робота 
и удаления платформы и объекта с рабочего пространства.

box_capture_transfer.cpp:
===========================================
Проведение экспериментов по захвату,поднятию и перемещению коробки на другую платформу.
------------  
Программа осществляет добавление платформы размером 0.21х0.21х0.13м и объекта 
для манипулирования (коробка) размером 0.065х0.008х0.03м перед моделью робота.
Добавление платформы размером 0.21х0.21х0.13м справа от модели робота.
Осуществение приклепления объекта к роботу, открепления объекта от робота 
и удаления платформ и объекта с рабочего пространства.

move.cpp:
===========================================
Проведение экспериментов по перемещению звеньев манипулятора в заданные положения.
------------ 
Программа осществляет задание положения для звеньев манипулятора. 
Выполнение планирования и расчет траектории движения манипулятора. 
Выполнение движения звеньев манипуляторпо запланированной траектории.

move_to_pose.cpp:
===========================================
Проведение экспериментов по перемещению эфектора в заданную точку пространства.
------------ 
Программа осществляет задание целевой позиции для эффектора манипулятора. 
Выполнение планирования и расчет траектории движения манипулятора. 
Выполнение движения звеньев манипуляторпо запланированной траектории.

robot_state.cpp:
===========================================
Получение информации о модели и состоянии робота.
------------ 
Программа осществляет получени текущего набора совместных значений, хранащихся в состоянии для манипулятора.
Вычисление прямой и обратной кинематики.

vitamins_capture_rise.cpp:
===========================================
Проведение экспериментов по захвату и поднятию баночки витаминов.
------------ 
Программа осществляет добавление платформы размером 0.21х0.21х0.13м и объекта 
для манипулирования (баночка витаминов) с радиусом 0.0225м и высотой 0.07м перед моделью робота. 
Осуществение приклепления объекта к роботу, открепления объекта от робота 
и удаления платформы и объекта с рабочего пространства.

vitamins_capture_transfer.cpp:
===========================================
Проведение экспериментов по захвату,поднятию и перемещению баночки витаминов на другую платформу.
------------ 
Программа осществляет добавление платформы размером 0.21х0.21х0.13м и объекта 
для манипулирования (баночка витаминов) с радиусом 0.0225м и высотой 0.07м перед моделью робота.
Добавление платформы размером 0.21х0.21х0.13м справа от модели робота.
Осуществение приклепления объекта к роботу, открепления объекта от робота 
и удаления платформ и объекта с рабочего пространства.
