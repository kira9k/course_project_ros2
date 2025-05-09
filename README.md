
# Курсовой проект

## Оглавление

- [Введение](#введение)
- [Инструкции по сборке и запуску Docker-контейнера](#инструкции-по-сборке-и-запуску-docker-контейнера)
- [Основная часть](#основная-часть)
    - [Характеристики робота и датчиков](#характеристики-робота-и-датчиков)
    - [Запуск симуляции](#запуск-симуляции)
    - [Картографирование с помощью SLAM Toolbox](#картографирование-с-помощью-slam-toolbox)
    - [Использование Nav2 с bcr_bot](#использование-nav2-с-bcr_bot)
- [Заключение](#заключение)
- [Благодаронсти](#благодарности)
- [Список литературы](#список-литературы)


## Введение
Автономная навигация — ключевая задача робототехники. Данная курсовая работа посвящена изучению и запуску готового проекта навигации робота bcr_bot с использованием ROS 2 и Nav2 в симулированной среде. Цель — освоить локализацию (AMCL), планирование пути и работу с сенсорами (лидар, IMU, Kinect). Проект основан на репозитории [bcr_bot](<https://github.com/blackcoffeerobotics/bcr_bot>) 

## Инструкции по сборке и запуску Docker-контейнера
Для сборки Docker-образа необходимо выполнить bash-скрипт:

```bash
bash run_docker.sh 
```

Запуск контейнера:

```bash
bash run_docker.sh 
```

Открытие еще одного терминала Docker-контейнера:
```bash
bash new_terminal.sh
```

## Основная часть
### Характеристики робота и датчиков
Рассматриваемый робот представлен на рисунке ниже.

![Мобильный робот](images/mobile_robot.png)

Параметры робота и датчиков прописаны в ```gazebo.xacro``` и в ```bcr_bot.xacro```.

Он состоит из двух тяговых колес (дифференциальная передача) и четырех опорных роликов. Основные характеристики:
- Радиус колес: 0.1 м
- Расстояние между колесами: 0.6 м
- Радиус роликов: 0.06 м
- Максимальный крутящий момент двигателей: 20000 Н*м
- Масса робота: 73.51 кг
- Длина корпуса: 0.9 м
- Ширина корпуса: 0.64 м
- Высота корпуса: 0.19 м

Параметры датчиков:
1. **2D-лидар**
```xml
 <!-- ........................... 2D LIDAR PLUGIN ................................... -->

    <xacro:if value="$(arg two_d_lidar_enabled)">

        <gazebo reference="two_d_lidar">
            <gravity>true</gravity>
            <sensor type="ray" name="two_d_lidar">
                <pose>0 0 0 0 0 0</pose>
                <visualize>false</visualize>
                <update_rate>${two_d_lidar_update_rate}</update_rate>
                <ray>
                    <scan>
                        <horizontal>
                            <samples>${two_d_lidar_sample_size}</samples>
                            <resolution>1</resolution>
                            <min_angle>${radians(two_d_lidar_min_angle)}</min_angle>
                            <max_angle>${radians(two_d_lidar_max_angle)}</max_angle>
                        </horizontal>
                    </scan>
                    <range>
                        <min>${two_d_lidar_min_range}</min>
                        <max>${two_d_lidar_max_range}</max>
                        <resolution>0.01</resolution>
                    </range>
                    <noise>
                        <type>gaussian</type>
                        <mean>0.0</mean>
                        <stddev>0.0</stddev>
                    </noise>
                </ray>
                <plugin name="gazebo_ros_laser" filename="libgazebo_ros_ray_sensor.so">
                    <ros>
                        <remapping>~/out:=$(arg robot_namespace)/scan</remapping>
                    </ros>
                    <output_type>sensor_msgs/LaserScan</output_type>
                    <frame_name>two_d_lidar</frame_name>
                </plugin>
            </sensor>
        </gazebo>
    </xacro:if>
```

Для лидара указывается:
- Количество лучей = 361
- Частота = 30 Гц
- Минимальное расстояние = 0.55 м
- Максимальное расстояние = 16 м
- Максимальный угол = 360°
- Минимальный угол = 0°

2. **Камера глубины**
```xml
    <!-- ........................... CAMERA PLUGIN ................................... -->

    <xacro:if value="$(arg camera_enabled)">
        <gazebo reference="kinect_camera">
            <sensor name="kinect_camera" type="depth">
                <pose> 0 0 0 0 0 0 </pose>
                <visualize>true</visualize>
                <update_rate>30</update_rate>
                <camera>
                    <horizontal_fov>1.089</horizontal_fov>
                    <image>
                        <format>R8G8B8</format>
                        <width>640</width>
                        <height>480</height>
                    </image>
                    <clip>
                        <near>0.05</near>
                        <far>8.0</far>
                    </clip>
                </camera>
                <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
                    <ros>
                        <namespace>$(arg robot_namespace)</namespace>
                    </ros>
                    <frame_name>kinect_camera_optical</frame_name>
                    <min_depth>0.1</min_depth>
                    <max_depth>100</max_depth>
                </plugin>
            </sensor>
        </gazebo>
    </xacro:if>
```
Для камеры глубины указывается:
- Частота обновления = 30 Гц
- Размер изображения = 640x480
- Диапазон видимости камеры: near = 0.05 м, far = 8.0 м
- Диапазон работы: min = 0.1 м max = 100 м

3. **IMU**
```xml
    <!--............................... IMU PLUGIN ..................................... -->

    <gazebo reference="imu_frame">
        <sensor name="imu" type="imu">
            <always_on>true</always_on>
            <update_rate>5</update_rate>
            <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
                <ros>
                    <namespace>$(arg robot_namespace)</namespace>
                    <remapping>~/out:=imu</remapping>
                </ros>
            </plugin>
        </sensor>
    </gazebo>
```
Для IMU указывается:
- Частота = 5 Гц

4. **Стереокамера**
```xml
    <!-- ...........................STEREO CAMERA PLUGIN ................................... -->

    <xacro:if value="$(arg stereo_camera_enabled)">
        <gazebo reference="stereo_camera">
            <sensor type="multicamera" name="stereo_camera">
                <update_rate>10.0</update_rate>
                <always_on>true</always_on>
                <camera name="left">
                    <pose>0 0 0 0 0 0</pose>
                    <horizontal_fov>1.3962634</horizontal_fov>
                    <image>
                        <width>1024</width>
                        <height>1024</height>
                        <format>R8G8B8</format>
                    </image>
                    <clip>
                        <near>0.3</near>
                        <far>20</far>
                    </clip>
                    <noise>
                        <type>gaussian</type>
                        <mean>0.0</mean>
                        <stddev>0.0</stddev>
                    </noise>
                    <distortion>
                        <k1>0.0</k1>
                        <k2>0.0</k2>
                        <k3>0.0</k3>
                        <p1>0.0</p1>
                        <p2>0.0</p2>
                        <center>0.5 0.5</center>
                    </distortion>
                </camera>

                <camera name="right">
                    <pose>0 -0.12 0 0 0 0</pose>
                    <horizontal_fov>1.3962634</horizontal_fov>
                    <image>
                        <width>1024</width>
                        <height>1024</height>
                        <format>R8G8B8</format>
                    </image>
                    <clip>
                        <near>0.3</near>
                        <far>20</far>
                    </clip>
                    <noise>
                        <type>gaussian</type>
                        <mean>0.0</mean>
                        <stddev>0.0</stddev>
                    </noise>
                    <distortion>
                        <k1>0.0</k1>
                        <k2>0.0</k2>0
                        <k3>0.0</k3>
                        <p1>0.0</p1>
                        <p2>0.0</p2>
                        <center>0.5 0.5</center>
                    </distortion>
                </camera>

                <plugin name="stereo_camera_controller" filename="libgazebo_ros_camera.so">
                    <ros>
                        <namespace>$(arg robot_namespace)</namespace>
                        <argument>image_raw:=image_raw</argument>
                        <argument>camera_info:=camera_info</argument>
                    </ros>
                    <camera_name>stereo_camera</camera_name>
                    <frame_name>stereo_camera_optical</frame_name>
                    <hack_baseline>0.12</hack_baseline>
                </plugin>

            </sensor>
        </gazebo>
    </xacro:if>
```
Для стереокамеры указывается:
- Частота обновления изображения6 10 Гц
- Разрешение: 1024x1024
- Дистанция видимости: near = 0.3, far = 20
- Расстояние между камерами: 12 см
- Коэффициенты дисторсии: радиальные коэффициенты: 0,0,0; тангенцальные коэффициенты: 0, 0


### Запуск симуляции
Для запуска симуляции в Gazebo используется launch-файл ```gazebo.launch.py```. Команда запуска выглядит следующим образом:
```bash
ros2 launch bcr_bot gazebo.launch.py
```
![Запуск в Gazebo](images/gazebo_bcr.png)

Файл запуска принимает несколько аргументов:
```bash
ros2 launch bcr_bot gazebo.launch.py \
	camera_enabled:=True \
	two_d_lidar_enabled:=True \
	stereo_camera_enabled:=False \
	position_x:=0.0 \
	position_y:=0.0 \
	orientation_yaw:=0.0 \
	odometry_source:=world \
	world_file:=small_warehouse.sdf \
	robot_namespace:="bcr_bot"
```
Описание параметров:
|Параметр|Принимаемый тип|Описание|
|:-|:-:|:-|
|```camera_enabled```       |```bool```  |Включение камеры глубины         |
|```two_d_lidar_enabled```  |```bool```  |Активация 2D лидара              |
|```stereo_camera_enabled```|```bool```  |Включение стереокамеры           |
|```position_x```           |```float``` |Начальная координата x (в метрах)|
|```position_y```           |```float``` |Начальная координата y (в метрах)|
|```orientation_yaw```      |```float``` |Начальный угол поворота (в рад)  |
|```odometry_source```      |```string```|Данные одометрии из симуляции (можно использовать ```encoder```)|
|```world_file```           |```string```|Файл мира Gazebo (располагаются в ```src/bcr_bot/worlds/```)|
|```robot_namespace```      |```string```|Пространство имен для топиков и tf-фреймов|

Для визуализации в RViz используется launch-файл ```rviz.launch.py```

```bash
ros2 launch bcr_bot rviz.launch.py
```
![Визуализация в RViz](images/rviz_bcr.png)

Также можно посмотреть изображения с камеры глубины, добавив в RViz элементы визуализации ```Image```

![Визуализация в RViz с Image](images/bcr_bot_rviz_with_Image.png)

Данный проект содержит, также, стереокамеру. Ниже приведена команда для запуска 

```bash
ros2 launch stereo_image_proc stereo_image_proc.launch.py left_namespace:=bcr_bot/stereo_camera/left right_namespace:=bcr_bot/stereo_camera/right
```

![Визуализация в RViz с стереокамерой](images/bcrbot_rviz_stereo_enabled.png)

Для просмотра данных с imu сенсора используется команад:
```bash
ros2 topic echo bcr_bot/imu
```

![Данные с IMU](images/bcr_bot_imu.png)

В данном сообщении содержится метка времени, кватернион ориентации, угловая скорость, линейное ускорение и матрицы ковариаций.

### Картографирование с помощью SLAM Toolbox
SLAM Toolbox — это пакет с открытым исходным кодом, предназначенный для картографирования окружающей среды с использованием лазерного сканирования и одометрии, а также для создания карты для автономной навигации.

Чтобы начать картографирование:
```bash
ros2 launch bcr_bot mapping.launch.py
```

Для управления роботом и картографирования местности используется Teleop Twist:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=/bcr_bot/cmd_vel
```

![Терминал teleop_twist_keyboard](images/teleop_key_terminal.png)

Данный узел позволяет управлять роботом с клавиатуры, считывая нажатия клавиш и публикуя их в виде сообщений ```Twist```. Функциональность представлена в таблице ниже.

Описание параметров:
|Назначение|Клавиши|
|:-------------------------------------------------------|:-------------:|
|Движение вперед/назад                                   |```i```/```,```|
|Поворот налево/направо                                  |```j```/```l```|
|Движение по окружности вперед по часовой/против часовой |```o```/```u```|
|Движение по окружности назад по часовой/против часовой  |```m```/```.```|
|Остановка                                               |```k```        |
|Увеличение/уменьшение угловой и линейной скорости на 10%|```q```/```z```|
|Увеличение/уменьшение только линейной скорости на 10%   |```w```/```x```|
|Увеличение/уменьшение только угловой скорости на 10%    |```e```/```c```|

![Картографирование](images/mapping.gif)

Чтобы сохранить карту:

```bash
cd src/bcr_bot/config
ros2 run nav2_map_server map_saver_cli -f bcr_map
```

Пример сохраненной карты показан ниже.

![Сохраненная карта](images/bcr_map.png)

### Использование Nav2 с bcr_bot

Nav2 — это навигационный пакет с открытым исходным кодом, который позволяет роботу легко перемещаться по среде. Он принимает данные лазерного сканирования и одометрии, а также карту среды в качестве входных данных.

Чтобы запустить Nav2 на bcr_bot:
```bash
ros2 launch bcr_bot nav2.launch.py
```

Ве параметры для навигации описаны в файле ```nav2_params.yaml```.

Для локализации робота используется алгоритм AMCL (аддапитвная локализация Монте-Карло). **Локализация Монте-Карло** (MCL) является алгоритмом, чтобы локализовать робота с помощью фильтра частиц. Алгоритм требует известной карты, и задача состоит в том, чтобы оценить положение робота в рамках карты на основе движения и обнаружения робота. Алгоритм запускается с начальной веры вероятностного распределения положения робота, которое представлено частицами, распределенными согласно такой вере. Эти частицы распространены после модели движения робота каждый раз изменения положения робота. После получения новых показаний датчика каждая частица оценит свою точность путем проверки, с какой вероятностью это получило бы такие показания датчика в своем текущем положении. Затем алгоритм перераспределит (передискретизируют) частицы, чтобы сместить частицы, которые более точны. Продолжите выполнять итерации этих перемещение, обнаружение и передискретизация шагов, и все частицы должны сходиться к одному кластеру около истинного положения робота, если локализация успешна [[1]](#список-литературы).

Адаптивная локализация Монте-Карло (AMCL) является вариантом MCL. AMCL динамически настраивает количество частиц на основе KL-расстояния [[2]](#список-литературы), чтобы гарантировать, что распределение частицы сходится к истинному распределению состояния робота на основе всего прошлого датчика и измерений движения с высокой вероятностью.

Для планирования пути используется алгоритм Дейкстры. **Алгоритм Дейкстры** — это классический алгоритм поиска пути, который был разработан голландским ученым Эдсгером Дейкстрой в 1959 году. Этот алгоритм используется для поиска кратчайшего пути в взвешенном графе от одной вершины до всех остальных вершин [[3]](#список-литературы).

![Навигация bcr_bot](images/bcr_bot_nav.gif)

## Заключение
В рамках курсовой работы был изучен и запущен проект навигации [bcr_bot](<https://github.com/blackcoffeerobotics/bcr_bot>), включающий локализацию, планирование пути и обработку данных сенсоров. Настройка Docker обеспечила воспроизводимость и изоляцию проекта.

## Благодарности
Особая благодарность [Black Coffee Robotics](<https://github.com/blackcoffeerobotics>) за разработку репозитория [bcr_bot](<https://github.com/blackcoffeerobotics/bcr_bot>), который послужил основой для данной работы. Код использован в соответствии с [Apache License 2.0](http://www.apache.org/licenses/LICENSE-2.0). Спасибо за ценный вклад в мое обучение!

## Список литературы
[1] Локализация TurtleBot с использованием метода Монте-Карло // Exponenta Docs. — URL: https://docs.exponenta.ru/nav/ug/localize-turtlebot-using-monte-carlo-localization.html (дата обращения: 09.05.2025).

[2] С. Трун, В. Бергард и Д. Фокс, вероятностная робототехника. Кембридж, MA: нажатие MIT, 2005.

[3] ROS 2 для начинающих: от установки до создания робота // Habr. — URL: https://habr.com/ru/companies/otus/articles/748470/ (дата обращения: 09.05.2025).