# Пакет патрулирования turtlebro_patrol для робота TurtleBro


## Установка пакета

Пакет входит в сборку метапакета `turtlebro_extra` и устанавливается автоматически при установке этого пакета. В случае, если вы хотите установить пакет патрулирования отдельно, следуйте следующим инструкциям:

```
cd ~/catkin_ws/src
git clone https://github.com/voltbro/turtlebro_patrol
cd ..
catkin_make --pkg turtlebro_patrol
```

## Запуск пакета патрулирования

Перед запуском вы должны очистить данные одометрии из STM32, отправив команду сброса:
```
rosservice call /reset
```
### Запуск нод патрулирования и навигации

```
roslaunch turtlebro_patrol patrol.launch
```

### Запуск ноды патрулирования

```
roslaunch turtlebro_patrol patrol_run.launch
```

### Настройка патрулирования

Координаты точек патрулирования задаются в файле:

```
~/catkin_ws/src/turtlebro_extra/turtlebro_patrol/data/goals.xml
```

_Важное примечание!_ 

когда вы добавляете в goals.xml точки, вида:
```
<goal x='1' y='0' theta='90' name="point_name"/>
```
помните, что **ось x** направлена вперед для робота, а **ось y** - влево для робота. 
"Theta" должна быть установлена в градусах и направлена по правилу правой руки:

![Направление осей робота](https://2.downloader.disk.yandex.ru/preview/c15743b31986a9b90d220a320e1071f9f6d04360e87ee3565abc5e774973aac4/inf/Jde04pTzavkgJ1XLLYpcugzdEN7C0f6ftKDJ61k_XuNAfPPYu_GusphWqHiBvBqZaJoPpx74D6azr-msnTGrTg%3D%3D?uid=51578132&filename=%D0%9D%D0%B0%D0%BF%D1%80%D0%B0%D0%B2%D0%BB%D0%B5%D0%BD%D0%B8%D0%B5%20%D0%BE%D1%81%D0%B5%D0%B9.jpg&disposition=inline&hash=&limit=0&content_type=image%2Fjpeg&owner_uid=51578132&tknv=v2&size=1920x872)


### Patrol control
The control of the patrol bot is performed by sending messages of the std_msgs/String type to the topic /patrol_control

Accepted commands:
1. start - starts the patrol cycle or switch robot to next goal
2. pause - pauses patrolling at any point
3. resume - resume patrolling at any point
4. home - go to home position
5. shutdown - stops patrolling and executing the package


### Callback_service

If you need run any action on patrol point, you must specify ```point_callback_service``` from ```.launch``` file

Patrol node will run Service request with ```PatrolPointCallback``` message type on the reach of the point.



### Run from console with callback_service

run with callback_service 
```
python3 patrol.py _point_callback_service:=my_service_name
```
