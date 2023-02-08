## Инструкция пакета turtlebro_excursion

### Установка пакета ROS

Перед установкой пакета необходимо установить системный пакет `festvox-ru`

```
sudo apt install festvox-ru
```

Для реализации патрулирования необходимо чтобы на роботе был установлен пакет ```turtlebro_patrol```

#### Установить пакет ROS 

```
cd ~/catkin_ws/src
git clone https://github.com/voltbro/turtlebro_excursions
cd ../
catkin_make --pkg turtlebro_excursions
```

### Тестирование text-to-speech

```
echo "Однажды, в студеную зимнюю пору Я из лесу вышел. Был сильный мороз." | festival --tts --language russian
```

__Настройка громкости динамика__
```
alsamixer
```

### Запуск пакета

Запуск ноды патрулирования и режима в которой робот озвучивает по координатам точки патрулирования
```
roslaunch turtlebro_excursions excursion.launch
```
Запуск ноды патрулирования и режима в которой робот считывает и озвучивает aruco маркеры
```
roslaunch turtlebro_excursions excursion_aruco.launch
```
_Важное примечание!_

Для того, чтобы робот начал считывать aruco маркеры, необходимо переключить работу камеры с веб-интерфейса на публикацию данных в ROS. Подробнее об этом в инструкции: https://manual.turtlebro.ru/paket-turtlebro/video

### Работа пакета

Пакет экскурсий реализует запуск ServiceServer, реализующий логику работы "Экскурсовода" в точках патрулирования.

При запуске ноды патрулирования указывается имя сервиса, который вызывается для реализации логики в точках патрулирования.

```xml
  <!--Patrol Node -->
  <node pkg="turtlebro_patrol" type="patrol.py" name="turtlebro_patrol" output="screen" required="true">
    <param name="waypoints_data_file" value="$(arg waypoints_data_file)"/>    
    <param name="point_callback_service" value="turtlebro_excursion"/>    
  </node>
```  

### Работа с Aruco маркерами

Для реализации работы с маркерами в пакете реализован ServiceServer ```aruco_detect``` который по запросу производит поиск и детекции меток. Возвращается номер самой большой найденной метки. 


Используется тип сообщения ```ArucoDetect```

```
---
uint16 id
uint16 size
```

Где 
`id` Номер найденного маркера
`size` Размер маркера (для оценки расстояния)
