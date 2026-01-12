<img src="https://user-images.githubusercontent.com/57194638/201707251-5aa29404-2494-4e16-be4a-0cd821a1c0d9.png" width="800" height="300">

#  Метапакет turtlebro_extra для робота Turtlebro


## Установка пакета

```
cd ~/catkin_ws/src
git clone https://github.com/voltbro/turtlebro_extra
cd ..
catkin_make --pkg=turtlebro_extra
```

## Доступные пакеты для роботов компании "Братья Вольт" в пакете  turtlebro_extra

Список доступных для использования пакетов с небольшим описанием:

### turtlebro_actions

https://github.com/voltbro/turtlebro_extra/tree/master/turtlebro_actions

Представляет собой набор стандартных действий робота (проезд вперед, поворот и т.д.), оформленных в сервисы.

### turtlebro_aruco

https://github.com/voltbro/turtlebro_extra/tree/master/turtlebro_aruco

Пакет, который реализует распознование Aruco-маркеров.

### turtlebro_delivery

https://github.com/voltbro/turtlebro_extra/tree/master/turtlebro_delivery

Пакет предназначен для работы робота TurtleBro с комплектом расширения для сборки полезной нагрузки "Робот-курьер".

### turtlebro_excursions

https://github.com/voltbro/turtlebro_extra/tree/master/turtlebro_excursions

Пакет предназначен для работы робота TurtleBro с комплектом расширения для сборки полезной нагрузки "Робот-экскурсовод".

### turtlebro_overheat_sensor

https://github.com/voltbro/turtlebro_extra/tree/master/turtlebro_overheat_sensor

Пакет предназначен для работы робота TurtleBro с комплектом расширения для сборки полезной нагрузки "Робот-инспектор".

### turtlebro_patrol

https://github.com/voltbro/turtlebro_extra/tree/master/turtlebro_patrol

Пакет, реализующий функцию "патрулирования (перемещение между заданными точками)" на роботах TurtleBro и BRover V.4

### turtlebro_searcher

https://github.com/voltbro/turtlebro_extra/tree/master/turtlebro_searcher

Пакет предназначен для работы робота TurtleBro в ролевой моедли "Поисковик". Этот пакет дублирует функицонал пакета turtlebro_excursions, но вместо озвучивания точек, данные публикуются в топик /aruco_marker

### turtlebro_speech

https://github.com/voltbro/turtlebro_extra/tree/master/turtlebro_speech

Пакет предназначен для осуществления звукового сопровождения различных дополнительных пакетов (turtlebro_delivery, turtlebro_excursions и д.р.).

## Работа пакета *turtlebro_extra* на компьютере

В случае, если необходимо запустить работу одного из пакетов, входящих в метапакет **turtlebro_extra**, на компьютере, то для начала необходимо сконфигурировать собственное рабочее пространство:

```
cd
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```
Далее добавить ссылку на рабочее пространство в файл *bashrc*:
```
echo "source /home/$USER/catkin_ws/devel/setup.bash" >> ~/.bashrc source ~/.bashrc
```
