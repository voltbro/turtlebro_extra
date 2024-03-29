# Пакет робота курьера turtlebro_delivery для робота TurtleBro 

Пакет предназначен для работы робота Turtlebro с комплектом расширения для сборки полезной нагрузки "Робот-курьер".
[Инструкция по сборке полезной нагрузки](http://docs.voltbro.ru/Set-Profi2023/DeliveryTB-manual.pdf)

## Установка пакета

Пакет входит в сборку метапакета `turtlebro_extra` и устанавливается автоматически при установке этого пакета. Для установки метапакета на роботе выполните команды:

```
cd ~/catkin_ws/src
git clone https://github.com/voltbro/turtlebro_extra
cd ../
catkin_make --pkg=turtlebro_extra
```

### Установка зависимостей

Для работы пакета необходима установка `python` зависимостей. Начиная с версии [образа ОС 0.22](https://manual.turtlebro.ru/administrirovanie-ros/raspberrypi#skachat-obraz) эти зависимости установлены по умолчанию, но если вы работаете с "чистого" образа, то для установки зависимостей воспользуйтесь командами: 

```
cd ~/catkin_ws/src
pip3 install -r turtlebro_extra/turtlebro_delivery/requirements.txt
```

## Подключение полезной нагрузки

Для управления полезной нагрузкой робота-курьера, необходимо загрузить на робота Ардуино скетч, который находится в папке: `arduino/delivery/delivery.ino`

Сервопривод подключить в разъем `D46`

Концевой выключатель в разъем `A12`

Кнопка управления крышкой в `A15`

## Алгоритм работы робота-курьера

Пакет доставки работает по следующему алгоритму:

1. Прежде чем положить товар в грузовой отсек, необходимо отсканировать товар (aruco-маркер) камерой робота. Это определит в какую точку необходимо доставить товар

2. Поместить товар в грузовой отсек робота и нажать на кнопку для закрытия крышки

3. Робот отправится в заданные координаты

4. В точке завершения маршрута необходимо в камеру продемонстрировать aruco-маркер клиента для открытия крышки грузового отсека

5. Если aruco-маркер клиента соответствует заказанному товару, робот откроет грузовой отсек

6. Необходимо забрать товар и нажать кнопку для завершения доставки

7. Робот вернется в место загрузки и будет ждать следующего заказа

Конфигурирование параметров робота-курьера происходит в файле `data/delivery.toml`

### Описание файла конфигурации доставки

Файл конфигурации располагается по следующему пути:

```
/home/pi/catkin_ws/src/turtlebro_extra/turtlebro_delivery/data/delivery.toml
```
В данном файле вы можете задать координаты "склада загрузки товаров":

```
[home]
pose = {x = 0,y = 0, theta = 0 }
```

Также можете добавлять "клиентов". В форму "клиента" входят следующие параметры:

```
1. pose = {x,y,theta} - задаются координаты "клиента"
2. products = [aruco_id] - задаются aruco_id товара/товаров для данного "клиента". Может быть несколько товаров для одного клиента.
3. secret = [aruco_id] - задается aruco_id "клиента", по которому будет выдан заказ. Может совпадать с aruco_id товара.
```

## Запуск доставки

__Запуск функционирования в режиме курьера__ 

```
roslaunch turtlebro_delivery delivery.launch
```

Для отладки кода есть возможность эмулировать работу без подключения навигации и реального перемещения робота. Для этого необходимо установить агрумент  `fake_move_base`:

```
roslaunch turtlebro_delivery delivery.launch fake_move_base:=true
```

### Подключение работы колонки для озвучки статусов доставки

Для подключения колонки к роботу, необходимо убедиться что правильно установлен пакет `turtlebro_speech` (https://github.com/voltbro/turtlebro_extra/tree/master/turtlebro_speech) из метапакета `turtlebro_extra`

__Запуск доставки с озвучкой статусов__

```
roslaunch turtlebro_delivery delivery_speech.launch
```

__Запуск доставки с озвучкой статусов в режиме эмуляции перемещения__

```
roslaunch turtlebro_delivery delivery_speech.launch fake_move_base:=true
```

Конфигурация фраз находится в файле `data/speech.toml`

