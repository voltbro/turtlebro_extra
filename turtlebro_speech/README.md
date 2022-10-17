
## Инструкция пакета turtlebro_speech

Данный пакет предназначен для осуществления звукового сопровождения различных дополнительных пакетов для робота Turtlebro (turtlebro_delivery, turtlebro_excursions и д.р.). Для работы данного пакета необходимо подключить колонки к аудиопорту RasberryPi робота.

Перед установкой пакета необходимо установить системный пакет `festvox-ru`:

```
sudo apt install festvox-ru
```

#### Установка пакета в ROS 

Для установки данного пакета на робота Turtlebro воспользуйтесь следующей командой:
```
cd ~/catkin_ws/src
git clone https://github.com/voltbro/turtlebro_extra
cd ../
catkin_make --pkg turtlebro_speech
```

### Тестирование установки с помощью text-to-speech

Для того, чтобы проверить работоспособность колонки, а также корректность установки системного пакета `festvox-ru`, можно воспользоваться технологией text-to-speech:
```
echo "Однажды, в студеную зимнюю пору Я из лесу вышел. Был сильный мороз." | festival --tts --language russian
```

__Настройка громкости динамика__
```
alsamixer
```

## Настройка festival для работы пакета

Для правильной работы пакета turtlebro_speech, необходимо чтобы ```festival``` быз запущен как сервис ```systemd```

### Запуск сервиса festival

Проверьте что файл ```festival.service``` находится в директории ```/lib/systemd/system```

В случае, если данного файла там нет, вы можете вручную скопировать его в необходимую директорию воспользовавшись командой:
```
sudo cp /home/pi/catkin_ws/src/turtlebro_extra/turtlebro_speech/services/festival.service /lib/systemd/system/festival.service
```


Включить сервис ```festival```:
```
sudo systemctl start festival
sudo systemctl enable festival
```

### Проверить работу сервиса:
```
echo '(SayText "Привет мир")' | festival_client
```
