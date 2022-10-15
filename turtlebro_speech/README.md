

## Говорилка через festival

Перед установкой пакета необходимо установить системный пакет `festvox-ru`

```
sudo apt install festvox-ru
```

#### Установить пакет ROS 

```
cd ~/catkin_ws/src
git clone https://github.com/voltbro/turtlebro_extra
cd ../
catkin_make --pkg turtlebro_speech
```

### Тестирование установки text-to-speech

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

Или скопируйте его туда
```
sudo cp services/festival.service /lib/systemd/system/festival.service
```


Включть сервис ```festival```
```
sudo systemctl start festival
sudo systemctl enable festival
```

### Проверить работу сервиса
```
echo '(SayText "Привет мир")' | festival_client
```
