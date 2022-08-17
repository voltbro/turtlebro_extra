

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

## Подключение к робот

Для работы необходимо чтобы festival сервер быз запущен через systemd

### Включить автозагрузку festival

```
sudo systemctl start festival
sudo systemctl enable festival
```

### Проверить сервер
```
echo '(SayText "Привет мир")' | festival_client
```
