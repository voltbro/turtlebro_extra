
# Пакет озвучивания turtlebro_speech для робота TurtleBro

Данный пакет предназначен для осуществления звукового сопровождения различных дополнительных пакетов для робота Turtlebro (**turtlebro_delivery**, **turtlebro_excursions** и д.р.). Для работы данного пакета необходимо подключить колонки к аудиопорту RasberryPi робота.

## Установка пакета

Пакет входит в сборку метапакета `turtlebro_extra` и устанавливается автоматически при установке этого пакета. В случае, если вы хотите установить пакет turtlebro_speech отдельно, на роботе выполните команды:

```
cd ~/catkin_ws/src
git clone https://github.com/voltbro/turtlebro_extra
cd ../
catkin_make --pkg turtlebro_speech
```

### Установка дополнительного пакета

Дополнительно для реализации функции оффлайн озвучивания необходимо установить системный пакет `festvox-ru`. Начиная с версии [образа ОС 0.22](https://manual.turtlebro.ru/administrirovanie-ros/raspberrypi#skachat-obraz) этот пакет установлен по умолчанию, но если вы работаете с "чистого" образа, то для установки пакета `festvox-ru` воспользуйтесь командами:

```
sudo apt install festvox-ru
```


## Тестирование установки с помощью text-to-speech

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

Проверьте, что файл ```festival.service``` находится в директории ```/lib/systemd/system```

В случае, если данного файла там нет, вы можете вручную скопировать его в необходимую директорию воспользовавшись командой:
```
sudo cp /home/pi/catkin_ws/src/turtlebro_extra/turtlebro_speech/services/festival.service /lib/systemd/system/festival.service
```


__Включение сервиса ```festival```__
```
sudo systemctl start festival
sudo systemctl enable festival
```

__Проверка работы сервиса__
```
echo '(SayText "Привет мир")' | festival_client
```
