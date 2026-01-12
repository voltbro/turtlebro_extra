#include <Wire.h>
#include <Adafruit_AMG88xx.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>

#define buttonPin A12  // Пин кнопки

// Время между считыванием данных с AMG88xx
const int amg88xx_delay = 1000; // мс
const int sub_delay = 1; // мс

unsigned long previousMillis = 0;

Adafruit_AMG88xx amg;  // Объект для работы с датчиком AMG88xx
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];  // Массив для хранения значений пикселей

// Класс для адаптации аппаратного интерфейса к ROS
class NewHardware : public ArduinoHardware {
public:
    NewHardware() : ArduinoHardware(&Serial1, 115200) {}
};

ros::NodeHandle_<NewHardware> nh;  // Объект управления узлом ROS
std_msgs::Float32MultiArray f_array_msg;  // Сообщение для передачи значений пикселей
std_msgs::Bool button_msg;  // Сообщение для передачи состояния кнопки

ros::Publisher amg88xx_pixels_pub("amg88xx_pixels", &f_array_msg);  // Публикатор для значений пикселей
ros::Publisher button_state_pub("limit_switch", &button_msg);  // Публикатор для состояния кнопки

void setup() {
    pinMode(buttonPin, INPUT);  // Установка пина кнопки как вход

    if (!amg.begin()) {
        while (1); // Остановка программы, если инициализация датчика не удалась
    }

    delay(100);

    f_array_msg.data = pixels;
    f_array_msg.data_length = AMG88xx_PIXEL_ARRAY_SIZE;

    nh.initNode();  // Инициализация узла ROS
    nh.advertise(amg88xx_pixels_pub);  // Регистрация публикатора для значений пикселей
    nh.advertise(button_state_pub);  // Регистрация публикатора для состояния кнопки
}

void loop() {
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= amg88xx_delay) {
        previousMillis = currentMillis;
        amg.readPixels(pixels);  // Считывание значений с датчика AMG88xx

        amg88xx_pixels_pub.publish(&f_array_msg);  // Публикация значений пикселей

        button_msg.data = digitalRead(buttonPin) == HIGH;  // Считывание состояния кнопки
        button_state_pub.publish(&button_msg);  // Публикация состояния кнопки
    }

    nh.spinOnce();  // Обработка входящих ROS-сообщений
    delay(sub_delay);  // Задержка между итерациями цикла
}
