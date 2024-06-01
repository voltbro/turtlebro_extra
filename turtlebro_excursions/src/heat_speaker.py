#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray, Bool
import numpy as np
import math
from turtlebro_patrol.srv import PatrolControlCallback, PatrolControlCallbackRequest
from turtlebro_speech.srv import Speech, SpeechRequest

class RobotSpeaker:
    def __init__(self):
        # Инициализация текущей максимальной температуры
        self.current_max_temp = 0  

        # Определение топиков и сервисов
        self._sensor_topic = 'amg88xx_pixels'  # Топик датчика тепловизора
        self._button_topic = 'limit_switch'  # Топик кнопки (концевого выключателя)
        self._patrol_service = 'patrol_control'  # Сервис для управления патрулированием
        self._speech_service = 'festival_speech'  # Сервис для озвучивания текста
        
        # Подписка на топик тепловизора
        self._heat_sub = rospy.Subscriber(self._sensor_topic, Float32MultiArray, self._heat_callback)
        
        # Подписка на топик кнопки
        self._button_sub = rospy.Subscriber(self._button_topic, Bool, self._button_callback)
        
        # Ожидание доступности сервиса патрулирования и создание клиента сервиса
        rospy.wait_for_service(self._patrol_service)
        self._patrol_client = rospy.ServiceProxy(self._patrol_service, PatrolControlCallback)
        
        # Ожидание доступности сервиса озвучивания и создание клиента сервиса
        rospy.wait_for_service(self._speech_service)
        self._speech_client = rospy.ServiceProxy(self._speech_service, Speech)

    def _heat_callback(self, heat_msg):
        # Коллбэк-функция для обработки данных от тепловизора
        # Обновление текущей максимальной температуры на основе данных из сообщения
        self.current_max_temp = np.max(heat_msg.data)

    def _button_callback(self, button_msg):
        # Коллбэк-функция для обработки данных от кнопки
        if button_msg.data:  # Проверка, нажата ли кнопка
            rospy.loginfo(f'LimitSwitch: {button_msg.data}')  # Логирование состояния кнопки
            # Разделение температуры на целую и дробную части для озвучивания
            temp_2 = int((self.current_max_temp - math.floor(self.current_max_temp)) * 10.0)
            temp_1 = int(self.current_max_temp - temp_2 / 10.0)
            # Вызов функции озвучивания температуры
            self.say_text(f"Ваша температура: {temp_1} и {temp_2} градусов")
            # Запуск патрулирования
            self._start_patrol()

    def _start_patrol(self):
        # Метод для запуска патрулирования
        try:
            # Создание запроса к сервису патрулирования с командой "start"
            request = PatrolControlCallbackRequest(command="start")
            # Отправка запроса и получение ответа
            response = self._patrol_client(request)
            # Логирование результата работы сервиса патрулирования
            rospy.loginfo(f"Patrol service response: {response.result}")
        except rospy.ServiceException as e:
            # Логирование ошибки в случае неудачного вызова сервиса
            rospy.logerr(f"Service call failed: {e}")

    def say_text(self, text):
        # Метод для озвучивания текста
        rospy.loginfo(f"Start speech: {text}")  # Логирование текста, который будет озвучен
        try:
            # Создание запроса к сервису озвучивания с переданным текстом
            request = SpeechRequest(data=text)
            # Отправка запроса и получение ответа
            response = self._speech_client(request)
            # Проверка результата работы сервиса озвучивания
            if response.success:
                rospy.loginfo("Speech succeeded")  # Логирование успешного озвучивания
            else:
                rospy.logerr("Speech failed")  # Логирование ошибки озвучивания
        except rospy.ServiceException as e:
            # Логирование ошибки в случае неудачного вызова сервиса
            rospy.logerr(f"Service call failed: {e}")
        rospy.loginfo("Speech end")  # Логирование окончания озвучивания

if __name__ == "__main__":
    rospy.init_node("heat_speaker")  # Инициализация ROS-ноды
    r = RobotSpeaker()  # Создание экземпляра класса RobotSpeaker
    rospy.spin()  # Запуск цикла обработки сообщений ROS
