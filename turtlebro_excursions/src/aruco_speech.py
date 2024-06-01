#!/usr/bin/env python3

from turtlebro_patrol.srv import PatrolPointCallback, PatrolPointCallbackRequest, PatrolPointCallbackResponse
from turtlebro_speech.srv import Speech, SpeechResponse, SpeechRequest
from turtlebro_aruco.srv import ArucoDetect, ArucoDetectResponse, ArucoDetectRequest

import rospy
import toml
from pathlib import Path

# Инициализация ROS-ноды
rospy.init_node('excursion_aruco_service')

# Создание клиента для сервиса озвучивания и ожидание его доступности
speech_service = rospy.ServiceProxy('festival_speech', Speech)
rospy.loginfo("Waiting for festival_speech service")
speech_service.wait_for_service()
rospy.loginfo("Have festival_speech service")

# Создание клиента для сервиса распознавания Aruco и ожидание его доступности
aruco_service = rospy.ServiceProxy('aruco_detect', ArucoDetect)
rospy.loginfo("Waiting for aruco service")
aruco_service.wait_for_service()
rospy.loginfo("Have aruco service")

# Загрузка файла с данными Aruco из параметра или по умолчанию
aruco_data_file = rospy.get_param('~aruco_data_file', str(Path(__file__).parent.absolute()) + '/../data/aruco_text.toml')
aruco_data = toml.load(aruco_data_file)

rospy.loginfo(f"Loading aruco file {aruco_data_file}")

def handle_request(req: PatrolPointCallbackRequest):
    # Обработчик запроса на обслуживание точки патрулирования
    
    text = ""  # Инициализация текста для озвучивания
    
    # Вызов сервиса распознавания Aruco и получение результата
    aruco_result: ArucoDetectResponse = aruco_service.call(ArucoDetectRequest())

    # Проверка распознанного ID Aruco
    if aruco_result.id > 0:  # Если ID больше 0
        if str(aruco_result.id) in aruco_data:  # Если ID найден в данных Aruco
            text += " " + aruco_data[str(aruco_result.id)]['text']  # Добавление текста для этого ID
        else:
            text += " " + aruco_data['unknown']['text']  # Текст для неизвестного ID
    else:
        text += " " + aruco_data['empty']['text']  # Текст для случая, когда ID не найден

    # Вызов сервиса озвучивания с подготовленным текстом
    result: SpeechResponse = speech_service.call(SpeechRequest(text))

    # Возврат ответа для сервиса патрулирования
    return PatrolPointCallbackResponse(1, "Speech end")

# Объявление сервиса point_aruco и указание обработчика запросов
s = rospy.Service('point_aruco', PatrolPointCallback, handle_request)
rospy.loginfo("point_aruco Service Ready")

# Ожидание запросов на сервис
rospy.spin()