import rospy
import actionlib
import subprocess
import math
import cv2
import numpy as np

from tf.transformations import quaternion_multiply, quaternion_inverse, euler_from_quaternion, quaternion_from_euler

from std_msgs.msg import Int16, Float32MultiArray
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, CompressedImage

from turtlebro_speech.srv import Speech, SpeechResponse, SpeechRequest
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

DEBUG = 1  # Включение/отключение отладочной печати

class TurtleBro():
    """
    Класс для базового робота TurtleBro с управлением движением, светодиодами, камерой и звуком
    """

    def __init__(self):
        # Инициализация ROS ноды и подписчиков
        rospy.init_node("tb_py")
        rospy.Subscriber("/odom", Odometry, self.__subscriber_odometry_cb)
        rospy.Subscriber("/thermovisor", Float32MultiArray, self.__subscriber_thermo_cb)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Начальные состояния
        self.odom = Odometry()
        self.thermo = Float32MultiArray()
        self.init_position_on_start = Odometry()
        self.odom_has_started = False

        self.u = Utility()  # Вспомогательные функции

        # Значения скорости
        self.linear_x_val = 0.09
        self.angular_z_val = 0.9

        self.wait_for_odom_to_start()
        self.sum_target_angle = self.__get_curent_angle(self.odom.pose.pose.orientation)

    def __del__(self):
        # Остановка робота при удалении объекта
        self.vel_pub.publish(Twist())
        print("Done")
    
    # Ожидание старта одометрии
    def wait_for_odom_to_start(self):
        while not self.odom_has_started:
            rospy.sleep(0.05)
        self.init_position_on_start = self.odom

    # Основные команды движения
    def forward(self, meters):
        assert meters > 0, "Ошибка! Количество метров должно быть положительным"
        self.__move(meters)

    def backward(self, meters):
        assert meters > 0, "Ошибка! Количество метров должно быть положительным"
        self.__move(-meters)

    def right(self, degrees):
        assert degrees > 0, "Ошибка! Количество градусов должно быть положительным"
        self.__turn(-degrees) 

    def left(self, degrees):
        assert degrees > 0, "Ошибка! Количество градусов должно быть положительным"
        self.__turn(degrees)

    def goto(self, x, y, theta = 0):
        self.__goto(x, y, theta)

    # Взаимодействие с Utility
    def call(self, name, button = 24, *args, **kwargs):
        self.u.call(name, button, *args, **kwargs)
    
    def wait(self, time):
        self.u.wait(time)

    def color(self, col):
        self.u.color(col)
    
    def save_photo(self, name = "robophoto"):
        self.u.photo(1, name)

    # Свойства для получения координат и данных с тепловизора
    @property
    def coords(self):
        angle_q = [self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, 
                   self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w]
        (_, _, theta) = euler_from_quaternion(angle_q)
        x = self.odom.pose.pose.position.x 
        y = self.odom.pose.pose.position.y
        theta = math.degrees(theta)
        return x, y, theta

    @property
    def thermo_pixels(self):
        return self.thermo.data

    # Работа с камерой и звуком
    def get_photo(self):
        return self.u.photo(0, "robophoto")

    def save_photo(self, name = "robophoto"):
        return self.u.photo(1, name)

    def record(self, timeval = 3, filename = "turtlebro_sound"):
        self.u.record(timeval, filename)

    def say(self, text = "Привет"):
        self.u.say(text)

    def play(self, filename):
        self.u.play(filename)
    
    def distance(self, angle = 0):
        return self.u.distance(angle)

    # Настройка скорости движения
    def speed(self, value):
        assert type(value) == str, "'Скорость' должна быть одним из слов: fastest, fast, normal, slow, slowest"
        Kp = 10
        speed_dict = {"fastest":0.17, "fast":0.12, "normal":0.09, "slow":0.04, "slowest":0.01}
        self.linear_x_val = speed_dict[value]
        self.angular_z_val = Kp * self.linear_x_val

    # Callback функции для подписчиков
    def __subscriber_odometry_cb(self, msg):
        self.odom = msg
        if not self.odom_has_started:
            self.odom_has_started = True

    def __subscriber_thermo_cb(self, msg):
        self.thermo = msg

    # Основной метод движения с трапецеидальной скоростью
    def __move(self, meters):
        if DEBUG:
            print("Начало движения на метров:", meters)

        init_x = self.odom.pose.pose.position.x
        init_y = self.odom.pose.pose.position.y
        total_distance = abs(meters)
        direction = 1 if meters > 0 else -1

        vel = Twist()

        while not rospy.is_shutdown():
            dx = self.odom.pose.pose.position.x - init_x
            dy = self.odom.pose.pose.position.y - init_y
            distance_passed = math.sqrt(dx**2 + dy**2)

            if distance_passed >= total_distance:
                vel.linear.x = 0
                self.vel_pub.publish(vel)
                if DEBUG:
                    print("Движение завершено. Проехано:", round(distance_passed, 2))
                return

            speed = self.__move_trapezoidal_trajectory(self.linear_x_val, distance_passed, total_distance)
            vel.linear.x = direction * speed
            self.vel_pub.publish(vel)
            rospy.sleep(0.03)

    # Навигация к точке
    def __goto(self, x, y, theta):
        heading = self.__get_turn_angle_to_point(x, y)
        distance = self.__get_distance_to_point(x, y)
        if DEBUG:
            print("поворот на:", heading)
            print("вперед на:", distance)
        self.__turn(math.degrees(heading))
        self.__move(distance)

    # Метод поворота с трапецеидальной скоростью
    def __turn(self, degrees):
        epsilon = 0.01
        min_speed = 0.05
        total_angle = math.radians(abs(degrees))
        self.sum_target_angle += math.radians(degrees)
        target_angle = self.sum_target_angle % (2 * math.pi)
        turn_dir = 1 if degrees > 0 else -1
        if turn_dir == 0:
            return

        vel = Twist()

        while not rospy.is_shutdown():
            current_angle = self.__get_curent_angle(self.odom.pose.pose.orientation) % (2 * math.pi)
            delta = target_angle - current_angle
            if delta > math.pi:
                delta -= 2 * math.pi
            elif delta < -math.pi:
                delta += 2 * math.pi

            if abs(delta) <= epsilon:
                vel.angular.z = 0
                self.vel_pub.publish(vel)
                if DEBUG:
                    print(f"Поворот завершен. Угол: {math.degrees(current_angle):.2f}°")
                return

            speed = self.__turn_trapezoidal_trajectory(self.angular_z_val, delta, total_angle, min_speed)
            vel.angular.z = turn_dir * speed
            self.vel_pub.publish(vel)
            rospy.sleep(0.05)

    # Вычисление скорости по трапецеидальной траектории для движения
    def __move_trapezoidal_trajectory(self, max_speed, distance_passed, total_distance, min_speed=0.05):
        move_deccel = max(total_distance * 0.5, 0.10)
        distance_remaining = total_distance - distance_passed
        if distance_remaining < move_deccel:
            speed = max(min_speed, max_speed * (distance_remaining / move_deccel))
        else:
            speed = max_speed
        return speed

    # Вычисление скорости по трапецеидальной траектории для поворота
    def __turn_trapezoidal_trajectory(self, max_speed, delta, total_angle, min_speed=0.075):
        turn_deccel = max(total_angle * 0.7, 0.2)
        if abs(delta) < turn_deccel:
            speed = max(min_speed, max_speed * (abs(delta) / turn_deccel))
        else:
            speed = max_speed
        return speed

    # Различные вспомогательные функции для углов и дистанций
    def __get_angle_diff(self, prev_orientation, current_orientation):
        prev_q = [prev_orientation.x, prev_orientation.y, prev_orientation.z, prev_orientation.w]
        current_q = [current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w]
        delta_q = quaternion_multiply(prev_q, quaternion_inverse(current_q))
        (_, _, yaw) = euler_from_quaternion(delta_q)
        return -yaw
    
    def __get_curent_angle(self,current_orientation):
        current_q = [current_orientation.x, current_orientation.y,
                     current_orientation.z, current_orientation.w]
        (_, _, yaw) = euler_from_quaternion(current_q)
        return yaw

    def __get_turn_angle_to_point(self, x, y):
        angle_q = [self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, 
                   self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w]
        (_, _, yaw) = euler_from_quaternion(angle_q)
        heading = -math.atan2(y,x)
        angle_to_turn = yaw - heading
        return angle_to_turn

    def __get_distance_to_point(self, x, y):
        return math.sqrt((self.odom.pose.pose.position.x - x)**2 + (self.odom.pose.pose.position.y - y)**2)

    # Прямое управление скоростью
    def linear_speed(self, v):
        vel = Twist()
        vel.linear.x = v
        self.vel_pub.publish(vel)
        if DEBUG:
            print("Еду с линейной скоростью:", v, "м/с")

    def angular_speed(self, w):
        vel = Twist()
        w = math.radians(w)
        vel.angular.z = w
        self.vel_pub.publish(vel)
        if DEBUG:
            print("Поворачиваю с угловой скоростью:", w, "радиан/с")


# Класс для автономной навигации
class TurtleNav(TurtleBro):
    """
    Робот с поддержкой move_base для навигации к координатам
    """

    def __init__(self):
        super().__init__()
        self.movebase_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    def __goal_message_assemble(self, x ,y, theta):
        # Формируем сообщение цели для move_base
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = float(x)
        goal.target_pose.pose.position.y = float(y)

        q = quaternion_from_euler(0, 0, math.radians(float(theta)))
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        return goal

    def __goto(self, x, y, theta):
        # Переопределяем goto для move_base
        goal = self.__goal_message_assemble(x, y, theta)
        self.movebase_client.wait_for_server()
        self.movebase_client.send_goal_and_wait(goal)


# Вспомогательный класс с сенсорами, светодиодами, камерой и звуком
class Utility():

    def __init__(self):
        self.scan = LaserScan()
        self.names_of_func_to_call = {}
        self.args_of_func_to_call = {}
        self.kwargs_of_func_to_call = {}
        rospy.Subscriber("/scan", LaserScan, self.__subscriber_scan_cb)
        rospy.Subscriber("/buttons", Int16, self.__subscriber_buttons_cb, queue_size=1)
        self.colorpub = rospy.Publisher("/py_leds", Int16, queue_size=10)
        self.speech_service = rospy.ServiceProxy('festival_speech', Speech)

        # Ждем появления данных сканера
        time_counter = 0
        time_to_wait = 3
        while(len(self.scan.ranges)<=1):
            dt = 0.1
            rospy.sleep(dt)
            time_counter += dt
            if time_counter > time_to_wait:
                break

        self.len_of_scan_ranges = len(self.scan.ranges)
        self.step_of_angles = self.len_of_scan_ranges / 360
        self.retscan = [0] * 360
        print("Поехали!!!")

    def __del__(self):
        self.color("blue")

    # Callback функции
    def __subscriber_scan_cb(self, msg):
        self.scan = msg

    def __subscriber_buttons_cb(self, msg):
        try:
            if(msg.data):
                self.names_of_func_to_call[msg.data](*self.args_of_func_to_call[msg.data], **self.kwargs_of_func_to_call[msg.data])
                rospy.sleep(0.5)
        except BaseException:
            pass

    # Регистрация функций на кнопки
    def call(self, name, button = 24, *args, **kwargs):
        self.names_of_func_to_call[button] = name
        self.args_of_func_to_call[button] = args
        self.kwargs_of_func_to_call[button] = kwargs
    
    def wait(self, time = 0):
        if time == 0:
            rospy.spin()
        else:
            rospy.sleep(time)

    # Управление светодиодами
    def color(self, col):
        assert type(col) == str, "Имя цвета должно быть: red, green, blue, yellow, white или off"
        rgb = {"red":1, "green":2, "blue":3, "yellow":4, "white":5, "off":6}
        shade = Int16()
        shade.data = int(rgb[col])
        self.colorpub.publish(shade)

    # Получение расстояния по лазеру
    def distance(self, angle):
        assert type(angle) == int or type(angle) == float, "Угол должен быть числом от 0 до 359"
        if (angle == 0):
            if self.scan.ranges[0] != float("inf"):
                return self.scan.ranges[0]
            else:
                for i in range(-3,3):
                    if self.scan.ranges[i] != float("inf"):
                        return self.scan.ranges[i]
            return 0
        elif angle < 360:
            return self.scan.ranges[int(angle * self.step_of_angles)]
        elif angle == 360:
            for i in range(self.len_of_scan_ranges):
                k = int(i / self.step_of_angles)
                self.retscan[k] = self.scan.ranges[i]
            return self.retscan
        else:
            return None
    
    # Работа с камерой
    def photo(self, save, name): 
        assert type(name) == str, "Имя файла фото должно быть строкой"
        try:
            image_msg = rospy.wait_for_message("/front_camera/image_raw/compressed", CompressedImage, timeout=3)
            np_arr = np.frombuffer(image_msg.data, np.uint8)
            image_from_ros_camera = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if save:
                cv2.imwrite("/home/pi/"+ name +".jpg", image_from_ros_camera)
                if DEBUG:
                    print("Фото записано в /home/pi/" + name +".jpg")
            else:
                return image_from_ros_camera
        except Exception as e:
            print(e)

    # Запись звука
    def record(self, timeval, filename, format = ".wav"):
        assert timeval > 0 and (type(timeval) == float or type(timeval) == int), "Временной интервал должен быть положительным числом"
        p = subprocess.Popen(["arecord", "-D", "hw:1,0", "-f", "S16_LE", "-r 48000", "/home/pi/" + filename + format]) 
        rospy.sleep(timeval)
        p.kill()

    # Произнесение текста
    def say(self, text):
        if type(text) != str:
            try:
                text = str(text)
            except:
                print("Текст должен быть строкой")
        self.speech_service.wait_for_service()
        self.speech_service.call(SpeechRequest(data = text))
    
    # Воспроизведение файла
    def play(self, filename):
        assert filename, "Файл для воспроизведения не задан"
        p = subprocess.Popen(["aplay", "/home/pi/" + filename]) 

    @staticmethod
    def __clamp(min_val, value, max_val):
        # Ограничение значения между min_val и max_val
        return max(min_val, min(value, max_val))
