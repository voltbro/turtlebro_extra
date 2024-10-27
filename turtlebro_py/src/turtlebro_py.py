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


DEBUG = 1

class TurtleBro():
    """
    Простой робот. Умеет:
    ехать вперед, назад на заданное расстояние в метрах - forward(расстояние), backward(расстояние)
    поворачивать направо и налево, на заданный угол в градусах - right(угол), left(угол)
    ехать по прямой с заданной скоростью в м/с - linear_speed(скорость) #положительная вперед, отрицательная назад
    поворачивать с заданной скоростью в градусах/сек - angular_speed(скорость) #положительная против часовой стрелки, отрицательная по часовой стрелке - правило правой руки
    задавать скорость езды по прямой и поворота - speed("скорость") # в функцию должно передаваться одно из слов "fastest", "fast", "normal", "slow", "slowest"
    полчуать текущие координаты tb.coords
    ехать на определенные координаты (x,y) - goto(x,y)
    получить текущие координаты и угол поворота относительно старта x,y,theta = tb.coords
    зажигать светодиоды - color("цвет")   "цвет" может быть = "red", "green", "blue", "yellow", "white", "off"
    записать фото - save_photo()
    получить фото с камеры как массив cv2 a = tb.photo
    записывать звук - record()
    измерять дистанцию - distance()
    вызывать пользовательские функции при нажатии на кнопку - call(имя_пользовательской_функции, аргументы_пользовательской_функции - необязательно)
    произносить фразы - say()
    проигрывать звуковые файлы - play()
    находиться в режиме ожидания - wait()
    получать данные с тепловизора tb.thermo_pixels #массив 64-х значений температуры полученных с тепловизора
    """

    def __init__(self):
        rospy.init_node("tb_py")
        rospy.Subscriber("/odom", Odometry, self.__subscriber_odometry_cb)
        rospy.Subscriber("/thermovisor", Float32MultiArray, self.__subscriber_thermo_cb)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom = Odometry()
        self.thermo = Float32MultiArray()
        self.init_position_on_start = Odometry()
        self.odom_has_started = False

        self.u = Utility()
             
        self.linear_x_val = 0.09
        self.angular_z_val = 0.9

        self.wait_for_odom_to_start()

        self.sum_target_angle = self.__get_curent_angle(self.odom.pose.pose.orientation)

    def __del__(self):
        self.vel_pub.publish(Twist())
        print("Done")
    
    def wait_for_odom_to_start(self):
        while not self.odom_has_started:
            rospy.sleep(0.05)
        self.init_position_on_start = self.odom

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

    def call(self, name, button = 24, *args, **kwargs):
        self.u.call(name, button, *args, **kwargs)
    
    def wait(self, time):
        self.u.wait(time)

    def color(self, col):
        self.u.color(col)
    
    def save_photo(self, name = "robophoto"):
        self.u.photo(1, name)

    @property
    def coords(self):
        angle_q = [self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w]
        (_, _, theta) = euler_from_quaternion(angle_q)
        x = self.odom.pose.pose.position.x 
        y = self.odom.pose.pose.position.y
        theta = math.degrees(theta)
        return x, y, theta

    @property
    def thermo_pixels(self):
        thermo_arr = self.thermo.data
        return thermo_arr

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

    def speed(self, value):
        assert type(value) == str, "'Скорость' должно быть одним из слов: fastest, fast, normal, slow, slowest"
        Kp = 10
        speed_dict = {"fastest":0.17, "fast":0.12, "normal":0.09, "slow":0.04, "slowest":0.01}
        if type(value) == str:
            self.linear_x_val = speed_dict[value]
        else:
            self.linear_x_val = Utility.__clamp(0.01, value, 0.17)
            self.angular_z_val = Kp * self.linear_x_val

    def __subscriber_odometry_cb(self, msg):
        self.odom = msg
        if not self.odom_has_started:
            self.odom_has_started = True

    def __subscriber_thermo_cb(self, msg):
        self.thermo = msg

    def __move(self, meters):
        move_decel = 0.01 #distance delta to apply P regulations for speed
        if DEBUG:
            print("init x: ", round(self.odom.pose.pose.position.x, 2), "y: ", round(self.odom.pose.pose.position.y,2))
            print("meters: ", round(meters,2))
        init_position = self.odom
        init_x = 0
        distance_passed = 0
        vel = Twist() 
        while not rospy.is_shutdown():
            distance_passed = math.sqrt((self.odom.pose.pose.position.x - init_position.pose.pose.position.x)**2 + (self.odom.pose.pose.position.y - init_position.pose.pose.position.y)**2)
            if (distance_passed < abs(meters)):
                if meters > 0:
                    vel.linear.x = self.__move_trapezoidal_trajectory(self.linear_x_val, init_x, distance_passed, meters, move_decel)
                else:
                    vel.linear.x = - self.__move_trapezoidal_trajectory(self.linear_x_val, abs(meters), distance_passed, init_x, move_decel)
                self.vel_pub.publish(vel)
            else:
                vel.linear.x = 0
                self.vel_pub.publish(vel)
                if DEBUG:
                    print("Проехал м.:", round(distance_passed, 2))
                return
            rospy.sleep(0.03)

    def __goto(self, x, y, theta):
        heading = self.__get_turn_angle_to_point(x, y)
        distance = self.__get_distance_to_point(x, y)
        if DEBUG:
            print("поворот на:", heading)
            print("вперед на:", distance)
        self.__turn(math.degrees(heading))
        self.__move(distance)

    def __turn(self, degrees):
        turn_deccel = 0.2 #delta of angles to apply P regulations for speed
        epsilon = 0.01 #accuracy in radians
        init_angle = self.__get_curent_angle(self.odom.pose.pose.orientation)

        self.sum_target_angle += math.radians(degrees)
        curent_target_angle = self.sum_target_angle % (math.pi * 2)

        readed_angle = self.__get_curent_angle(self.odom.pose.pose.orientation)
        curent_angle = readed_angle % (math.pi*2)

        turn_dir = 0
        if (degrees > 0):
            turn_dir = 1
        elif(degrees < 0):
            turn_dir = -1
        else:
            return

        vel = Twist()

        while not rospy.is_shutdown():
            readed_angle = self.__get_curent_angle(self.odom.pose.pose.orientation)
            curent_angle = readed_angle % (math.pi*2)
            delta = (curent_target_angle - curent_angle)

            if DEBUG:
                print("градус на который надо повернуть: ",  curent_target_angle)
                print(readed_angle, "прочитанный угол")
                print(init_angle, "начальный угол")
                print(curent_angle, "текущий угол") 
                print(delta, "дельта")
            
            if (turn_dir > 0 and abs(delta) > epsilon):
                vel.angular.z = self.__turn_trapezoidal_trajectory(self.angular_z_val, delta, turn_deccel)
            elif(turn_dir < 0 and abs(delta) > epsilon):
                vel.angular.z = -self.__turn_trapezoidal_trajectory(self.angular_z_val, delta, turn_deccel)
            else:
                vel.angular.z = 0
                self.vel_pub.publish(vel)
                if DEBUG:
                    print("Повернул град.:", math.degrees(curent_angle))
                return
            self.vel_pub.publish(vel)
            rospy.sleep(0.05)     

    def __move_trapezoidal_trajectory(self, speed, init_x, curent_x, aim_x, zero_deccel):
        if abs(curent_x - aim_x) < zero_deccel:
            return abs(curent_x - aim_x) * speed
        elif abs(curent_x - init_x) < zero_deccel:
            return (0.5 * speed)
        else:
            return speed

    def __turn_trapezoidal_trajectory(self, speed, delta, zero_deccel):
        if abs(delta) < zero_deccel:
            return abs(delta) * speed
        return speed

    def __get_angle_diff(self, prev_orientation, current_orientation):
        prev_q = [prev_orientation.x, prev_orientation.y,
                    prev_orientation.z, prev_orientation.w]
        current_q = [current_orientation.x, current_orientation.y,
                        current_orientation.z, current_orientation.w]

        delta_q = quaternion_multiply(prev_q, quaternion_inverse(current_q))
        (_, _, yaw) = euler_from_quaternion(delta_q)
        return -yaw
    
    def __get_curent_angle(self,current_orientation):
        current_q = [current_orientation.x, current_orientation.y,
                current_orientation.z, current_orientation.w]
        (_, _, yaw) = euler_from_quaternion(current_q)
        return yaw

    def __get_turn_angle_to_point(self, x, y):
        angle_q = [self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w]
        (_, _, yaw) = euler_from_quaternion(angle_q)
        heading = -math.atan2(y,x)
        angle_to_turn = yaw - heading
        return angle_to_turn

    def __get_distance_to_point(self, x, y):
        return math.sqrt((self.odom.pose.pose.position.x - x)**2 + (self.odom.pose.pose.position.y - y)**2)

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

         

class TurtleNav(TurtleBro):
    """
    Робот осуществляющий передвижения при помощи автономной навигации. 
    Умеет:
    ехать на определенные координаты (x,y) и theta(опционально) угол поворота после того, как робот приедет на эти координаты - goto(x,y, theta)
    Кроме того доступны все остальные команды простого робота:
    """

    def __init__(self):
        super().__init__()
        self.movebase_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    def __goal_message_assemble(self, x ,y, theta):
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
        """
        Переопределенная функция базового класса для езды по навигации
        """
        goal = self.__goal_message_assemble(x, y, theta)
        self.movebase_client.wait_for_server()
        self.movebase_client.send_goal_and_wait(goal)


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

        time_counter = 0
        time_to_wait = 3 #seconds to wait for scan to start
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

    def __subscriber_scan_cb(self, msg):
        self.scan = msg

    def __subscriber_buttons_cb(self, msg):
        try:
            if(msg.data):
                self.names_of_func_to_call[msg.data](*self.args_of_func_to_call[msg.data], **self.kwargs_of_func_to_call[msg.data])
                rospy.sleep(0.5) #workaround for non lib functions
        except BaseException:
            pass

    def call(self, name, button = 24, *args, **kwargs):
        self.names_of_func_to_call[button] = name
        self.args_of_func_to_call[button] = args
        self.kwargs_of_func_to_call[button] = kwargs
    
    def wait(self, time = 0):
        if time == 0:
            rospy.spin()
        else:
            rospy.sleep(time)

    def color(self, col):
        assert type(col) == str, "Имя цвета должно быть: red, green, blue, yellow, white или off"
        rgb = {"red":1, "green":2, "blue":3, "yellow":4, "white":5, "off":6}
        shade = Int16()
        shade.data = int(rgb[col])
        self.colorpub.publish(shade)

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

    def record(self, timeval, filename, format = ".wav"):
        assert timeval > 0 and (type(timeval) == float or type(timeval) == int), "Временной интервал должен быть положительным числом"
        p = subprocess.Popen(["arecord", "-D", "hw:1,0", "-f", "S16_LE", "-r 48000", "/home/pi/" + filename + format]) 
        rospy.sleep(timeval)
        p.kill()

    def say(self, text):
        if type(text) != str:
            try:
                text = str(text)
            except:
                print("Текст должен быть строкой")
        self.speech_service.wait_for_service()
        self.speech_service.call(SpeechRequest(data = text))
    
    def play(self, filename):
        assert filename, "Файл для воспроизведения не задан"
        p = subprocess.Popen(["aplay", "/home/pi/" + filename]) 

    def __clamp(min_val, value, max_val):
        return max(min_val, min(value, max_val))