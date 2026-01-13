import rospy
import actionlib
import subprocess
import math
import cv2
import numpy as np
import sys

from tf.transformations import quaternion_multiply, quaternion_inverse, euler_from_quaternion, quaternion_from_euler

from std_msgs.msg import Int16
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, CompressedImage

from turtlebro_speech.srv import Speech, SpeechResponse, SpeechRequest
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class TurtleBro():
    """
    Простой робот. Умеет ехать вперед - forward()
    назад - backward()
    поворачивать направо и налево - right(), left()
    ехать на определенные координаты (x,y) - goto(x,y)
    получить текущие координаты x,y = tb.coords
    зажигать светодиоды - color("цвет")   "цвет" может быть = "red", "green", "blue", "yellow", "white", "off"
    записать фото - save_photo()
    получить фото с камеры как массив cv2 a = tb.photo
    записывать звук - record()
    измерять дистанцию - lidar_distance()
    вызывать пользовательские функции при нажатии на кнопку - call()
    произносить фразы - say()
    находиться в режиме ожидания - wait()
    Для запуска робота в режиме отладки вызовите программу с ключем -d, --d, -debug, --debug или при инициализации класса TurtleBro поставьте 1 в аргумент TurtleBro(1)
    """

    def __init__(self, debug = False):
        rospy.init_node("tb_py")
        rospy.Subscriber("/odom", Odometry, self.__subscriber_odometry_cb)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom = Odometry()

        self.debug = self._detect_debug() or debug
        if self.debug:
            print("Debug mode")

        self.u = Utility(self.debug)
             
        self.linear_x_val = 0.09
        self.angular_z_val = 0.9
     
        rospy.sleep(0.3)

    def __del__(self):
        self.vel_pub.publish(Twist())
        if self.debug:
            print("Done")

    def _detect_debug(self) -> bool:
    # argv процесса, очищенный от ROS remap-аргументов
        try:
            argv = rospy.myargv(argv=sys.argv)  # сохраняет обычные флаги типа -d
        except Exception:
            argv = sys.argv

        tokens = {"-d", "--debug", "-debug", "d", "debug", "--d"}
        return any(a in tokens for a in argv[1:])

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

    def call(self, name, button = 24):
        self.u.call(name, button)
    
    def wait(self, time):
        self.u.wait(time)

    def color(self, col):
        self.u.color(col)
    
    def save_photo(self, name = "robophoto"):
        self.u.photo(1, name)

    @property
    def coords(self):
        return self.odom.pose.pose.position.x, self.odom.pose.pose.position.y

    @property
    def photo(self):
        return self.u.photo(0, "robophoto")

    def record(self, timeval = 3, filename = "turtlebro_sound"):
        self.u.record(timeval, filename)

    def say(self, text = "Привет"):
        self.u.say(text)
    
    def lidar_distance(self, angle = 0):
        return self.u.get_lidar_distance(angle)

    def speed(self, value):
        assert type(value) == str, "'Скорость' должно быть одним из слов: fastest, fast, normal, slow, slowest"
        Kp = 10
        speed_dict = {"fastest":0.17, "fast":0.12, "normal":0.09, "slow":0.04, "slowest":0.01}
        if type(value) == str:
            self.linear_x_val = speed_dict[value]
        else:
            self.linear_x_val = Utility.clamp(0.01, value, 0.17)
            self.angular_z_val = Kp * self.linear_x_val

    def __subscriber_odometry_cb(self, msg):
        self.odom = msg

    def __move(self, meters):
        if self.debug:
            print("init x: ", self.odom.pose.pose.position.x, "y: ", self.odom.pose.pose.position.y)
            print("meters: ", meters)
        init_position = self.odom
        init_x = 0
        distance_passed = 0
        epsilon = 0.005
        vel = Twist() 
        while not rospy.is_shutdown():
            distance_passed = math.sqrt((self.odom.pose.pose.position.x - init_position.pose.pose.position.x)**2 + (self.odom.pose.pose.position.y - init_position.pose.pose.position.y)**2)
            if (distance_passed + epsilon < abs(meters)):
                if meters > 0:
                    vel.linear.x = self.__vel_x_move_value(self.linear_x_val, init_x, distance_passed, meters)
                else:
                    vel.linear.x = - self.__vel_x_move_value(self.linear_x_val, init_x, distance_passed, abs(meters))
                self.vel_pub.publish(vel)
            else:
                vel.linear.x = 0
                self.vel_pub.publish(vel)
                if self.debug:
                    print("Проехал м.:", distance_passed)
                return
            rospy.sleep(0.05)

    def __goto(self, x, y, theta):
        heading = self.__get_turn_angle_to_point(x, y)
        distance = self.__get_distance_to_point(x, y)
        self.__turn(heading)
        self.__move(distance)

    def __turn(self, degrees):
        """
        Поворот на заданный угол (градусы) по IMU.
        degrees > 0: влево, degrees < 0: вправо.
        """
        angle_goal = math.radians(float(degrees))
        if abs(angle_goal) < 1e-4:
            return

        rate = rospy.Rate(50)

        # ---------------- helpers ----------------
        def valid_yaw(y):
            return (y is not None) and (not math.isnan(y)) and (not math.isinf(y))

        def wrap_pi(a):
            # "железобетонная" регуляризация
            while a > math.pi:
                a -= 2.0 * math.pi
            while a < -math.pi:
                a += 2.0 * math.pi
            return a

        def get_yaw():
            current_q = [self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y,
                self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w]
            (_, _, yaw) = euler_from_quaternion(current_q)
            if not valid_yaw(yaw):
                return None
            # Если понадобится инверсия знака IMU под вашу кинематику — раскомментируй:
            # yaw *= getattr(self, "imu_yaw_sign", 1.0)  # +1 или -1
            return yaw

        # -------- wait IMU stable measurements --------
        stable_needed = 8
        stable = 0
        t0 = rospy.Time.now()
        yaw0 = None

        while not rospy.is_shutdown() and stable < stable_needed:
            y = get_yaw()
            if y is not None:
                yaw0 = y
                stable += 1
            else:
                stable = 0

            if (rospy.Time.now() - t0).to_sec() > 2.0:
                raise RuntimeError("IMU не отвечает/невалидна: нет стабильного yaw")
            rate.sleep()

        # ---------------- controller params ----------------
        # max_w задаётся внешней speed()
        max_w = abs(float(self.angular_z_val))

        # минимальная скорость, чтобы не “залипать” на трении; привяжем к max_w
        min_w = max(0.12, 0.15 * max_w)

        # P-регулятор по ошибке угла (можно подбирать)
        kp = 1.8

        stop_eps = math.radians(0.3)     # точность остановки ~0.3°
        slow_zone = math.radians(20.0)   # зона замедления

        # таймаут по “физике” (чем больше угол — тем больше времени)
        timeout_sec = max(4.0, 1.5 + abs(angle_goal) / max(0.20, 0.35 * max_w))

        # ---------------- unwrap state ----------------
        yaw_prev = yaw0
        yaw_unwrapped = 0.0

        vel = Twist()
        t_start = rospy.Time.now()

        # settle logic: удержаться в точности несколько циклов
        settle_cycles_need = 6
        settle_cycles = 0

        # no-progress protection
        best_abs_err = float("inf")
        no_progress = 0
        no_progress_limit = 25  # ~0.5 сек при 50 Гц

        # санитарный порог на скачок IMU между соседними измерениями
        # (на 50 Гц физически сложно получить огромный dy без глюка/перезапуска фильтра)
        dy_jump_limit = 1.2  # рад ~ 69°

        while not rospy.is_shutdown():
            y = get_yaw()
            if y is None:
                rate.sleep()
                continue

            # unwrap step
            dy = wrap_pi(y - yaw_prev)
            yaw_prev = y

            if abs(dy) > dy_jump_limit:
                # пропускаем подозрительный скачок (не портим unwrap)
                rate.sleep()
                continue

            yaw_unwrapped += dy

            err = angle_goal - yaw_unwrapped
            abs_err = abs(err)

            # settle condition
            if abs_err <= stop_eps:
                settle_cycles += 1
                if settle_cycles >= settle_cycles_need:
                    break
            else:
                settle_cycles = 0

            # progress tracking
            if abs_err < best_abs_err - math.radians(0.2):
                best_abs_err = abs_err
                no_progress = 0
            else:
                no_progress += 1
                if no_progress >= no_progress_limit:
                    break

            # compute angular velocity
            w = kp * err

            # замедление вблизи цели и анти-залипание
            if abs_err < slow_zone:
                w = max(min_w, abs(w)) * (1.0 if w >= 0 else -1.0)

            # clamp по max_w (из speed())
            if w > max_w:
                w = max_w
            elif w < -max_w:
                w = -max_w

            vel.linear.x = 0.0
            vel.angular.z = w
            self.vel_pub.publish(vel)

            # timeout safety
            if (rospy.Time.now() - t_start).to_sec() > timeout_sec:
                break

            rate.sleep()

        if self.debug:
            print(f"Повернул на градус.: {math.degrees(y)}")

        # stop
        vel.linear.x = 0.0
        vel.angular.z = 0.0
        self.vel_pub.publish(vel)
    
    #TODO to add flat range at the begining and the end of traj
    def __vel_x_move_value(self, speed, init_x, curent_x, aim_x):
        fixed_inklin = 0.01 #fixed distance (in m.) there acceleration/decceleration is performing
        if (curent_x == init_x):
            return 0.01
        elif (curent_x < init_x):
            return (init_x - curent_x) * speed
        elif (curent_x > aim_x):
            return 0
        elif ((curent_x - init_x) < fixed_inklin):
            return ((curent_x - init_x) / fixed_inklin) * speed
        elif((aim_x - curent_x) < fixed_inklin):
            return (aim_x - curent_x) * speed
        else:
            return speed
        
    def __vel_z_turn_value(self, speed, init_x, curent_x, aim_x):
        fixed_inklin = 0.08 #fixed angle (in deg.) there acceleration/decceleration is performing
        Kp = 2
        if (curent_x == init_x):
            return 0.01
        elif (curent_x < init_x):
            return (init_x - curent_x) * speed * Kp
        elif (curent_x > aim_x):
            return 0
        elif ((curent_x - init_x) < fixed_inklin):
            return ((curent_x - init_x) / fixed_inklin) * speed * Kp
        elif((aim_x - curent_x) < fixed_inklin):
            return (aim_x - curent_x) * speed * Kp
        else:
            return speed

    def __get_angle_diff(self, prev_orientation, current_orientation):
        prev_q = [prev_orientation.x, prev_orientation.y,
                    prev_orientation.z, prev_orientation.w]
        current_q = [current_orientation.x, current_orientation.y,
                        current_orientation.z, current_orientation.w]

        delta_q = quaternion_multiply(prev_q, quaternion_inverse(current_q))
        (_, _, yaw) = euler_from_quaternion(delta_q)
        return -yaw

    def __get_turn_angle_to_point(self, x, y):
        current_q = [self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y,
                self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w]
        (_, _, yaw) = euler_from_quaternion(current_q)
        heading = math.atan2(y - self.odom.pose.pose.position.y, x - self.odom.pose.pose.position.x)
        angle_to_turn = math.degrees(heading - yaw)
        return angle_to_turn

    def __get_distance_to_point(self, x, y):
        distance = math.hypot((self.odom.pose.pose.position.x - x),(self.odom.pose.pose.position.y - y))
        return distance

class TurtleNav(TurtleBro):
    """
    Робот осуществляющий передвижения при помощи автономной навигации. 
    Умеет:
    ехать на определенные координаты (x,y) и theta(опционально) угол поворота после того, как робот приедет на эти координаты - goto(x,y, theta)
    Кроме того доступны все остальные команды простого робота:
    ехать вперед - forward()
    назад - backward()
    поворачивать направо и налево - right(), left()
    ехать на определенные координаты (x,y) - goto(x,y)
    получить текущие координаты x,y = tb.coords
    зажигать светодиоды - color("цвет")   "цвет" может быть = "red", "green", "blue", "yellow", "white", "off"
    записать фото - save_photo()
    получить фото с камеры как массив cv2 a = tb.photo
    записывать звук - record()
    измерять дистанцию - lidar_distance()
    вызывать пользовательские функции при нажатии на кнопку - call()
    произносить фразы - say()
    находиться в режиме ожидания - wait()
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

    def goto(self, x, y, theta):
        """
        Переопределенная функция базового класса для езды по навигации
        """
        goal = self.__goal_message_assemble(x, y, theta)
        self.movebase_client.wait_for_server()
        self.movebase_client.send_goal_and_wait(goal)


class Utility():

    def __init__(self, debug):
        self.scan = LaserScan()
        self.names_of_func_to_call = {}
        rospy.Subscriber("/scan", LaserScan, self.__subscriber_scan_cb)
        rospy.Subscriber("/buttons", Int16, self.__subscriber_buttons_cb, queue_size=1)
        self.colorpub = rospy.Publisher("/py_leds", Int16, queue_size=10)
        
        self.debug = debug

        rospy.sleep(0.3)

        # odom_reset = rospy.ServiceProxy('reset', Empty)
        # odom_reset.wait_for_service()
        # odom_reset.call()

        self.len_of_scan_ranges = len(self.scan.ranges)
        self.step_of_angles = self.len_of_scan_ranges / 360
        self.retscan = [0] * 360
        self.speech_service = rospy.ServiceProxy('festival_speech', Speech)
    
    def __del__(self):
        self.color("off")

    def __subscriber_scan_cb(self, msg):
        self.scan = msg

    def __subscriber_buttons_cb(self, msg):
        try:
            if(msg.data):
                self.names_of_func_to_call[msg.data]()
                rospy.sleep(0.5) #workaround for non lib functions
        except BaseException:
            pass

    def call(self, name, button = 24):
        self.names_of_func_to_call[button] = name
    
    def wait(self, time = 0):
        if time == 0:
            rospy.spin()
        else:
            rospy.sleep(time)

    def color(self, col):
        assert type(col) == str, "Имя цвета должно быть строкой"
        rgb = {"red":1, "green":2, "blue":3, "yellow":4, "white":5, "off":6}
        shade = Int16()
        shade.data = int(rgb[col])
        self.colorpub.publish(shade)

    def get_lidar_distance(self, angle):
        assert type(angle) == int or type(angle) == float, "Угол должен быть числом"
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
    
    def photo(self, save, name = "robophoto"): 
        assert type(name) == str, "Имя файла фото должно быть строкой"
        image_msg = rospy.wait_for_message("/front_camera/image_raw/compressed", CompressedImage)
        np_arr = np.frombuffer(image_msg.data, np.uint8)
        image_from_ros_camera = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if save:
            cv2.imwrite("/home/pi/"+ name +".jpg", image_from_ros_camera)
            if self.debug:
                print("Фото записано в /home/pi/" + name +".jpg")
        else:
            return image_from_ros_camera

    def record(self, timeval, filename):
        assert timeval > 0 and (type(timeval) == float or type(timeval) == int), "Временной интервал должен быть положительным числом"
        p = subprocess.Popen(["arecord", "-D", "hw:1,0", "-f", "S16_LE", "-r 48000", "/home/pi/" + filename + ".ogg"]) 
        rospy.sleep(timeval)
        p.kill()

    def say(self, text):
        #здесь не assert, а попытка прикастовать аргумент к строке
        if type(text) != str:
            try:
                text = str(text)
            except:
                print("Текст должен быть строкой")
        self.speech_service.wait_for_service()
        self.speech_service.call(SpeechRequest(data = text))

    def clamp(min_val, value, max_val):
        return max(min_val, min(value, max_val))