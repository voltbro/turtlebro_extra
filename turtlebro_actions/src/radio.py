#! /usr/bin/env python3
import serial, signal, struct, rospy, time
from commands_controller import CommandsController

rospy.init_node('radio_command_node')

ser = serial.Serial()
ser.baudrate = rospy.get_param('~baud', 19200)
ser.port = rospy.get_param('~port', "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0")
ser.dtr = False
ser.timeout = 0.5
ser.open()

term = b'\x00\xFF\xFF'
lenterm = len(term)
global_loop = True

# unsigned char + short int
# struct_format = "!Bh"
# unsigned char  + short_uint
struct_format = "!BH"


def stop_global_loop(signal, frame):
    global global_loop
    print("Ctrl+C captured, ending read.")
    global_loop = False


signal.signal(signal.SIGINT, stop_global_loop)

linear_speed = rospy.get_param('~linear_speed', 0.22)
angular_speed = rospy.get_param('~angular_speed', 1)

controller = CommandsController(linear_speed, angular_speed)

while global_loop:

    pack = ser.read_until(term)    
    if(len(pack) > 2 and pack[-lenterm:] == term):

        pack = pack[:-lenterm]
        commands_count  = int(len(pack)/struct.calcsize(struct_format))

        for i in range(0, commands_count):
            try:
                (command, value) = struct.unpack(struct_format, pack[i*3:(i+1)*3])
                ret_data = controller.execute(command, value)

                # image echo    
                pack_size = 1024           
                if(command == 55 or command == 56):
                    pack_num  = int(len(ret_data)/pack_size)+1
                    for i in range(0, pack_num):
                        serial_pack = ret_data[i*pack_size:(i+1)*pack_size]
                        send_bytes = ser.write(serial_pack)
                        rospy.loginfo("Send data pack%i: %s", i, send_bytes)
                        time.sleep(0.5)

            except:
                pass



