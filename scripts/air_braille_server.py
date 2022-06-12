#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from std_msgs.msg import UInt8
import os
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Pose, PoseArray, TransformStamped, PoseStamped
from align_plane_to_points.srv import *

import pyttsx3
import threading
import queue

# To play audio text-to-speech during execution
class TTSThread(threading.Thread):
    def __init__(self, queue):
        threading.Thread.__init__(self)
        self.queue = queue
        self.daemon = True
        self.start()

    def run(self):
        tts_engine = pyttsx3.init()
        tts_engine.startLoop(False)
        t_running = True
        while t_running:
            if self.queue.empty():
                tts_engine.iterate()
            else:
                data = self.queue.get()
                if data == "exit":
                    t_running = False
                else:
                    tts_engine.say(data)
        tts_engine.endLoop()

class AirBrailleServer:
    def __init__(self):

        self.file_location = str(rospy.get_param('~file_location'))
        self.file_type = str(rospy.get_param('~file_type'))

        self.width = rospy.get_param('~width')
        self.height = rospy.get_param('~height')

        self.pin_1_offset = rospy.get_param('~pin_1_offset')
        self.pin_2_offset = rospy.get_param('~pin_2_offset')
        self.pin_3_offset = rospy.get_param('~pin_3_offset')
        self.pin_4_offset = rospy.get_param('~pin_4_offset')
        self.pin_5_offset = rospy.get_param('~pin_5_offset')
        self.pin_6_offset = rospy.get_param('~pin_6_offset')

        self.world_frame =str(rospy.get_param('~world_frame'))
        self.device_frame = str(rospy.get_param('~device_frame'))

        self.pixels_per_char = rospy.get_param('~pixels_per_char')
        self.horizontal_spacer_pixels = rospy.get_param('~horizontal_spacer_pixels')
        self.vertical_spacer_pixels = rospy.get_param('~vertical_spacer_pixels')

        self.sound_enabled = rospy.get_param('~sound_enabled')

        self.mode = rospy.get_param('~mode') #single_char, text_page, image

        self.total_height_in_pixels = self.pixels_per_char*self.height+self.vertical_spacer_pixels*(self.height-1)
        self.total_width_in_pixels = self.pixels_per_char*self.width+self.horizontal_spacer_pixels*(self.width-1)

        self.img = np.zeros((self.total_height_in_pixels,self.total_width_in_pixels,3), dtype=np.uint8)
        self.img.fill(255)

        self.serial_data_out_topic = str(rospy.get_param('~serial_data_out_topic'))
        self.serial_data_in_topic = str(rospy.get_param('~serial_data_in_topic'))

        self.ascii_page = np.full((self.height,self.width),32,np.ubyte) #fills with space

        if(self.file_type == "txt"):
            ## initialise with text
            self.load_text_file()
            self.draw_text_page()
            # cv2.imshow('image',self.img)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()

        elif (self.file_type == "img"):
            #initialise with image
            rospy.logerr("images not implemented yet")
        else:
            rospy.logerr("unknown file_type: ",self.file_type)

        self.braille_dict = { " ": 0b00000000,
                        "a": 0b00000001,
                        "b": 0b00000011,
                        "c": 0b00001001,
                        "d": 0b00011001,
                        "e": 0b00010001,
                        "f": 0b00001011,
                        "g": 0b00011011,
                        "h": 0b00010011,
                        "i": 0b00001010,
                        "j": 0b00011010,
                        "k": 0b00000101,
                        "l": 0b00000111,
                        "m": 0b00001101,
                        "n": 0b00011101,
                        "o": 0b00010101,
                        "p": 0b00001111,
                        "q": 0b00011111,
                        "r": 0b00010111,
                        "s": 0b00001110,
                        "t": 0b00011110,
                        "u": 0b00100101,
                        "v": 0b00100111,
                        "w": 0b00111010,
                        "x": 0b00101101,
                        "y": 0b00111101,
                        "z": 0b00110101,
                        "#": 0b00111100,
                        "1": 0b00000001,
                        "2": 0b00000011,
                        "3": 0b00001001,
                        "4": 0b00011001,
                        "5": 0b00010001,
                        "6": 0b00001011,
                        "7": 0b00011011,
                        "8": 0b00010011,
                        "9": 0b00001010,
                        "0": 0b00011010,
                        ".": 0b00110010,
                        ",": 0b00000010,
                        "!": 0b00010110,
                        "?": 0b00100110,
        }

    def load_text_file(self):
        with open(self.file_location) as f:
            for iy,ix in np.ndindex(self.ascii_page.shape):
                c = f.read(1).lower()
                if ord(c) < 32:
                    c = f.read(1).lower()
                if not c:
                    print(")End of file")
                    break
                self.ascii_page[iy,ix] = ord(c)           
        f.close()

    def draw_text_page(self):
        #draw vertical rectangles

        for i in range(self.width-1):
            point_start = (self.pixels_per_char*(i+1) + self.horizontal_spacer_pixels * i, 0)
            point_end = (self.pixels_per_char*(i+1)+self.horizontal_spacer_pixels * (i+1), self.total_height_in_pixels)
            self.img = cv2.rectangle(self.img, point_start, point_end, (200,200,200), -1)

        #draw horizontal rectangles

        for i in range(self.height-1):
            point_start = (0, self.pixels_per_char*(i+1) + self.vertical_spacer_pixels * i)
            point_end = (self.total_width_in_pixels, self.pixels_per_char*(i+1)+self.vertical_spacer_pixels * (i+1))
            self.img = cv2.rectangle(self.img, point_start, point_end, (150,150,150), -1)

        #draw text
        for iy,ix in np.ndindex(self.ascii_page.shape):

            #origin = (ix*pixels_per_char+10,iy*pixels_per_char+30)
            character = chr(self.ascii_page[iy,ix])

            textsize = cv2.getTextSize(character, cv2.FONT_HERSHEY_SIMPLEX, 1, 2)[0]
            textX = int((self.pixels_per_char-textsize[0])/2 + ix*self.pixels_per_char + ix * self.horizontal_spacer_pixels)
            textY = int((self.pixels_per_char + textsize[1])/2 + iy*self.pixels_per_char + iy*self.vertical_spacer_pixels)

            self.img = cv2.putText(self.img, character,(textX,textY), cv2.FONT_HERSHEY_SIMPLEX,1,0,2,cv2.LINE_AA)

    
    def draw_cursor(self, x_coord_int, y_coord_int):

        #cursor_img = np.zeros((self.pixels_per_char*self.height,self.pixels_per_char*self.width,3), dtype=np.uint8)
        #cursor_img.fill(255)
        cursor_img = self.img.copy()


        if x_coord_int < 0:
            x_coord_int = 0
        if x_coord_int >= self.total_width_in_pixels:
            x_coord_int = self.total_width_in_pixels-1

        if y_coord_int < 0:
            y_coord_int = 0
        if y_coord_int >= self.total_height_in_pixels:
            y_coord_int = self.total_height_in_pixels-1

        cursor_img = cv2.circle(cursor_img, (x_coord_int,y_coord_int), radius = 4, color = (0,0,255), thickness = 2)
        cursor_img = cv2.circle(cursor_img, (x_coord_int,y_coord_int), radius = 15, color = (0,0,255), thickness = 2)

        return cursor_img

    def char_to_command(self,character):
        
        return self.braille_dict.get(character,0b00000000)

    def pixel_coord_to_id_and_spacer(self,x_coord, y_coord):

        is_on_horizontal_spacer = False
        is_on_vertical_spacer = False

        x_id = int(x_coord / (self.pixels_per_char + self.horizontal_spacer_pixels))

        if (x_coord % (self.pixels_per_char + self.horizontal_spacer_pixels)) >= self.pixels_per_char:
            is_on_horizontal_spacer = True
        
        y_id = int(y_coord / (self.pixels_per_char + self.vertical_spacer_pixels))

        if (y_coord % (self.pixels_per_char + self.vertical_spacer_pixels)) >= self.pixels_per_char:
            is_on_vertical_spacer = True

        return x_id, y_id, is_on_horizontal_spacer, is_on_vertical_spacer




if __name__ == '__main__':

    rospy.init_node('air_braille_server')

    server = AirBrailleServer()

    q = queue.Queue() 
    tts_thread = TTSThread(q)
   
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    publisher = rospy.Publisher(server.serial_data_out_topic , UInt8, queue_size=10)

    rospy.loginfo("Starting Display Calibration")
    rospy.loginfo("Warm Up Procedure")

    publisher.publish(0b00000000)
    rospy.sleep(0.3)
    publisher.publish(0b00111111)
    rospy.sleep(0.3)
    publisher.publish(0b00000000)
    rospy.sleep(0.3)
    publisher.publish(0b00111111)
    rospy.sleep(0.3)

    rospy.loginfo("Moving All Pins Down")

    publisher.publish(0b00000000)

    rospy.sleep(0.3)
    r = rospy.Rate(10)

    for _ in range(10):
        #Sforce move pins down
        publisher.publish(0b10000000)
        r.sleep()

    rospy.loginfo("Moving Pins Up by Calibrated Distances")

    for _ in range(server.pin_1_offset):
        publisher.publish(0b10011001)
        r.sleep()
    
    for _ in range(server.pin_2_offset):
        publisher.publish(0b10011010)
        r.sleep()

    for _ in range(server.pin_3_offset):
        publisher.publish(0b10011011)
        r.sleep()

    for _ in range(server.pin_4_offset):
        publisher.publish(0b10011100)
        r.sleep()

    for _ in range(server.pin_5_offset):
        publisher.publish(0b10011101)
        r.sleep()

    for _ in range(server.pin_6_offset):
        publisher.publish(0b10011110)
        r.sleep()

    publisher.publish(0b00111111)

    rospy.loginfo("Finished Display Calibration")

    #get position of point 1
    rospy.loginfo("waiting for first point")
    #wait until button pressed
    rospy.wait_for_message(server.serial_data_in_topic, UInt8)

    #get current transform
    try:
        trans1 = tfBuffer.lookup_transform(server.world_frame,server.device_frame,rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr("tf lookup transform error")

    #trans is a transfrorm stampted
    trans1.header.frame_id = server.world_frame
    trans1.child_frame_id = "point1"
    #publish it
    #broadcaster1.sendTransform(trans1)

    #get position of point 2
    rospy.loginfo("waiting for second point")
    #wait until button pressed
    rospy.wait_for_message(server.serial_data_in_topic, UInt8)

    #get current transform
    try:
        trans2 = tfBuffer.lookup_transform(server.world_frame,server.device_frame,rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr("tf lookup transform error")

    #trans is a transfrorm stampted
    trans2.header.frame_id = server.world_frame
    trans2.child_frame_id = "point2"
    #publish it
    #broadcaster2.sendTransform(trans2)

    #get position of point 3
    rospy.loginfo("waiting for third point")
    #wait until button pressed
    rospy.wait_for_message(server.serial_data_in_topic, UInt8)

    #get current transform
    try:
        trans3 = tfBuffer.lookup_transform(server.world_frame,server.device_frame,rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr("tf lookup transform error")

    #trans is a transfrorm stampted
    trans3.header.frame_id = server.world_frame
    trans3.child_frame_id = "point3"
   
    #use the service to get the aligned point

    rospy.wait_for_service('align_plane')
    try:
        align_plane = rospy.ServiceProxy('align_plane',AlignPlane)
        req = AlignPlaneRequest()
        req.point1 = trans1.transform.translation
        req.point2 = trans2.transform.translation
        req.point3 = trans3.transform.translation

        resp = align_plane(req)

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s"%e)

    trans_aligned = TransformStamped()
    trans_aligned.transform.translation = resp.alignedpose.position
    trans_aligned.transform.rotation = resp.alignedpose.orientation
    trans_aligned.header.frame_id = server.world_frame
    trans_aligned.child_frame_id = "aligned_point"

    broadcaster.sendTransform([trans1, trans2, trans3,trans_aligned])
    rospy.sleep(0.1)

    pixels_per_meter_x = server.total_width_in_pixels / resp.page_width
    pixels_per_meter_y = server.total_height_in_pixels / resp.page_height

    r = rospy.Rate(20)

    old_x_character_id = server.width
    old_y_character_id = server.height

    while not rospy.is_shutdown():

        device_transform = tfBuffer.lookup_transform("aligned_point",server.device_frame,rospy.Time())

        x_distance = device_transform.transform.translation.x
        y_distance = device_transform.transform.translation.y

        x_pixel_coord_int = int(x_distance * pixels_per_meter_x)
        y_pixel_coord_int = int(y_distance * pixels_per_meter_y)

        #check it is within page bounds
        if x_pixel_coord_int >= 0 and x_pixel_coord_int < server.total_width_in_pixels and y_pixel_coord_int >= 0 and y_pixel_coord_int < server.total_height_in_pixels:
            
            x_character_id, y_character_id, is_on_horizontal_spacer, is_on_vertical_spacer = server.pixel_coord_to_id_and_spacer(x_pixel_coord_int, y_pixel_coord_int)

            if is_on_vertical_spacer:
                #high vibration
                publisher.publish(0b01110100)
            elif is_on_horizontal_spacer:
                #slight vibration
                publisher.publish(0b01100100)
            else:
                if server.sound_enabled:
                    #check if it is on the start character
                    if x_character_id == 0 and y_character_id == 0 and (x_character_id != old_x_character_id or y_character_id != old_y_character_id):
                        q.put("start")
                    elif y_character_id != old_y_character_id:
                        q.put(str(y_character_id+1))

                #it is directly over a character cell
                current_character_ascii = chr(server.ascii_page[y_character_id,x_character_id])
                #print(current_character_ascii)

                publisher.publish(server.char_to_command(current_character_ascii))

                old_x_character_id = x_character_id
                old_y_character_id = y_character_id

                
        else:
            #outside the page bounds
            publisher.publish(0b01010010)


        img = server.draw_cursor(x_pixel_coord_int, y_pixel_coord_int)

        cv2.imshow('Air Braille',img)
        cv2.waitKey(1)

        r.sleep()
       
        #print(x_character_id, "and ", y_character_id)
 
    cv2.destroyAllWindows()

