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


class AirBrailleServer:
    def __init__(self):

        self.file_location = str(rospy.get_param('~file_location'))
        self.file_type = str(rospy.get_param('~file_type'))

        self.width = rospy.get_param('~width')
        self.height = rospy.get_param('~height')

        self.world_frame =str(rospy.get_param('~world_frame'))
        self.device_frame = str(rospy.get_param('~device_frame'))

        self.pixels_per_char = rospy.get_param('~pixels_per_char')

        self.img = np.zeros((self.pixels_per_char*self.height,self.pixels_per_char*self.width,3), dtype=np.uint8)
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
                        "c": 0b00100001,
                        "d": 0b00110001,
                        "e": 0b00010001,
                        "f": 0b00100011,
                        "g": 0b00110011,
                        "h": 0b00010011,
                        "i": 0b00100010,
                        "j": 0b00110010,
                        "k": 0b00000101,
                        "l": 0b00000111,
                        "m": 0b00100101,
                        "n": 0b00110101,
                        "o": 0b00010101,
                        "p": 0b00100111,
                        "q": 0b00110111,
                        "r": 0b00010111,
                        "s": 0b00100110,
                        "t": 0b00110110,
                        "u": 0b00001101,
                        "v": 0b00001111,
                        "w": 0b00111010,
                        "x": 0b00101101,
                        "y": 0b00111101,
                        "z": 0b00011101,
                        "#": 0b00111100,
                        "1": 0b00000001,
                        "2": 0b00000011,
                        "3": 0b00100001,
                        "4": 0b00110001,
                        "5": 0b00010001,
                        "6": 0b00100011,
                        "7": 0b00110011,
                        "8": 0b00010011,
                        "9": 0b00100010,
                        "0": 0b00110010,
                        ".": 0b00011010,
                        ",": 0b00000010,
                        "!": 0b00010110,
                        "?": 0b00001110,
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
        #draw vertical lines

        for i in range(self.width+1):
            point_top = (self.pixels_per_char*i-1, 0)
            point_bottom = (self.pixels_per_char*i-1, self.pixels_per_char*self.height)
            self.img = cv2.line(self.img,point_top, point_bottom, 0,2)

        #draw horizontal lines
        for i in range(self.height+1):
            point_left = (0, self.pixels_per_char*i-1)
            point_right = (self.width*self.pixels_per_char, self.pixels_per_char*i-1)
            self.img = cv2.line(self.img,point_left, point_right, 0,2)

        for iy,ix in np.ndindex(self.ascii_page.shape):

            #origin = (ix*pixels_per_char+10,iy*pixels_per_char+30)
            character = chr(self.ascii_page[iy,ix])

            textsize = cv2.getTextSize(character, cv2.FONT_HERSHEY_SIMPLEX, 1, 2)[0]
            textX = int((self.pixels_per_char-textsize[0])/2 + ix*self.pixels_per_char)
            textY = int((self.pixels_per_char + textsize[1])/2 + iy*self.pixels_per_char)

            self.img = cv2.putText(self.img, character,(textX,textY), cv2.FONT_HERSHEY_SIMPLEX,1,0,2,cv2.LINE_AA)
    
    def draw_cursor(self, x_char_id, y_char_id):

        #cursor_img = np.zeros((self.pixels_per_char*self.height,self.pixels_per_char*self.width,3), dtype=np.uint8)
        #cursor_img.fill(255)
        cursor_img = self.img.copy()

        x_coord = int(x_char_id*self.pixels_per_char)
        y_coord = int(y_char_id*self.pixels_per_char)

        if x_coord < 0:
            x_coord = 0
        if x_coord >= (self.width*self.pixels_per_char):
            x_coord = (self.width*self.pixels_per_char)

        if y_coord < 0:
            y_coord = 0
        if y_coord >= (self.height*self.pixels_per_char):
            y_coord = (self.height*self.pixels_per_char)

        cursor_img = cv2.circle(cursor_img, (x_coord,y_coord), radius = 3, color = (0,0,255), thickness = 2)
        cursor_img = cv2.circle(cursor_img, (x_coord,y_coord), radius = 15, color = (0,0,255), thickness = 2)

        return cursor_img

    def char_to_command(self,character):
        
        return self.braille_dict.get(character,0b00000000)







if __name__ == '__main__':

    rospy.init_node('air_braille_server')

    server = AirBrailleServer()
   
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    publisher = rospy.Publisher(server.serial_data_out_topic , UInt8, queue_size=10)



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


    #print("trans1")
    #print(trans1)

    #print("response")
    #print(resp)

    trans_aligned = TransformStamped()
    trans_aligned.transform.translation = resp.alignedpose.position
    trans_aligned.transform.rotation = resp.alignedpose.orientation
    trans_aligned.header.frame_id = server.world_frame
    trans_aligned.child_frame_id = "aligned_point"


    #print("trans_aligned")
    #print(trans_aligned)

    #print(trans1)
    
    broadcaster.sendTransform([trans1, trans2, trans3,trans_aligned])
    rospy.sleep(0.5)

    #get transform from aligned to device

    chars_per_meter_x = server.width / resp.page_width
    chars_per_meter_y = server.height / resp.page_height

    old_x_character_id_int = 0
    old_y_character_id_int = 0

    r = rospy.Rate(20)

    while not rospy.is_shutdown():
        device_transform = tfBuffer.lookup_transform("aligned_point",server.device_frame,rospy.Time())

        x_distance = device_transform.transform.translation.x
        y_distance = device_transform.transform.translation.y

        x_character_id = x_distance * chars_per_meter_x
        y_character_id = server.height - y_distance * chars_per_meter_y

        x_character_id_int = int(x_character_id)
        y_character_id_int = int(y_character_id)

        if x_character_id_int >= 0 and x_character_id_int < server.width and y_character_id_int >=0 and y_character_id_int < server.height:

            current_character_ascii = chr(server.ascii_page[y_character_id_int,x_character_id_int])
            print(current_character_ascii)

            publisher.publish(server.char_to_command(current_character_ascii))

            if x_character_id_int != old_x_character_id_int:
            #slight vibration
                publisher.publish(0b01100100)
            

            if y_character_id_int != old_y_character_id_int:
                #max high vibration
                publisher.publish(0b01110100)

        else:
            #out of bounds, slight vibration
            publisher.publish(0b01010010)


        

        old_x_character_id_int = x_character_id_int
        old_y_character_id_int = y_character_id_int



        img = server.draw_cursor(x_character_id, y_character_id)
        cv2.imshow('Air Braille',img)
        #cv2.waitKey(delay_in_ms)
        cv2.waitKey(1)

        r.sleep()
       
        #print(x_character_id, "and ", y_character_id)
 
    cv2.destroyAllWindows()

