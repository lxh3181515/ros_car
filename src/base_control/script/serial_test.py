#!/usr/bin/python
# coding=gbk

import os
import rospy
import time
import sys
import serial


#class queue is design for uart receive data cache
class queue:
    def __init__(self, capacity = 1024*4):
        self.capacity = capacity
        self.size = 0
        self.front = 0
        self.rear = 0
        self.array = [0]*capacity
 
    def is_empty(self):
        return 0 == self.size
 
    def is_full(self):
        return self.size == self.capacity
 
    def enqueue(self, element):
        if self.is_full():
            raise Exception('queue is full')
        self.array[self.rear] = element
        self.size += 1
        self.rear = (self.rear + 1) % self.capacity
 
    def dequeue(self):
        if self.is_empty():
            raise Exception('queue is empty')
        self.size -= 1
        self.front = (self.front + 1) % self.capacity
 
    def get_front(self):
        return self.array[self.front]
    
    def get_front_second(self):
        return self.array[((self.front + 1) % self.capacity)]

    def get_queue_length(self):
        return (self.rear - self.front + self.capacity) % self.capacity


def crc_byte(data,length):
    pass


Circleloop = queue(capacity = 1024*4)
def timerCommunicationCB(event):
    length = serial.in_waiting
    if length:
        reading = serial.read_all()
        if len(reading)!=0:
            for i in range(0,len(reading)):
                data = (int(reading[i].encode('hex'),16)) 
                try:
                    Circleloop.enqueue(data)
                except:
                    pass
    else:
        pass
    if Circleloop.is_empty()==False:
        data = Circleloop.get_front()
        if data == 0x5a:
            length = Circleloop.get_front_second()
            if length > 1 :
                if Circleloop.get_front_second() <= Circleloop.get_queue_length():
                    databuf = []
                    for i in range(length):
                        databuf.append(Circleloop.get_front())
                        Circleloop.dequeue()
                    
                    if (databuf[length-1]) == crc_byte(databuf,length-1):
                        pass
                    else:
                        pass
                    #parse receive data
                    if (databuf[3] == 0x12):
                        rospy.loginfo("%d %d" %(databuf[8], databuf[9]));                   
                    else:
                        pass
            else:
                pass
        else:
            Circleloop.dequeue()
    else:
        # rospy.loginfo("Circle is Empty")
        pass


def timerOdomCB(event):
    #Get move base velocity data
    output = chr(0x5a) + chr(0x06) + chr(0x01) + chr(0x11) + chr(0x00) + chr(0xa2) 
    try:
        while serial.out_waiting:
            pass
        serial.write(output)
    except:
        rospy.logerr("Odom Command Send Faild")




#main function
if __name__=="__main__":
    # Serial Communication
    device_port = '/dev/ttyTHS1'
    baudrate = 115200
    try:
        serial = serial.Serial(device_port, baudrate, timeout=10)
        rospy.loginfo("Opening Serial")
        try:
            if serial.in_waiting:
                serial.readall()
        except:
            rospy.loginfo("Opening Serial Try Faild")
            pass
    except:
        rospy.logerr("Can not open Serial"+ device_port)
        # serial.close
        sys.exit(0)
    rospy.loginfo("Serial Open Succeed")
        
    try:
        rospy.init_node('base_control',anonymous=True)
        timer_odom = rospy.Timer(rospy.Duration(1.0/50), timerOdomCB)
        timer_communication = rospy.Timer(rospy.Duration(1.0/500), timerCommunicationCB)
        rospy.spin()
    except KeyboardInterrupt:
        serial.close
        print("Shutting down")
