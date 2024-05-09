#
# Copyright 2021 HiveMQ GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
import time
import paho.mqtt.client as paho
from paho import mqtt
import smbus
# import SMBus module of I2C
from time import sleep          #import
import time
import math

#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

alpha = 0.98
beta = 1 - alpha

last_time = time.time()
last_angle = 0

#constant
NUM_SAMPLES = 1000



def MPU_Init():
	#write to sample rate register
	bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
	
	#Write to power management register
	bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
	
	#Write to Configuration register
	bus.write_byte_data(Device_Address, CONFIG, 0)
	
	#Write to Gyro configuration register
	bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
	
	#Write to interrupt enable register
	bus.write_byte_data(Device_Address, INT_ENABLE, 1)
    
def read_raw_data(addr):
	#Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value

bus = smbus.SMBus(1)
# or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

MPU_Init()

#calibrating accelerometer
def calibrate_accelerometer():
    bias_acc_x = 0
    bias_acc_y = 0
    bias_acc_z = 0
    
    for _ in range(NUM_SAMPLES):
        cal_accel_x = read_raw_data(ACCEL_XOUT_H) / 16384.0
        cal_accel_y = read_raw_data(ACCEL_YOUT_H) / 16384.0
        cal_accel_z = read_raw_data(ACCEL_ZOUT_H) / 16384.0
        
        bias_acc_x += cal_accel_x
        bias_acc_y += cal_accel_y
        bias_acc_z += cal_accel_z
        
    bias_acc_x /= NUM_SAMPLES
    bias_acc_y /= NUM_SAMPLES
    bias_acc_z /= NUM_SAMPLES
    
    return bias_acc_x, bias_acc_y, bias_acc_z

#calibrating gyroscope
def calibrate_gyroscope():
    bias_gyro_x = 0
    bias_gyro_y = 0
    bias_gyro_z = 0
    
    for _ in range(NUM_SAMPLES):
        cal_gyro_x = read_raw_data(GYRO_XOUT_H) / 131.0
        cal_gyro_y = read_raw_data(GYRO_XOUT_H) / 131.0
        cal_gyro_z = read_raw_data(GYRO_XOUT_H) / 131.0
        
        bias_gyro_x += cal_gyro_x
        bias_gyro_y += cal_gyro_y
        bias_gyro_z += cal_gyro_z
        
    bias_gyro_x /= NUM_SAMPLES
    bias_gyro_y /= NUM_SAMPLES
    bias_gyro_z /= NUM_SAMPLES
    
    return bias_gyro_x, bias_gyro_y, bias_gyro_z

#calibrate accel
accel_bias_x, accel_bias_y, accel_bias_z = calibrate_accelerometer()

#calibrate gyro
gyro_bias_x, gyro_bias_y, gyro_bias_z = calibrate_gyroscope()
    


# setting callbacks for different events to see if it works, print the message etc.
def on_connect(client, userdata, flags, rc, properties=None):
    print("CONNACK received with code %s." % rc)

# with this callback you can see if your publish was successful
def on_publish(client, userdata, mid, properties=None):
    print("mid: " + str(mid))

# print which topic was subscribed to
def on_subscribe(client, userdata, mid, granted_qos, properties=None):
    print("Subscribed: " + str(mid) + " " + str(granted_qos))

# print message, useful for checking if it was successful
def on_message(client, userdata, msg):
    print(msg.topic + " " + str(msg.qos) + " " + str(msg.payload))

# using MQTT version 5 here, for 3.1.1: MQTTv311, 3.1: MQTTv31
# userdata is user defined data of any type, updated by user_data_set()
# client_id is the given name of the client
client = paho.Client(client_id="", userdata=None, protocol=paho.MQTTv5)
client.on_connect = on_connect

# enable TLS for secure connection
client.tls_set(tls_version=mqtt.client.ssl.PROTOCOL_TLS)
# set username and password
client.username_pw_set("test1", "Test1test1")
# connect to HiveMQ Cloud on port 8883 (default for MQTT)
client.connect("604435b82fbe4893baac9a823f140b9d.s1.eu.hivemq.cloud", 8883)

# setting callbacks, use separate functions like above for better visibility
client.on_subscribe = on_subscribe
client.on_message = on_message
client.on_publish = on_publish

# subscribe to all topics of encyclopedia by using the wildcard "#"
client.subscribe("project/potential", qos=1)

messages = []

def add(tmp):
    messages.append(tmp)
    if len(messages) > 10:
        messages.pop(0)
    average = calculate_average()
    send(average)
    
def calculate_average():
    sum = 0
    for message in messages:
        sum += message
    return sum / len(messages)
    
def send(msg):
    client.publish("project/data", payload=msg, qos=0)

# a single publish, this can also be done in loops, etc.
while True:
	
	#Read Accelerometer raw value
    acc_x = read_raw_data(ACCEL_XOUT_H)
    acc_y = read_raw_data(ACCEL_YOUT_H)
    acc_z = read_raw_data(ACCEL_ZOUT_H)
	
	#Read Gyroscope raw value
    gyro_x = read_raw_data(GYRO_XOUT_H)
    gyro_y = read_raw_data(GYRO_YOUT_H)
    gyro_z = read_raw_data(GYRO_ZOUT_H)
	
	#Full scale range +/- 250 degree/C as per sensitivity scale factor
    Ax = acc_x/16384.0
    Ay = acc_y/16384.0
    Az = acc_z/16384.0
	
    Gx = gyro_x/131.0
    Gy = gyro_y/131.0
    Gz = gyro_z/131.0
    
    #calibrated readings
    cAx = (Ax - accel_bias_x)
    cAy = (Ay - accel_bias_y)
    cAz = (Az - accel_bias_z)
    
    #calculate elapsed time
    current_time = time.time()
    dt = current_time - last_time
    last_time = current_time
	
    #calculate pitch and roll angles using accelerometer data
    pitch = math.atan2(cAy, math.sqrt(cAx ** 2 + cAz ** 2))
    roll = math.atan2(-cAx, cAz)
    
    #combine gyroscope and accelerometer data using complementary filter
    pitch = alpha * (pitch + Gx * dt) + beta * pitch
    roll = alpha * (roll + Gy * dt) + beta * roll
    
    #calculate gravity vector
    gravity_x = math.cos(pitch) * math.sin(roll)
    gravity_y = math.sin(pitch)
    gravity_z = math.cos(pitch) * math.cos(roll)
    
    #remove gravity component from accelerometer readings
    lAx = Ax * math.cos(roll) + Az * math.sin(roll)
    lAy = Ay * math.cos(pitch) - Ax * math.sin(roll) * math.sin(pitch) + Az * math.cos(roll) * math.sin(pitch)
    lAz = -Ay * math.sin(pitch) + Ax * math.cos(pitch) * math.sin(roll) + Az * math.cos(pitch) * math.cos(roll)
    
    #string to send
    msg = math.sqrt(lAx ** 2 + lAy ** 2 + lAz ** 2)
    
    add(msg)

    sleep(0.1)

# loop_forever for simplicity, here you need to stop the loop manually
# you can also use loop_start and loop_stop
client.loop_forever()
