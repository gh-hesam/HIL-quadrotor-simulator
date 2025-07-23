from collections import deque
from imusensor.MPU9250 import MPU9250
from imusensor.filters import kalman
import board
import busio
import adafruit_pca9685
import smbus
from PID_lib import PID
import time

import socket
import sys
import redis
import time 


################################################ set 
receive_command = True
apply_commands = True
main_loop_lenght = 750 # steps - int
cte_command = 32499*0.3 # motors power - float16

################################################set reciver 

rec_ip = '192.168.53.94'#server ip
rec_port = 5555
rec_s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Bind the socket to the port
rec_server_address = (rec_ip, rec_port)
rec_s.bind(rec_server_address)

def receive_data():
    data, address = rec_s.recvfrom(4096)
    return data.decode("utf-8")

##############################################set transmitter
trans_ip = '192.168.53.6'#quad ip
trans_port = 6666
trans_s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
def data_transmitter(data):
    trans_s.sendto(data.encode('utf-8'), (trans_ip, trans_port))

################################################# set up PWM generator

i2c = busio.I2C(board.SCL , board.SDA)
pca = adafruit_pca9685.PCA9685(i2c)
pca.frequency = 500
standby = pca.channels[3]
standby.duty_cycle = 64999 # set standby pin to high
Motor0 = pca.channels[13] # set actuators
Motor1 = pca.channels[2]
Motor2 = pca.channels[0] # set actuators
Motor3 = pca.channels[14]


################################################# set up IMU : MPU9250
address = 0x68 
calib_file = "/home/pi/IMU/calib.json" 
roll = 0
pitch = 0 
yaw = 0    
time_difference = 0 # a value that pass with IMU data in each data gathering for controller usage

kalman_filter = kalman.Kalman()
bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, address)
imu.begin()
imu.loadCalibDataFromFile(calib_file)
imu.setLowPassFilterFrequency("AccelLowPassFilter10") # low pass filter

Motor0.duty_cycle = int(0) 
Motor1.duty_cycle = int(0)
Motor2.duty_cycle = int(0)
Motor3.duty_cycle = int(0)
def get_IMU_output() :
    ################################# read IMU [fused data --- without eany filtering]
    imu.readSensor()
    imu.computeOrientation()
    output = [imu.roll , imu.pitch , imu.yaw] 
    ################################################
    return output


######################################################################################
def command_normalizar(command , Min_c , Max_c):
    if command > Max_c :
        return Max_c
    elif command < Min_c :
        return Min_c
    else :
        return command
################################################## set PID controllers
phi_PID = PID(910,10,190,0,-32499,32499,0.01)
theta_PID = PID(910,10,190,0,-32499,32499,0.01)
psi_PID = PID(800,1,5,0,-32499,32499,0.01)

############################################### main loop of control system

Motor0.duty_cycle = int(0) 
Motor1.duty_cycle = int(0)
Motor2.duty_cycle = int(0)
Motor3.duty_cycle = int(0)
phi_hist = []
theta_hist = []
psii_hist = []



s_time = time.time()
for i in range(main_loop_lenght):
    # set PID timer
    phi_PID.timer_stop()
    theta_PID.timer_stop()
    psi_PID.timer_stop()
    # get INS data
    angels = get_IMU_output()
    # send INS data to main PC
    data_transmitter(str(angels))
    #recieve commands from main PC
    if receive_data:
        input_command = receive_data()
    else:
        input_command = [0,0,0]
    phi , theta , psi = angels[0]- input_command[0] , angels[1]-input_command[1] , angels[2]- input_command[2]
    if phi > 0 :
        phi -= 180
    else :
        phi +=180

    phi_PID.time_start()
    theta_PID.time_start()
    psi_PID.time_start()

    # calculate PID commands to apply to motors
    phi_command  = phi_PID.calculate_command_new_2(phi , 0) # command should be in range (-32499 , 32499)
    theta_command = theta_PID.calculate_command_new_2(theta , 0)
    psi_command = psi_PID.calculate_command_new_2(psi , 0)
    M1_command = cte_command + phi_command*0.5 - theta_command*0.5 + psi_command*0.05
    M2_command = cte_command - phi_command*0.5 + theta_command*0.5 + psi_command*0.05
    M3_command = cte_command - phi_command*0.5 - theta_command*0.5 - psi_command*0.05
    M4_command = cte_command + phi_command*0.5 + theta_command*0.5 - psi_command*0.05
    M1_command = command_normalizar(M1_command , 0 , 64999)
    M2_command = command_normalizar(M2_command , 0 , 64999)
    M3_command = command_normalizar(M3_command , 0 , 64999)
    M4_command = command_normalizar(M4_command , 0 , 64999)
    #################################################################

    #apply command
    Motor0.duty_cycle = int(M1_command) 
    Motor1.duty_cycle = int(M2_command)
    Motor2.duty_cycle = int(M3_command) 
    Motor3.duty_cycle = int(M4_command)
# turning off All motors
print(time.time() - s_time)
Motor0.duty_cycle = int(0) 
Motor1.duty_cycle = int(0)
Motor2.duty_cycle = int(0)
Motor3.duty_cycle = int(0)

