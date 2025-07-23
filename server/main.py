import math
import socket
import sys
import redis
import time 
import matplotlib.pyplot as plt 
import plotly.graph_objects as go
import numpy as np
import matplotlib.pyplot as plt



class quadrotor:
    def __init__(self, x0, y0, z0, phi0, theta0, psii0, thrust0, delta_t, mass, g):
        # init state of quadrotor
        self.x = [x0]
        self.y = [y0]
        self.z = [z0]

        self.xd = [0]
        self.yd = [0]
        self.zd = [0]

        self.phi = [phi0]
        self.theta = [theta0]
        self.psii = [psii0]
        self.thrust = thrust0
        self.delta_t = delta_t
        self.mass = mass
        self.g = g
        ######################################reciever
        self.rec_ip = '192.168.1.106'
        self.rec_port = 6666
        # Create a UDP socket
        self.rec_s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Bind the socket to the port
        self.rec_server_address = (self.rec_ip, self.rec_port)
        self.rec_s.bind(self.rec_server_address)
        print("Do Ctrl+c to exit the program !!")


        #####################################transmitter 

        self.trans_ip = '192.168.1.108'
        self.trans_port = 5555 
        self.trans_s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)


    def move(self, phi, theta, psii, F):
        xdd = (math.cos(psii)*math.sin(theta)*math.cos(phi))+(math.sin(psii)*math.sin(phi))*(F/self.mass)
        ydd = (math.sin(psii)*math.sin(theta)*math.cos(phi))-(math.sin(phi)*math.cos(psii))*(F/self.mass)
        zdd = (math.cos(theta)*math.cos(phi))*(F/self.mass)-self.g

        print(xdd , ydd, zdd)
        #print(phi, theta, psii) 
        xd = self.xd[-1] + self.delta_t*xdd
        self.xd.append(xd)
        yd = self.yd[-1] + self.delta_t*ydd
        self.yd.append(yd)
        zd = self.zd[-1] + self.delta_t*zdd
        self.zd.append(zd)

        X = self.x[-1] + xd*self.delta_t + 0.5*xdd*self.delta_t*self.delta_t
        self.x.append(X)
        Y = self.y[-1] + yd*self.delta_t + 0.5*ydd*self.delta_t*self.delta_t
        self.y.append(Y)
        Z = self.z[-1] + zd*self.delta_t + 0.5*zdd*self.delta_t*self.delta_t
        self.z.append(Z*1)



    def get_3dof_data(self):
        data, address = self.rec_s.recvfrom(4096)
        data = data.decode("utf-8")
        data = data.split(" ")
        roll = float(data[0][1:-1])
        if roll > 0 :
            roll -= 180
        else :
            roll +=180

        pitch = float(data[1][0:-1])
        yaw = float(data[2][0:-1]) 
        self.phi.append(math.radians(roll))
        self.theta.append(math.radians(pitch))
        self.psii.append(math.radians(yaw))


    def send_RPY_command(self, roll, pitch, yaw, thrust):
        self.thrust = thrust
        data = str(roll) + str(',')+str(pitch) + str(',')+str(yaw) + str(',')+str(thrust) + str(',')
        self.trans_s.sendto(data.encode('utf-8'), (self.trans_ip, self.trans_port))


if __name__ == "__main__":
    my_quad = quadrotor(x0=0, y0=0, z0=0, phi0=0, theta0=0, psii0=0, thrust0=10, delta_t=0.01, mass=1, g=10)
    for i in range(1000):
        # get data
        my_quad.send_RPY_command(0,0,0,10.251)
        my_quad.get_3dof_data()
        my_quad.move(my_quad.phi[-1], my_quad.theta[-1], my_quad.psii[-1], my_quad.thrust)
    # plot x, y, z separatly
    plt.plot(my_quad.x)
    plt.plot(my_quad.y)
    plt.plot(my_quad.z)
    plt.show()

    #plot in 3D
    fig = go.Figure(data=go.Scatter3d(
        x=my_quad.x, y=my_quad.y, z=my_quad.z,
        marker=dict(
            size=1,
            colorscale='Viridis',
        ),
        line=dict(
            color='darkblue',
            width=2
        )))
    fig.update_layout(
        width=1220,
        height=900,
        autosize=False,
        scene=dict(
            camera=dict(
                up=dict(
                    x=0,
                    y=0,
                    z=0
                ),
                eye=dict(
                    x=1,
                    y=1.0707,
                    z=1,
                )
            ),
            aspectratio = dict( x=2, y=2, z=1 ),
            aspectmode = 'manual'
        ),
    )

    fig.show()
