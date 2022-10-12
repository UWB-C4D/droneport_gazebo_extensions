import pymavlink.mavutil as mavutil
import time
import random 
udp_addr = "127.0.0.1"
udp_port = 14750

mav = mavutil.mavlink_connection(
        'udpout:' + udp_addr + ":" + str(udp_port), source_system=201)
i = -1

for i in range(1000):
    i+=1
    value = random.randint(0,100)
    mav.mav.battery_status_send(101 + i%4, 1+i%4, 0, 0, [0,0,0,0,0,0,0,0,0,0], -1, -1, -1, value)   
    time.sleep(0.1)
