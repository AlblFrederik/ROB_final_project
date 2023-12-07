import sys
from robotCRS import robCRS93
from CRS_commander import Commander
from test import circle_trajectory

rob = robCRS93()  # nebo rob = robCRS97()
c = Commander(rob)
c.open_comm(r"/dev/ttyUSB0")  # ???/dev/ttyUSB0 ???
#c.init()
c.hard_home()
print("circle - start")
sol = circle_trajectory(c)
print("circle - stop")
# c.soft_home()
c.rcon.close()
