to run simulations...

1) dronekit-sitl copter --home=YOUR,COORDINATES,0,0

for example 
dronekit-sitl copter --home=34.2410,118.5277,0,0

2) mavproxy.py --master tcp:127.0.0.1:5760 --out udp:127.0.0.1:14551 --out udp:127.0.0.1:14550

3) python script_name  --connect udp:127.0.0.1:14551

4) mission planner or qgroundcontrl, connect via udp:14550
