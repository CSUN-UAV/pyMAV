to run simulations...

dronekit-sitl copter --home=YOUR,COORDINATES

mavproxy.py --master tcp:127.0.0.1:5760 --out udp:127.0.0.1:14551 --out udp:127.0.0.1:14550

python script_name  --connect udp:127.0.0.1:14551

mission planner or qgroundcontrl, connect via udp:14550