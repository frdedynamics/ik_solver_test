Pretty fast ik solver but I couln't set joint limits there. Probably not supported. Maybe a selection would be fine.

Update 12/04/2021:
I will make it as a class in my ros package


How created:
python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot=ur5e.robot.xml --iktype=translation3d --baselink=3 --eelink=6 --savefile=ikfast31.cpp --maxcasedepth 1

