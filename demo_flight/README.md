# demo_flight
code of demo flight on Feb. 25th,2016

I use yuyun's math labrary and paul yang's PID library.

just do not need to change the code.

use :

    roslaunch demo_flight demo_ground.launch

and you will get all msg about vicon and the whole links

make sure the name of wand(stick) in vicon space is "wand" and the msg name is "/vicon/wand/wand"

make sure the name of M100 in vicon space is "M100_1" and the msg name is "/vicon/M100_1/M100_1"

be careful the friquancy of the messages from vicon-node is up to 200Hz!!!I limited SDK-message sent out friquancy to 48 Hz in pid-node.
