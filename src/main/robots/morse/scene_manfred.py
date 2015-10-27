"""Created on Feb 28, 2014 by @author lucag.
"""

from morse.builder import ATRV, Pose, Waypoint, Environment, SemanticCamera, PassiveObject,Destination


from morse.builder import Zone
from math import pi
from morse.builder import *
import morse.helpers.colors
import json

import os
path = os.getcwd() + "/src/main/robots/morse/"


box_path = path + 'custom/indoors-1/boxes'
indoor_path = path + 'custom/indoors-1/indoor-1'
empty = path + "custom/indoors-1/empty-room"

def build():
    # Place robots in the environment
    
    atrv = ATRV('robot1_instance')
    
    orientation_robot1_instance = Orientation()
    orientation_robot1_instance.translate(x=0, y=0, z=0)
    orientation_robot1_instance.rotate(0, 0, 0)

    atrv.append(orientation_robot1_instance)
    orientation_robot1_instance.add_stream("socket")
    orientation_robot1_instance.properties(speed=1.0)
    
    
    pose_robot1_instance = Pose()
    pose_robot1_instance.add_interface('socket')
    atrv.append(pose_robot1_instance)
    
    motion_robot1_instance = Waypoint()
    #motion = Destination()
    #motion.properties(ControlType = 'Position')
    motion_robot1_instance.properties(ObstacleAvoidance = False)
    motion_robot1_instance.add_interface('socket')
    atrv.append(motion_robot1_instance)

    # creates a new instance of the sensor
    camera_robot1_instance = SemanticCamera('camera_robot1_instance')
    # place your component at the correct location
    camera_robot1_instance.translate(0.0, 0.0, 40.0) # 20.0
    camera_robot1_instance.rotate(0.0, -pi/2.0, 0.0)
    # define one or several communication interface, like 'socket'
    camera_robot1_instance.add_stream('socket')
    atrv.append(camera_robot1_instance)

    proximity_robot1_instance = Proximity()
    atrv.append(proximity_robot1_instance)
    proximity_robot1_instance.properties(Range = 4.0, Track = "Catch_me")
    proximity_robot1_instance.add_stream('socket')
    proximity_robot1_instance.add_service('socket')
    #proximity.set_tracked_tag("Catch_me")
    atrv.properties(Object=True)

    #atrv.set_color(0, 0, 0)

    atrv2 = ATRV(name='robot2_instance')


    orientation_robot2_instance = Orientation()
    orientation_robot2_instance.translate(x=3, y=3, z=0)
    orientation_robot2_instance.rotate(0, 0, 0)

    atrv2.append(orientation_robot2_instance)
    orientation_robot2_instance.add_stream("socket")
    orientation_robot2_instance.properties(speed=1.0)

    #atrv2.scale = (1.5, 1.5, 1.5)

    atrv2.set_color(.75, .75, .5)

    #print(dir(atrv2))
    #quit()

    #atrv2 = ATRV('robot2_instance')
    pose_robot2_instance = Pose()
    pose_robot2_instance.add_interface('socket')
    atrv2.append(pose_robot2_instance)
    
    motion_robot2_instance = Waypoint()
    #motion = Destination()
    #motion.properties(ControlType = 'Position')
    motion_robot2_instance.properties(ObstacleAvoidance = False)
    motion_robot2_instance.add_interface('socket')
    atrv2.append(motion_robot2_instance)

    proximity_robot2_instance = Proximity()
    atrv2.append(proximity_robot2_instance)
    proximity_robot2_instance.properties(Range = 4.0, Track = "Catch_me")
    proximity_robot2_instance.add_stream('socket')
    proximity_robot2_instance.add_service('socket')

    #assign(atrv2)

    atrv2.translate(x=3.0, y=3.0, z=0.0)
    # creates a new instance of the sensor
    camera_robot2_instance = SemanticCamera('camera_robot2_instance')
    #camera_robot2_instance.properties(tag="box")
    # place your component at the correct location
    camera_robot2_instance.translate(3.0, 3.0, 40.0)
    camera_robot2_instance.rotate(0.0, -pi/2.0, 0.0)
    # define one or several communication interface, like 'socket'
    camera_robot2_instance.add_stream('socket')
    camera_robot1_instance.properties(noocclusion=True)
    camera_robot2_instance.properties(noocclusion=True)
    atrv2.append(camera_robot2_instance)
    atrv2.properties(Object=True)
    


    box1_instance = PassiveObject(box_path, 'RedBox_big')
    box1_instance.setgraspable()
    box1_instance.translate(x=6.0, y=6.0, z=2)
    box1_instance.properties(Object=True, Label = "box1_instance", Type="box", Catch_me=True, Description=json.dumps({'color': 'red', 'size': 2}))

    #adding a 2nd box
    box2_instance = PassiveObject(box_path, 'BlueBox')
    box2_instance.setgraspable()
    box2_instance.translate(x=-5.0, y=4.0, z=2)
    box2_instance.properties(Object=True, Label = "box2_instance", Type="box", Catch_me=True, Description=json.dumps({'color': 'blue', 'size': 2}))

    #adding a 3rd box
    box3_instance = PassiveObject(box_path, 'GreenBox')
    box3_instance.setgraspable()
    # box3_instance.translate(x=-2, y=8, z=2)
    box3_instance.translate(x=-2, y=-8, z=2)
    #box3.rotate(z=0.2)
    box3_instance.properties(Object=True, Label = "box3_instance", Catch_me=True, Type="box", Description=json.dumps({'color': 'green', 'size': 2}))

    box4_instance = PassiveObject(box_path, 'RedBox_small')
    box4_instance.setgraspable()
    box4_instance.translate(x=3, y=-7.0, z=2)
    #box4.rotate(z=0.2)
    box4_instance.properties(Object=True, Label = "box4_instance", Type="box", Catch_me=True, Description=json.dumps({'color': 'red', 'size': 1}))

    box5_instance = PassiveObject(indoor_path, 'PinkBox')
    box5_instance.setgraspable()
    # box5_instance.translate(x=-8, y=9.0, z=2)
    box5_instance.translate(x=6, y=0.0, z=2)
    #box4.rotate(z=0.2)
    box5_instance.properties(Object=True, Label = "box5_instance", Type="box", Catch_me=True, Description=json.dumps({'color': 'pink', 'size': .5}))

    desk_instance = PassiveObject(indoor_path, 'Desk_1.001')
    desk_instance.setgraspable()
    desk_instance.translate(x=-10, y=-10.0, z=3)
    desk_instance.properties(Object=True, Label = "desk_instance", Type="desk", Catch_me=True, Description=json.dumps({'color': 'brown', 'size': 4}))
    

    # Environment
    env = Environment(empty)
    env.add_service('socket')

    # can see the desk from here...
    env.set_camera_location([-5.0, 5.0, 3.0])
    env.set_camera_rotation([1.2470, -0.0, -0.9854])

    # diagonal bird eye
    env.set_camera_location([-20.0, -20.0, 15.0])
    env.set_camera_rotation([0.8470, -0.0, -0.9854])


if __name__ == '__main__':
    build()
