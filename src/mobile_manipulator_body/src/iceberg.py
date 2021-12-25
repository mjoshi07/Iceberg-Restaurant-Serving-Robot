#!/usr/bin/env python3
import rospy

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
import spawn_models
import simple_move
import sys, select, termios, tty

msg = """
Control Your Toy!
---------------------------
Moving around:
       i    
   j   k    l
       ,    
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

---------------------------
To operate manipulator:
Press 'a' to make the manipulator acheive configuration suitable to pickup payload
Press 's' to spawn an object on the base of the manipulator which acts as payload
Press 'd' to deliver the payload upon reaching the destination
Press 'f' to detach the object from the manipulator
CTRL-C to quit
"""

# modified the values according to our messed up model car
# conceptually works for all differential drive vehicle
moveBindings = {
        'i':(1,1),
        'j':(-1,1),
        'l':(1,-1),
        ',':(-1,-1)
           }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
          }

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = 4

def vels(speed):
    return "currently:\tspeed %s" % (speed)

def spawn_and_attach(model_name):
    spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    rospy.loginfo("Waiting for /gazebo/spawn_sdf_model service...")
    spawn_srv.wait_for_service()
    rospy.loginfo("Connected to service!")

    
    rospy.loginfo("Spawning cube3")
    req3 = spawn_models.create_cube_request(model_name,
                              0, 10, 2,  # position
                              0.0, 0.0, 0.0,  # rotation
                              0.1, 0.1, 0.1)  # size
    spawn_srv.call(req3)
    rospy.sleep(1.0)
    rospy.loginfo("Spawned cube3 in world")

    simple_move.spawn(model_name)
    rospy.loginfo("Spawned cube3 on robot")

    rospy.loginfo("Creating ServiceProxy to /link_attacher_node/attach")
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',
                                    Attach)
    attach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")

    # Link them
    rospy.loginfo("Attaching cube and end effector")
    req = AttachRequest()
    req.model_name_1 = model_name
    req.link_name_1 = "link"
    req.model_name_2 = "mobile_manipulator"
    req.link_name_2 = "top_wrist"

    attach_srv.call(req)
    rospy.loginfo("Attached cube and end effector")

def detach(model_name):
    rospy.loginfo("Creating ServiceProxy to /link_attacher_node/detach")
    attach_srv = rospy.ServiceProxy('/link_attacher_node/detach',
                                    Attach)
    attach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/detach")

    # Link them
    rospy.loginfo("Detaching cube and arm")
    req = AttachRequest()
    req.model_name_1 = model_name
    req.link_name_1 = "link"
    req.model_name_2 = "mobile_manipulator"
    req.link_name_2 = "top_wrist"

    attach_srv.call(req)


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('iceberg_teleop')
    

    vel_pub = rospy.Publisher('/robot_base_velocity_controller/cmd_vel', Twist, queue_size=10)

    stop=False
    status = 0
    target_speed = 0.0
    left_speed = 0.0
    right_speed = 0.0

    control_speed = 0.0
    vel_cmd = Twist()
    count = 0

    try:
        print(msg)
        print(vels(speed))
       
        while(1):
            left_dir = 0
            right_dir = 0
            key = getKey()
            if(key=='a'):
                simple_move.move_to_pickup()
            if(key=='s'):
                count+=1
                model_name = "cube"+str(count)
                simple_move.move_to_pickup()
                spawn_and_attach(model_name)
            if(key=='d'):
                simple_move.do_pick_and_place()
            if(key=='f'):
                model_name = "cube"+str(count)
                detach(model_name)

            if key in moveBindings.keys():
                left_dir = moveBindings[key][0]
                right_dir = moveBindings[key][1]
  
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                print(vels(speed))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            elif key == ' ' or key == 'k' :
                stop=True
            elif key == '\x03':
                break

            target_speed = speed

            if target_speed > control_speed:
                control_speed = min( target_speed, control_speed + 0.5 )
            elif target_speed < control_speed:
                control_speed = max( target_speed, control_speed - 0.5 )
            else:
                control_speed = target_speed

            if stop:
                control_speed = 0
                stop = False
                left_dir = 0
                right_dir = 0

            if left_dir == right_dir and left_dir != 0:
                vel_cmd.linear.x = left_dir*control_speed  
            else:
                vel_cmd.linear.x = 0
                vel_cmd.angular.z = 0      


            if left_dir < 0 and right_dir > 0:
                vel_cmd.angular.z = 1*control_speed

            if right_dir < 0 and left_dir > 0:
                vel_cmd.angular.z = -1*control_speed

                  
            vel_pub.publish(vel_cmd)

    except Exception as e:
        print(e)

    finally:
        print("########################################################")
        vel_pub.publish(vel_cmd)


    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
