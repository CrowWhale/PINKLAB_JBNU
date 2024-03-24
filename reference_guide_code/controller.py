import sys
import threading

import geometry_msgs.msg
import rclpy

import time

def main():
    # settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('teleop_twist_keyboard')

    # parameters
    stamped = node.declare_parameter('stamped', False).value
    frame_id = node.declare_parameter('frame_id', '').value
    if not stamped and frame_id:
        raise Exception("'frame_id' can only be set when 'stamped' is True")

    if stamped:
        TwistMsg = geometry_msgs.msg.TwistStamped
    else:
        TwistMsg = geometry_msgs.msg.Twist

    pub = node.create_publisher(TwistMsg, '/base_controller/cmd_vel_unstamped', 10)

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    speed = 1.0
    turn = 1.0
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0.0

    twist_msg = TwistMsg()

    if stamped:
        twist = twist_msg.twist
        twist_msg.header.stamp = node.get_clock().now().to_msg()
        twist_msg.header.frame_id = frame_id
    else:
        twist = twist_msg
        
	

    try:
        with open('state.txt', 'r') as f:
            state = f.read()
        state = int(state)
        # if state != '':
        #     state = int(state)
        # else:
        #     pass
        # print('1st :', state)

        while state != 0:
            with open('state.txt', 'r') as f:
                state = f.read()
            state = int(state)
            time.sleep(0.1)
            # state = float(state)
            # print('2nd :', state)

            # GO straight = 1
            if state == 1:
                x = 1.0
                y = 0.0
                z = 0.0
                th = 0.0
                
            # Turn Right = 2
            elif state == 2:
                x = 0.0
                y = 0.0
                z = 0.0
                th = -1.0
                
            # Turn Left = 4
            elif state == 4:
                x = 0.0
                y = 0.0
                z = 0.0
                th = 1.0
                           
            else:
                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0
                print(" stop ")
                

            if stamped:
                twist_msg.header.stamp = node.get_clock().now().to_msg()

            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = z * speed
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = th * turn
            pub.publish(twist_msg)
            
			

    except Exception as e:
        print(e)

    finally:
        if stamped:
            twist_msg.header.stamp = node.get_clock().now().to_msg()

        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist_msg)
        rclpy.shutdown()
        spinner.join()

        # restoreTerminalSettings(settings)
    


if __name__ == '__main__':
    main()