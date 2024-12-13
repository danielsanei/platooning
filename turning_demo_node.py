import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, Int32MultiArray
from geometry_msgs.msg import Twist
import time
import os
from std_msgs.msg import Float32MultiArray

NODE_NAME = 'turn_demo'
CENTROID_TOPIC_NAME = '/centroid'
ACTUATOR_TOPIC_NAME = '/cmd_vel'
OBJECT_TOPIC_NAME = '/object_detection/results'
WALL_TOPIC_NAME = '/wall_detect'

class PathPlanner(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.twist_publisher = self.create_publisher(Twist, ACTUATOR_TOPIC_NAME, 10)
        self.twist_cmd = Twist()
        self.centroid_subscriber = self.create_subscription(Float32MultiArray, CENTROID_TOPIC_NAME,lambda msg: self.controller(msg,CENTROID_TOPIC_NAME),10)
        self.centroid_subscriber
        self.object_detection_subscriber = self.create_subscription(Float32,OBJECT_TOPIC_NAME,lambda msg: self.controller(msg, OBJECT_TOPIC_NAME),10)
        self.object_detection_subscriber
        self.wall_detection_subscriber = self.create_subscription(Float32,WALL_TOPIC_NAME,lambda msg:self.controller(msg, WALL_TOPIC_NAME), 10)
        self.wall_detection_subscriber
        # Default actuator values
        self.declare_parameters(
            namespace='',
            parameters=[
                ('Kp_steering', 1),
                ('Ki_steering', 0),
                ('Kd_steering', 0),
                ('error_threshold', 0.15),
                ('zero_throttle',0.0),
                ('max_throttle', 0.2),
                ('min_throttle', 0.1),
                ('max_right_steering', 0.5),
                ('max_left_steering', -0.5)
            ])
        self.Kp = 0.5 # between [0,1]
        self.Ki = 0 # between [0,1]
        self.Kd = 0.0 # between [0,1]
        self.error_threshold = 0.15 # between [0,1]
        self.zero_throttle = 0.0 # between [-1,1] but should be around 0
        self.max_throttle = 0.2 # between [-1,1]
        self.min_throttle = 0.1 # between [-1,1]
        self.max_right_steering = 1.0 # between [-1,1]
        self.max_left_steering = -1.0 # between [-1,1]
        # initializing PID control
        self.Ts = float(1/5)
        self.need_stop = 0.0
        self.ek = 0 # current error
        self.ek_1 = 0 # previous error
        self.eo = 0
        self.eo_1 = 0
        self.lane = 1.0 #Default as right
        self.proportional_error = 0 # proportional error term for steering
        self.derivative_error = 0 # derivative error term for steering
        self.integral_error = 0 # integral error term for steering
        self.integral_max = 1E-8
        self.get_logger().info(
            f'\nKp_steering: {self.Kp}'
            f'\nKi_steering: {self.Ki}'
            f'\nKd_steering: {self.Kd}'
            f'\nerror_threshold: {self.error_threshold}'
            f'\nzero_throttle: {self.zero_throttle}'
            f'\nmax_throttle: {self.max_throttle}'
            f'\nmin_throttle: {self.min_throttle}'
            f'\nmax_right_steering: {self.max_right_steering}'
            f'\nmax_left_steering: {self.max_left_steering}'
        )

    def controller(self, data, topic_name):
        need_change = False
        # setting up PID control
        self.get_logger().info(f"Received message on {topic_name}: {data.data}")
        if topic_name == OBJECT_TOPIC_NAME:
            self.eo = 0 -  data.data
        elif topic_name == CENTROID_TOPIC_NAME:
            self.ek = data.data[0]
            self.lane = data.data[1]
        if topic_name == WALL_TOPIC_NAME:
            self.need_stop = data.data
        if self.lane == 1.0: #on right lane
           if self.eo > self.ek : #car is righter
                 need_change = False
           else:
                 need_change = True
        if self.lane == -1.0: #on left lane
           if self.eo > self.ek :
                 need_change = True
           else:
                 need_change = False
        if self.need_stop != 1.0: #No obstacle
            self.inf_throttle = self.min_throttle - (self.min_throttle - self.max_throttle) / (1 - self.error_threshold)
            throttle_float_raw = ((self.min_throttle - self.max_throttle)  / (1 - self.error_threshold)) * abs(self.eo) + self.inf_throttle
            throttle_float = self.clamp(throttle_float_raw, self.max_throttle, self.min_throttle)
        else: #Obstacle detected
            throttle_float = 0.0
        if True: #follow the car
           # self.inf_throttle = self.min_throttle - (self.min_throttle - self.max_throttle) / (1 - self.error_threshold)
           # throttle_float_raw = ((self.min_throttle - self.max_throttle)  / (1 - self.error_threshold)) * abs(self.eo) + self.inf_throttle
           # throttle_float = self.clamp(throttle_float_raw, self.max_throttle, self.min_throttle)

        # Steering PID terms
            self.proportional_error = self.Kp * self.eo
            self.derivative_error = self.Kd * (self.eo - self.eo_1) / self.Ts
            self.integral_error += self.Ki * self.eo * self.Ts
            self.integral_error = self.clamp(self.integral_error, self.integral_max)
            steering_float_raw = self.proportional_error + self.derivative_error + self.integral_error
            steering_float = self.clamp(steering_float_raw, self.max_right_steering, self.max_left_steering)
        # Throttle gain scheduling (function of error)
        else: #do lane following
            self.get_logger().info('lane_following')
           # self.inf_throttle = self.min_throttle - (self.min_throttle - self.max_throttle) / (1 - self.error_threshold)
           # throttle_float_raw = ((self.min_throttle - self.max_throttle)  / (1 - self.error_threshold)) * abs(self.ek) + self.inf_throttle
           # throttle_float = self.clamp(throttle_float_raw, self.max_throttle, self.min_throttle)

        # Steering PID terms
            self.proportional_error = self.Kp * self.ek
            self.derivative_error = self.Kd * (self.ek - self.ek_1) / self.Ts
            self.integral_error += self.Ki * self.ek * self.Ts
            self.integral_error = self.clamp(self.integral_error, self.integral_max)
            steering_float_raw = self.proportional_error + self.derivative_error + self.integral_error
            steering_float = self.clamp(steering_float_raw, self.max_right_steering, self.max_left_steering)


        # Publish values
        try:
            # publish control signals
            self.twist_cmd.angular.z = steering_float
            self.twist_cmd.linear.x = throttle_float
            self.twist_publisher.publish(self.twist_cmd)
            if throttle_float == 0.0:
               time.sleep(1/10) #The lidar detection seems to be inconsistent. Pause for a second to make it more robust)
            # shift current time and error values to previous values
            self.eo_1 = self.eo
            self.ek_1 = self.ek

        except KeyboardInterrupt:
            self.twist_cmd.linear.x = self.zero_throttle
            self.twist_publisher.publish(self.twist_cmd)

    def clamp(self, value, upper_bound, lower_bound=None):
        if lower_bound==None:
            lower_bound = -upper_bound # making lower bound symmetric about zero
        if value < lower_bound:
            value_c = lower_bound
        elif value > upper_bound:
            value_c = upper_bound
        else:
            value_c = value
        return value_c


def main(args=None):
    rclpy.init(args=args)
    path_planner_publisher = PathPlanner()
    try:
        rclpy.spin(path_planner_publisher)
        path_planner_publisher.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        path_planner_publisher.get_logger().info(f'Shutting down {NODE_NAME}...')
        path_planner_publisher.twist_cmd.linear.x = path_planner_publisher.zero_throttle
        path_planner_publisher.twist_publisher.publish(path_planner_publisher.twist_cmd)
        time.sleep(1)
        path_planner_publisher.destroy_node()
        rclpy.shutdown()
        path_planner_publisher.get_logger().info(f'{NODE_NAME} shut down successfully.')


if __name__ == '__main__':
    main()
