import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from Phidget22.Phidget import *
from Phidget22.Devices.BLDCMotor import *




class MotorDriver(Node):
   def __init__(self, motorright1, motorright2, motorright3, motorleft1, motorleft2, motorleft3):
       super().__init__('motor_driver')


       self.motorleft1 = motorleft1
       self.motorleft2 = motorleft2
       self.motorleft3 = motorleft3

       self.motorright1 = motorright1
       self.motorright2 = motorright2
       self.motorright3 = motorright3
      
       self.twist_subscription = self.create_subscription(
           Twist,
           '/cmd_vel', 
           self.twist_callback,
           10  # Queue size
       )
      
  
   def twist_callback(self, msg: Twist):

      
       # Extract linear and angular velocities from the Twist message
       linear_velocity = msg.linear.x 
       angular_velocity = msg.angular.z


       # Ensure velocity is a float
       angular_velocity = float(angular_velocity)
       linear_velocity = float(linear_velocity)


       right_speed, left_speed = compute_wheel_speeds(linear_velocity, angular_velocity)


       self.motorleft1.setTargetVelocity(left_speed)
       self.motorleft2.setTargetVelocity(left_speed)
       self.motorleft3.setTargetVelocity(left_speed)

       self.motorright1.setTargetVelocity(right_speed)
       self.motorright2.setTargetVelocity(right_speed)
       self.motorright3.setTargetVelocity(right_speed)




def compute_wheel_speeds(linear_vel, angular_vel):
   left_speed = -(linear_vel + angular_vel) / 2.0
   right_speed = (linear_vel + angular_vel) / 2.0
   return right_speed, left_speed




def initialize_motor(serialNumber, channel):
   motor = BLDCMotor()
  
   # Placeholder values - adjust as needed
   try:
       print("Initializing motor...")
       motor.setDeviceSerialNumber(serialNumber) 
       motor.setChannel(0) 
       motor.setHubPort(channel)
       motor.openWaitForAttachment(5000)
       motor.setAcceleration(5.0)

       print("Motor attached and initialized.")
       return motor


   except PhidgetException as e:
       print(f"Could not initialize motor: {e.details}")
       return None




def close_motor(motorleft1, motorleft2, motorleft3, motorright1, motorright2, motorright3):
   try:
       print("Closing motor...")
       motorleft1.close()
       motorleft2.close()
       motorleft3.close()

       motorright1.close()
       motorright2.close()
       motorright3.close()
       print("Motor closed successfully.")
   except PhidgetException as e:
       print(f"[ERROR] Could not close motor: {e.details}")




def main(args=None):
    rclpy.init(args=args)
    
    motorleft1 = initialize_motor(767272, 3)
    motorleft2 = initialize_motor(767272, 4)
    motorleft3 = initialize_motor(767272, 5)
    
    motorright1 = initialize_motor(767272, 0)
    motorright2 = initialize_motor(767272, 1)
    motorright3 = initialize_motor(767272, 2)
    
    if motorleft1 is None:
        return
    if motorleft2 is None:
        return
    if motorleft3 is None:
        return

    if motorright1 is None:
        return
    if motorright2 is None:
        return
    if motorright3 is None:
        return


    node = MotorDriver(motorright1, motorright2, motorright3, motorleft1, motorleft2, motorleft3)
    #node = MotorDriver(motorright2, motorleft2)
            
    # Spin the node to keep it running
    rclpy.spin(node)

    close_motor(motorleft1, motorleft2, motorleft3, motorright1, motorright2, motorright3)
    #motorleft2.close()
    #motorright2.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
   main()
