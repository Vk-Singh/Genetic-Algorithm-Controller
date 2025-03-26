"""task1_BBR controller."""
from controller import Robot
import time
import sys

class Controller:
    def __init__(self, robot):        
        # Robot Parameters
        self.robot = robot
        self.time_step = 32
        self.max_speed = 6.28
 
        # Enable Motors
        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
    
        # Enable Proximity Sensors
        self.proximity_sensors = []
        for i in range(8):
            sensor_name = 'ps' + str(i)
            self.proximity_sensors.append(self.robot.getDevice(sensor_name))
            self.proximity_sensors[i].enable(self.time_step)
       
        # Enable Ground Sensors
        self.left_ir = self.robot.getDevice('gs0')
        self.left_ir.enable(self.time_step)
        self.center_ir = self.robot.getDevice('gs1')
        self.center_ir.enable(self.time_step)
        self.right_ir = self.robot.getDevice('gs2')
        self.right_ir.enable(self.time_step)
        
        self.grounds = []
        self.distances = []
        self.behaviors = []
        
        # If there is a mark on the floor
        self.clue = 0
        
    def save_ground_values(self):
        self.grounds = []
        self.grounds.append(self.left_ir.getValue())
        self.grounds.append(self.center_ir.getValue())
        self.grounds.append(self.right_ir.getValue())
        return
        
    def save_distance_values(self):
        self.distances = []
        for i in range(8):
            self.distances.append(self.proximity_sensors[i].getValue())
            # print("i: ", i, "val :", self.proximity_sensors[i].getValue())
        return
        
    def normal_behavior(self):
        self.left_motor.setVelocity(self.max_speed)
        self.right_motor.setVelocity(self.max_speed)
        return
        
    def stop_behavior(self):
        if self.distances[0] > 900 and self.distances[7] > 900 and self.distances[2] > 200 and self.distances[5] > 200:
            self.left_motor.setVelocity(0.0)
            self.right_motor.setVelocity(0.0)
            print("Got the food!")
            sys.exit(0)
        return
        
    def avoid_wall_right(self):
        if self.distances[1] > 200:
            self.left_motor.setVelocity(-1.5)
            self.right_motor.setVelocity(1.5)
        return

    def avoid_wall_left(self):
        if self.distances[6] > 200:
            self.left_motor.setVelocity(1.5)
            self.right_motor.setVelocity(-1.5)
        return
    def detect_clue(self):
        if self.grounds[0] < 400 and self.grounds[1] < 400 and self.grounds[2] < 400:
            self.clue = 1
        return
        
    def intersection_behavior(self):
        if self.distances[2] < 70 and self.distances[5] < 70:
            if self.clue == 1:
                self.left_motor.setVelocity(1.5)
                self.right_motor.setVelocity(-1.5)
            else:
                self.left_motor.setVelocity(-1.5)
                self.right_motor.setVelocity(1.5)
        return
                
        
    def run_robot(self):
            # Main loop
        while self.robot.step(self.time_step) != -1:
            # Read and save input ground sensors
            self.save_ground_values()
            # Read and save input distance sencors
            self.save_distance_values()
            
            self.stop_behavior() 
            self.normal_behavior()
            self.avoid_wall_right()
            self.avoid_wall_left()
            self.detect_clue()
            self.intersection_behavior()
                
# Enter here exit cleanup code.
if __name__ == "__main__":
    # create the Robot instance.
    rat_robot = Robot()
    controller = Controller(rat_robot)
    controller.run_robot()
    