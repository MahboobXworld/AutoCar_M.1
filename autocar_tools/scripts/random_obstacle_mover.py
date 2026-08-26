#!/usr/bin/env python3
import rospy
import random
import math
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Pose, Twist

class RandomObstacleMover:
    def __init__(self):
        rospy.init_node('random_obstacle_mover', anonymous=True)
        
        # List of dynamic models we want to move (now including people!)
        self.model_names = [
            'dynamic_box_1', 
            'dynamic_box_2', 
            'dynamic_box_3', 
            'dynamic_barrel_1', 
            'dynamic_barrel_2', 
            'dynamic_vehicle_1',
            'dynamic_person_1',
            'dynamic_person_2'
        ]
        
        # Active models currently spawned in Gazebo
        self.active_models = set()
        
        # State tracking
        self.model_poses = {name: Pose() for name in self.model_names}
        self.model_twists = {name: Twist() for name in self.model_names}
        
        # Target velocities
        self.target_linear_x = {name: 0.0 for name in self.model_names}
        self.target_linear_y = {name: 0.0 for name in self.model_names}
        self.target_angular_z = {name: 0.0 for name in self.model_names}
        
        # Time to next random velocity change
        self.change_time = {name: 0.0 for name in self.model_names}
        
        # Publisher to set model states
        self.state_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
        
        # Subscriber to read current model states
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.states_callback)
        
        rospy.loginfo("[RandomObstacleMover] Initialized and waiting for Gazebo states...")
 
    def states_callback(self, msg):
        current_active = set()
        for name in self.model_names:
            if name in msg.name:
                idx = msg.name.index(name)
                self.model_poses[name] = msg.pose[idx]
                self.model_twists[name] = msg.twist[idx]
                current_active.add(name)
        self.active_models = current_active
 
    def update_targets(self, name, current_time):
        # Update targets if interval passed
        if current_time >= self.change_time[name]:
            self.target_linear_x[name] = random.uniform(-0.8, 0.8)
            self.target_linear_y[name] = random.uniform(-0.8, 0.8)
            self.target_angular_z[name] = random.uniform(-0.6, 0.6)
            # Set next change between 2.0 and 4.0 seconds
            self.change_time[name] = current_time + random.uniform(2.0, 4.0)
            
        # Boundary limits (warehouse walls at +/-17.5m, let's keep them within +/-15.5m)
        x = self.model_poses[name].position.x
        y = self.model_poses[name].position.y
        
        # Push back if close to walls
        if x > 15.0:
            self.target_linear_x[name] = random.uniform(-1.0, -0.4)
        elif x < -15.0:
            self.target_linear_x[name] = random.uniform(0.4, 1.0)
            
        if y > 15.0:
            self.target_linear_y[name] = random.uniform(-1.0, -0.4)
        elif y < -15.0:
            self.target_linear_y[name] = random.uniform(0.4, 1.0)
 
    def run(self):
        dt = 0.05 # 20Hz update loop time delta
        rate = rospy.Rate(20) 
        
        # Wait until we receive states for at least one model
        while not rospy.is_shutdown() and not self.active_models:
            rate.sleep()
            
        rospy.loginfo("[RandomObstacleMover] Received initial states. Starting active movement loop.")
        
        while not rospy.is_shutdown():
            current_time = rospy.get_time()
            
            # Use list snapshot to prevent dictionary size change during iteration
            active_list = list(self.active_models)
            for name in active_list:
                self.update_targets(name, current_time)
                
                # Apply low-pass filter to smooth out velocity updates (simulating physical inertia)
                curr_twist = self.model_twists[name]
                new_twist = Twist()
                new_twist.linear.x = 0.9 * curr_twist.linear.x + 0.1 * self.target_linear_x[name]
                new_twist.linear.y = 0.9 * curr_twist.linear.y + 0.1 * self.target_linear_y[name]
                new_twist.angular.z = 0.9 * curr_twist.angular.z + 0.1 * self.target_angular_z[name]
                
                # Integrate pose based on current velocity to actively force motion (bypassing high static friction)
                pose = self.model_poses[name]
                pose.position.x += new_twist.linear.x * dt
                pose.position.y += new_twist.linear.y * dt
                
                # Enforce hard boundaries
                pose.position.x = max(-16.5, min(16.5, pose.position.x))
                pose.position.y = max(-16.5, min(16.5, pose.position.y))
                
                # Keep objects on ground plane z-level
                if 'person' in name:
                    pose.position.z = 0.85
                elif 'barrel' in name or 'box' in name:
                    pose.position.z = 0.5
                elif 'vehicle' in name:
                    pose.position.z = 0.6
                
                # Prepare ModelState message
                state_msg = ModelState()
                state_msg.model_name = name
                state_msg.pose = pose
                state_msg.twist = new_twist
                state_msg.reference_frame = 'world'
                
                self.state_pub.publish(state_msg)
                
            rate.sleep()

if __name__ == '__main__':
    try:
        mover = RandomObstacleMover()
        mover.run()
    except rospy.ROSInterruptException:
        pass
