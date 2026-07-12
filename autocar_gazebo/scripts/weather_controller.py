#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from gazebo_msgs.msg import LightState
from gazebo_msgs.srv import SetLightState
from geometry_msgs.msg import Pose

class WeatherController:
    def __init__(self):
        rospy.init_node('weather_controller', anonymous=True)
        
        # Subscribe to target weather mode
        self.weather_sub = rospy.Subscriber('campus_weather', String, self.weather_callback)
        
        # Service client to update light states in Gazebo
        rospy.wait_for_service('/gazebo/set_light_state')
        self.set_light_srv = rospy.ServiceProxy('/gazebo/set_light_state', SetLightState)
        
        rospy.loginfo("Weather controller initialized. Publish to 'campus_weather' with: sunny, night, rainy, or dawn.")

    def weather_callback(self, msg):
        mode = msg.data.lower().strip()
        rospy.loginfo(f"Setting weather mode to: {mode}")
        
        # Prepare light state parameters
        light_state = LightState()
        light_state.light_name = "sun"
        light_state.pose = Pose()
        
        # Set light positions and values based on selected mode
        if mode == "sunny":
            # Normal bright sunny day
            light_state.pose.position.x = 0
            light_state.pose.position.y = 0
            light_state.pose.position.z = 50
            light_state.diffuse.r = 0.8
            light_state.diffuse.g = 0.8
            light_state.diffuse.b = 0.8
            light_state.diffuse.a = 1.0
            light_state.specular.r = 0.1
            light_state.specular.g = 0.1
            light_state.specular.b = 0.1
            light_state.specular.a = 1.0
        elif mode == "night":
            # Dark night
            light_state.pose.position.x = 0
            light_state.pose.position.y = 0
            light_state.pose.position.z = 50
            light_state.diffuse.r = 0.05
            light_state.diffuse.g = 0.05
            light_state.diffuse.b = 0.1
            light_state.diffuse.a = 1.0
            light_state.specular.r = 0.01
            light_state.specular.g = 0.01
            light_state.specular.b = 0.02
            light_state.specular.a = 1.0
        elif mode == "rainy" or mode == "cloudy":
            # Overcast gray lighting
            light_state.pose.position.x = 0
            light_state.pose.position.y = 0
            light_state.pose.position.z = 40
            light_state.diffuse.r = 0.4
            light_state.diffuse.g = 0.4
            light_state.diffuse.b = 0.45
            light_state.diffuse.a = 1.0
            light_state.specular.r = 0.05
            light_state.specular.g = 0.05
            light_state.specular.b = 0.05
            light_state.specular.a = 1.0
        elif mode == "dawn" or mode == "dusk":
            # Reddish horizon lighting
            light_state.pose.position.x = -30
            light_state.pose.position.y = 0
            light_state.pose.position.z = 10
            light_state.diffuse.r = 0.7
            light_state.diffuse.g = 0.4
            light_state.diffuse.b = 0.3
            light_state.diffuse.a = 1.0
            light_state.specular.r = 0.1
            light_state.specular.g = 0.05
            light_state.specular.b = 0.05
            light_state.specular.a = 1.0
        else:
            rospy.logwarn(f"Unknown weather mode: {mode}")
            return
            
        try:
            resp = self.set_light_srv(light_state)
            if resp.success:
                rospy.loginfo(f"Successfully updated Gazebo light states to {mode}")
            else:
                rospy.logwarn(f"Failed to update light state: {resp.status_message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to SetLightState failed: {e}")

if __name__ == '__main__':
    try:
        controller = WeatherController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
