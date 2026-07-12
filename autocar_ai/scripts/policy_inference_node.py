#!/usr/bin/env python3
import os
import numpy as np
import torch
import torch.nn as nn
import rospy
import rospkg
from autocar_interfaces.srv import GetRLAction, GetRLActionResponse
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String

class PolicyNetwork(nn.Module):
    def __init__(self, state_dim=12, action_dim=9):
        super(PolicyNetwork, self).__init__()
        self.actor = nn.Sequential(
            nn.Linear(state_dim, 64),
            nn.Tanh(),
            nn.Linear(64, 64),
            nn.Tanh(),
            nn.Linear(64, action_dim)
        )

    def forward(self, state):
        return self.actor(state)

class PolicyInferenceNode:
    def __init__(self):
        rospy.init_node('policy_inference_node')
        
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('autocar_ai')
        
        # Support loading specific model versions via parameter
        model_name = rospy.get_param('~policy_model_name', 'ppo_policy.pth')
        self.model_path = os.path.join(pkg_path, 'models', model_name)
        
        # Initialize network
        self.state_dim = 12
        self.action_dim = 9
        self.policy = PolicyNetwork(self.state_dim, self.action_dim)
        
        # Load weights if available, otherwise save initialization for fallback
        if os.path.exists(self.model_path):
            self.policy.load_state_dict(torch.load(self.model_path), strict=False)
            rospy.loginfo(f"[Policy Server] Loaded weights from {self.model_path}")
        else:
            rospy.logwarn(f"[Policy Server] Weights not found at {self.model_path}. Using initialized random policy.")
            os.makedirs(os.path.dirname(self.model_path), exist_ok=True)
            torch.save(self.policy.state_dict(), self.model_path)
            
        self.policy.eval()

        # Advertise service and status publisher
        self.pub_status = rospy.Publisher('/policy_status', String, queue_size=10)
        self.srv_reload = rospy.Service('/reload_policy', Trigger, self.handle_reload_policy)
        self.srv = rospy.Service('/get_rl_action', GetRLAction, self.handle_get_rl_action)
        rospy.loginfo("[Policy Server] Service /get_rl_action and /reload_policy are ready.")

    def handle_reload_policy(self, req):
        res = TriggerResponse()
        if os.path.exists(self.model_path):
            try:
                self.policy.load_state_dict(torch.load(self.model_path), strict=False)
                self.policy.eval()
                
                status_msg = String()
                status_msg.data = f"RELOADED:{os.path.basename(self.model_path)}"
                self.pub_status.publish(status_msg)
                
                res.success = True
                res.message = f"Successfully reloaded model from {self.model_path}"
                rospy.loginfo(f"[Policy Server] {res.message}")
            except Exception as e:
                res.success = False
                res.message = f"Failed to reload model weights: {e}"
                rospy.logerr(f"[Policy Server] {res.message}")
        else:
            res.success = False
            res.message = f"Model file not found at {self.model_path}"
            rospy.logerr(f"[Policy Server] {res.message}")
        return res

    def handle_get_rl_action(self, req):
        state_tensor = torch.tensor(req.state_vector, dtype=torch.float32).unsqueeze(0)
        
        with torch.no_grad():
            logits = self.policy(state_tensor)
            probs = torch.softmax(logits, dim=-1)
            
            # Selected action
            action = torch.argmax(probs, dim=-1).item()
            
            # Policy confidence is the probability of the chosen action
            confidence = probs[0, action].item()
            
            # Alternative actions sorted by probability descending (excluding best)
            sorted_indices = torch.argsort(probs, dim=-1, descending=True).squeeze().tolist()
            alternative_actions = sorted_indices[1:]

        response = GetRLActionResponse()
        response.action = action
        response.confidence = confidence
        response.alternative_actions = [int(a) for a in alternative_actions]
        
        return response

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    node = PolicyInferenceNode()
    node.spin()
