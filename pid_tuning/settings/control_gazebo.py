#!/usr/bin/env python3
import rclpy
import os
import subprocess
from rclpy.node import Node
from ros_gz_interfaces.srv import ControlWorld
from ament_index_python.packages import get_package_share_directory

class ControlGazebo(Node):
    def __init__(self):
        super().__init__("control_gazebo_node")
        self.world_control_srv = self.create_client(ControlWorld, '/world/empty/control')
        self.world_control_req = ControlWorld.Request()
        
    def pause(self):
        print("ControlGazebo Pause")
        self.world_control_req.world_control.pause = True 
        while not self.world_control_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Pause service not available, waiting again...')
        resp = self.world_control_srv.call_async(self.world_control_req)
        rclpy.spin_until_future_complete(self, resp)
        
    def unpause(self):
        print("ControlGazebo Unpause")
        self.world_control_req.world_control.pause = False 
        while not self.world_control_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Unpause service not available, waiting again...')
        resp = self.world_control_srv.call_async(self.world_control_req)
        rclpy.spin_until_future_complete(self, resp)
    
    def restart(self):
        pid_tuning_dir = get_package_share_directory('pid_tuning')
        bash_file = os.path.join(pid_tuning_dir, 'bash_scripts/restart_gazebo.sh')
        subprocess.run([bash_file], shell=True)

def main(args=None):
    rclpy.init(args=args)
    node = ControlGazebo()
    node.restart()
    rclpy.shutdown()

if __name__ == "__main__":
    main()