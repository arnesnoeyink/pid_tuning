#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import ControlWorld
import subprocess

# Das setzen der Physik Parameter funktioniert nicht im Moment nicht.
# Es gibt keine Fehlermeldung, aber die Parameter werden nicht gesetzt.
# Der Workaround ist, in die Welt die Parameter zu schreiben. 
# /gazebo/set_physics_properties

class ControlGazebo(Node):
    def __init__(self):
        super().__init__("control_gazebo_node")
        print("ControlGazebo Init")
        self.world_control_srv = self.create_client(ControlWorld, '/world/empty/control')
        self.world_control_req = ControlWorld.Request()

    def update(self):
        print("ControlGazebo update")
        
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
        subprocess.run(["/home/arne/bash_scripts/restart_gazebo.sh"], shell=True)

def main(args=None):
    rclpy.init(args=args)
    node = ControlGazebo()
    node.restart()
    rclpy.shutdown()

if __name__ == "__main__":
    main()