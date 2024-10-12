#! /usr/bin/env python3

import rclpy
from rclpy.node import Node 
from pid_tuning.settings.control_gazebo import ControlGazebo
from pid_tuning.evolutive_algorithms.evolutive_algorithms import HarmonySearch

class TuningHS(Node):
    def __init__(self):
        super().__init__("tuning_hs_node")

        self.A = 3
        self.m = 9
        self.N = 10
        self.Gm = 10000
        self.r_accept = 0.7
        self.r_pa = 0.45
        self.hz = 25

        self.reset_control = ControlGazebo()

        self.hs = HarmonySearch(self.N, self.m, self.Gm, self.A, self.r_accept, self.r_pa, '/home/arne/jazzy_ws/src/scara_robot_description/config/paths.json', epsilon_1=1, tm = 28800)
        self.X = self.hs.gen_population()

        self.timer = self.create_timer(1.0/self.hz, self.timer_callback)

    def timer_callback(self):
        try:
            self.get_logger().warn("Läuft!")
            file = open("best_pid_values_HS.txt", 'w')
            self.hs.evaluate(self.X, self.reset_control, self.hz)
            X_best = self.hs.harmony_search(self.X, self.reset_control, self.hz)
            file.write(str(X_best))
            file.close()
        except Exception as e:
            self.get_logger().warn(f"Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TuningHS()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()