#!/usr/bin/env python3
import rclpy
import os
import json
import time
from rclpy.node import Node
from pid_tuning.evolutive_algorithms.dif_evolution import DifferentialEvolution
from pid_tuning.settings.control_gazebo import ControlGazebo
from ament_index_python.packages import get_package_share_directory

class TuningDE(Node):
    def __init__(self):
        super().__init__("tuning_node_de")
        
        self.A = 3
        self.m = 9
        self.N = 10
        self.Gm = 10000
        self.F = 0.65
        self.C = 0.85
        self.hz = 25
        self.reset_control = ControlGazebo()

        self.de = DifferentialEvolution(self.N, self.m, self.Gm, self.F, self.C, self.A, '/home/arne/jazzy_ws/install/scara_robot_description/share/scara_robot_description/config/paths.json', epsilon_1=1, tm=28800)
        self.X = self.de.gen_population()

        self.timer = self.create_timer(1.0/self.hz, self.timer_callback)

        self.iter = 0
        pid_tuning_dir = get_package_share_directory('pid_tuning')
        self.file_name = 'best_pid_values_DE_' + time.strftime("%Y%m%d-%H%M%S") + '.txt'
        self.file_path = os.path.join(pid_tuning_dir, 'results', self.file_name)
        file = open(self.file_path, 'w')
        file.close()

    def timer_callback(self):
        try:
            self.de.evaluate(self.X, self.reset_control, self.hz)
            X_best = self.de.dif_evolution(self.X, self.reset_control, self.hz)
            X_best = X_best.tolist()
            with open(self.file_path, 'a') as file:
                file.write(f"Iteration of best parameter: {self.iter}" + '\n')
                file.write(json.dumps(X_best) + '\n')
            self.iter += 1
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TuningDE()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()