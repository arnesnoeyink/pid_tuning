#!/usr/bin/env python3
from .evolutive_interface import *
from rclpy.node import Node
from std_msgs.msg import Float64
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import ParameterValue, Parameter
from control_msgs.msg import MultiDOFCommand, MultiDOFStateStamped

class AbstractEvolutive(Node, EvolutiveInterface):
    def __init__(self, N: int, m: int, Gm : int, A : int, epsilon_1 = 0.10):
        super().__init__('abstract_evolutive_node')
        EvolutiveInterface().__init__(self)
        self.N = N
        self.m = m
        self.Gm = Gm
        self.a = np.zeros((1, self.m)) 
        self.b = np.ones((1, self.m))
        self.A = A
        self.errors = np.zeros(self.A)
        self.reference_pub = self.create_publisher(MultiDOFCommand, "/pid_controller/reference", 10)
        self.generations_info_pub = self.create_publisher(EvolutiveInfo, 'generations_info', 10)
        self.update_pid_srv = self.create_client(SetParameters,"/pid_controller/set_parameters") 
        self.controller_state_sub = self.create_subscription(MultiDOFStateStamped, "/pid_controller/controller_state", self.error_callback, 0)
        self.g1_x = 0
        self.g1_y = 0
        self.g1_z = 0
        self.x_o = 0
        self.y_o = 0
        self.z_o = 0
        self.epsilon_1 = epsilon_1
        self.dof_names = ["base_link_link_01", "link_01_link_02", "link_02_link_03"]

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.fb_callback, 10)
        self.info = EvolutiveInfo()
    
    def read_json(self, file_path: str):
        """ 
            Arguments:
                @file_path = (String) json path file\n
            Definition: 
                Reads json file with paths (trajectories [csv], command and state 
                topics, client), PID bounds (lower and upper) and stores in class 
                attribute.\n
            Returns: 
                None\n
        """
        with open(file_path, 'r') as file:
            self.data = json.load(file) 

    def get_trajectories(self):
        """
            Arguments:
                None\n
            Description:
                Reads csv files containing trajectories of joints and/or floating base.\n 
            Returns:
                None\n        
        """
        columns = self.data['trajectories']['column_names']
        self.trajectories = pd.read_csv(self.data['trajectories']['path'], usecols=columns)

        try:
            fb_columns = self.data['trajectories']['fb_column_names']
            self.fb_trajectory = pd.read_csv(self.data['trajectories']['floating_base_path'], usecols=fb_columns)
            self.x_d = self.fb_trajectory['X']
            self.y_d = self.fb_trajectory['Y']
            self.z_d = self.fb_trajectory['Z']
        except Exception as e:
            self.get_logger().error(f"Exception: {e}")
            # vector x_d size len(self.trajectories)
            size = len(self.trajectories)
            self.x_d = np.zeros(size)
            self.y_d = np.zeros(size)
            self.z_d = np.zeros(size)
            pass

    def bounds(self):
        """
            Arguments:
                None\n
            Definition:
                Establishes minimum and maximum limit values for each design variable 
                (PID gains for all the controllers) and stores in class attribute.\n
            Returns:
                None\n
        """
        pid_bounds = self.data['pid_bounds']
        p_min, p_max = pid_bounds['p']
        i_min, i_max = pid_bounds['i']
        d_min, d_max = pid_bounds['d']

        cnt = 0
        for i in range(self.m):
            if cnt == 0:
                self.a[0][i] += p_min
                self.b[0][i] = p_max*self.b[0][i]
            if cnt == 1:
                self.a[0][i] += i_min
                self.b[0][i] = i_max*self.b[0][i]     #10   
            if cnt == 2:
                self.a[0][i] += d_min
                self.b[0][i] = d_max*self.b[0][i]   #2 
            cnt = (cnt + 1) % 3 

    def gen_population(self):
        """
            Arguments:
                None\n
            Definition:
                Generates initial random population.\n
            Returns: 
                np.array: returning value\n
        """
        X = np.zeros((self.N,self.m+2))
        self.bounds()
        for i in range(self.N):
            for j in range(self.m):
                X[i][j] = (self.b[0][j]- self.a[0][j]) * np.random.random_sample() - self.a[0][j]
        return X   
     
    def error_callback(self, data: MultiDOFStateStamped):
        """
            Arguments:
                @data = (JointControllerState) current information of the State\n
                @args = (int) number of joint\n
            Definition:
                Reads data obtained by the subscriber and stores the absolute
                error for each joint in an array of size A, with A being the 
                number of joints of the robot.\n
            Returns:
                Float64: absolute error of joint "args"\n
        """
        # Calculate error for each joint with: error = setpoint - state
        for i in range(self.A):
            self.errors[i] += abs(data.dof_states[i].error)

        return self.errors

    def fb_callback(self, data: Odometry):
        """
            Arguments:
                @data = (Odometry) current information of the floating base Odometry\n
            Definition:
                Gets odometry data in order to calculate the cartesian error required
                for the equality constrained method.\n
            Returns:
                None\n        
        """
        # global g1_x, g1_y, g1_z
        self.x_o = data.pose.pose.position.x
        self.y_o = data.pose.pose.position.y
        self.z_o = data.pose.pose.position.z

    def evaluate(self, P: np.ndarray, reset_control: ControlGazebo, hz: float):
        """ Arguments:
                @P = (np.array) population matrix\n
                @reset_control = (ControlGazebo) object\n
                @hz = float object (frequency)\n
            Definition:
                Evaluates population updating PID gains (in rosparam) and sends
                joint trajectories in order to get each tracking error.\n
            Returns:
                None\n
        """
        period = 1.0 / hz

        for w,p in zip(range(self.N), P):
            reset_control.pause() # Pause simulation

            # TODO: Verallgemeinern mithilfe einer for Schleife
            # Joint0
            p_0 = p[0]
            i_0 = p[1]
            d_0 = p[2]
            req_0 = SetParameters.Request()
            parameters_to_set_0 = [
                {
                    'name': 'gains.base_link_link_01.p',
                    'type': 3,
                    'value': p_0
                },
                {
                    'name': 'gains.base_link_link_01.i',
                    'type': 3,
                    'value': i_0
                },
                {
                    'name': 'gains.base_link_link_01.d',
                    'type': 3,
                    'value': d_0
                }
            ]

            for param_data in parameters_to_set_0:
                param = Parameter()
                param.name = param_data['name']

                value = ParameterValue()
                value.type = param_data['type']
                value.double_value = param_data['value']

                param.value = value
                req_0.parameters.append(param)

            while not self.update_pid_srv.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('update pid values service not available, waiting again...')
            resp_0 = self.update_pid_srv.call_async(req_0)
            rclpy.spin_until_future_complete(self, resp_0)
            self.get_logger().info('update pid values successfull')
            self.errors[0] = 0.0

            # Joint1
            p_1 = p[3]
            i_1 = p[4]
            d_1 = p[5]
            req_1 = SetParameters.Request()
            parameters_to_set_1 = [
                {
                    'name': 'gains.link_01_link_02.p',
                    'type': 3,
                    'value': p_1
                },
                {
                    'name': 'gains.link_01_link_02.i',
                    'type': 3,
                    'value': i_1
                },
                {
                    'name': 'gains.link_01_link_02.d',
                    'type': 3,
                    'value': d_1
                }
            ]

            for param_data in parameters_to_set_1:
                param = Parameter()
                param.name = param_data['name']

                value = ParameterValue()
                value.type = param_data['type']
                value.double_value = param_data['value']

                param.value = value
                req_1.parameters.append(param)

            while not self.update_pid_srv.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('update pid values service not available, waiting again...')
            resp_1 = self.update_pid_srv.call_async(req_1)
            rclpy.spin_until_future_complete(self, resp_1)
            self.get_logger().info('update pid values successfull')
            self.errors[1] = 0.0

            # Joint2
            p_2 = p[6]
            i_2 = p[7]
            d_2 = p[8]
            req_2 = SetParameters.Request()
            parameters_to_set_2 = [
                {
                    'name': 'gains.link_02_link_03.p',
                    'type': 3,
                    'value': p_2
                },
                {
                    'name': 'gains.link_02_link_03.i',
                    'type': 3,
                    'value': i_2
                },
                {
                    'name': 'gains.link_02_link_03.d',
                    'type': 3,
                    'value': d_2
                }
            ]

            for param_data in parameters_to_set_2:
                param = Parameter()
                param.name = param_data['name']

                value = ParameterValue()
                value.type = param_data['type']
                value.double_value = param_data['value']

                param.value = value
                req_2.parameters.append(param)

            while not self.update_pid_srv.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('update pid values service not available, waiting again...')
            resp_2 = self.update_pid_srv.call_async(req_2)
            rclpy.spin_until_future_complete(self, resp_2)
            self.get_logger().info('update pid values successfull')
            self.errors[2] = 0.0

            self.g1_x = 0.0
            self.g1_y = 0.0
            self.g1_z = 0.0

            reset_control.unpause() # Unpause simulation

            # publish each element of the trajectories
            length = len(self.trajectories) 
            for i in range(length):
                traj_msg = MultiDOFCommand()
                traj_msg.dof_names = self.dof_names
                for j in range(self.A):
                    traj_msg.values[j] = self.trajectories[f"q{j+1}"][i]
                    traj_msg.values_dot[j] = 0.0
                self.reference_pub.publish(traj_msg)
                time.sleep(period)  

                self.g1_x += abs(self.x_d[i] - self.x_o)
                self.g1_y += abs(self.y_d[i] - self.y_o)
                self.g1_z += abs(self.z_d[i] - self.z_o)
 
            stop_msg = MultiDOFCommand()
            stop_msg.dof_names = self.dof_names 
            for i in range(self.A):
                stop_msg.values[i] = 0.0
                stop_msg.values_dot[j] = 0.0
            self.reference_pub.publish(stop_msg)
            time.sleep(period)

            reset_control.restart() # Restart simulation
            reset_control.pause() # Pause simulation

            P[w][self.m] = sum(self.errors)

            P[w][self.m+1] = self.scv() # Calculate SCV
            
            self.info.header.stamp = self.get_clock().now().to_msg()
            self.info.header.frame_id = "base_link"
            self.info.individual = w
            self.info.genes = P[w, 0:-2]
            self.info.of = P[w][-2]
            self.info.scv = P[w][-1]
            
            self.generations_info_pub.publish(self.info)

    def scv(self):
        """ Arguments:
                None\n
            Definition:
                Penality function used to obtain the sum of constraint violation\n
            Returns:
                Float64: scv value\n
        """
        self.f = 0
        vvr = 0

        g1 = self.g1_x + self.g1_y + self.g1_z
        
        vvr = abs(g1) - self.epsilon_1
        if vvr > 0:
            self.f = self.f + vvr

        return self.f

    def deb(self, u: np.ndarray, x: np.ndarray):
        """ Arguments:
                @u = (np.array) son design variable vector 
                including fitness function and scv values\n
                @x = (np.array) father design variable vector 
                including fitness function and scv values\n
            Definition:
                Uses feasibles Deb rules as selection criteria.\n
                Deb Rules:\n
                \t + Between a feasible and an infeasible vector, the
                feasible solution is choosen.
                \t + Between two infeasibles vectors, one with the lowest
                sum of constraint violations is preferred.
                \t + Between two feasible vectors the one with the 
                best value of the objective function is selected.
            Returns:
                bool: flag that indicates replacement\n
                0 : keep father\n
                1 : replace father with son\n
        """
        # Between a feasible and an infeasible vector, the
        # feasible solution is choosen.
        if u[-1] <= 0 and x[-1] > 0:
            return 1      
        # Between two infeasibles vectors, one with the lowest
        # sum of constraint violations is preferred.
        if u[-1] > 0 and x[-1] > 0:
            if u[-1] <= x[-1]:
                return 1
        else:
        # between two feasible vectors the one with the best value of
        # the objective function is selected
            if u[-1] <= 0 and x[-1] <= 0:
                if u[-2] < x[-2]:
                    return 1
        return 0

    def deb_bubble_sort(self, P: np.ndarray):
        """ Arguments:
                @P = population matrix\n
            Description:
                Sorts harmony memory using BubbleSort with Deb's rules as 
                sorting method and criteria respectively.\n
            Return:
                np.array: Sorted harmony memory\n
        """
        n = len(P)
        for i in range(n-1): # n = 3 => n-1 = 2
            for j in range(n-1):
                x = np.array([P[j][-2], P[j][-1]]) #parent
                u = np.array([P[j+1][-2], P[j+1][-1]]) #son

                if (self.deb(u,x) == 1):
                    copy = P[j, :].copy()
                    P[j, :] = P[j+1, :]
                    P[j+1, :] = copy

        return P