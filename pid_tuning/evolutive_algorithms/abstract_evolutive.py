#!/usr/bin/env python3
from .evolutive_interface import *
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


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
        self.dic_cmds = {}
        self.dic_cli = {}
        self.pubs = {}
        self.jointstate_sub = self.create_subscription(JointState, "/joint_states", self.error_callback, 0)
        self.g1_x = 0
        self.g1_y = 0
        self.g1_z = 0
        self.x_o = 0
        self.y_o = 0
        self.z_o = 0
        self.epsilon_1 = epsilon_1

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

    def set_paths(self):
        #TODO: Beschreibung überarbeiten
        """ 
            Arguments:
                None\n
            Definition:
                Creates dictionaries to store the command and state
                topics, and also a dictionary for the dynamic reconfigure clients.\n                
            Returns:
                None\n
        """
        cmd_paths = self.data["command_paths"] 
        cl_paths = self.data["client_paths"]

        for i, cmd, st, cl in zip(range(1, len(cmd_paths)+1), cmd_paths, cl_paths):
            self.dic_cmds[f"command{i}"] = cmd
            self.dic_cli[f"client{i}"] = cl

    def get_paths(self):
        """
            Arguments:
                None\n
            Definition:
                Returns the dictionaries of the paths for command and clients.\n
            Returns:
                (tuple): dictionaries\n
        """
        return self.dic_cmds, self.dic_cli

    def set_publisher(self):
        """ 
            Arguments:
                None\n
            Description:
                Initializes and instantiates Publisher and Subscriber objects.\n 
            Returns:
                None\n
        """
        for i in range(1, self.A+1):
            self.pubs[f"pub{i}"] = self.create_publisher(Float64, self.dic_cmds[f"command{i}"], 10)  # TODO: Maybe change message type 
        self.pub = self.create_publisher(EvolutiveInfo, 'generations_info', 10)

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
     
    # TODO: Change JointControllerState to other message type 
    def error_callback(self, data: Float64, args: int):
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
        self.errors[args] += abs(data.error)
        return self.errors[args]

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

            for i in range(self.A):                
                params = {'p' : p[i*3], 'i' : p[i*3+1], 'd' : p[i*3+2]}
                client = self.dic_cli[f"client{i}"]
                # future = client.call_async(UpdatePIDParams.Request(**params))   # TODO: Das habe ich noch nicht verstanden
                # rclpy.spin_until_future_complete(self, future)
                self.errors[i] = 0.0

            self.g1_x = 0.0
            self.g1_y = 0.0
            self.g1_z = 0.0

            reset_control.unpause() # Unpause simulation

            # publish each element of the trajectories
            length = len(self.trajectories) 
            for i in range(length):
                
                for j in range(self.A):
                    traj_msg = Float64()
                    traj_msg.data = self.trajectories[f"q{j+1}"][i]
                    self.pubs[f"pub{j+1}"].publish(traj_msg)
                time.sleep(period)  

                self.g1_x += abs(self.x_d[i] - self.x_o)
                self.g1_y += abs(self.y_d[i] - self.y_o)
                self.g1_z += abs(self.z_d[i] - self.z_o)
 
                
            for i in range(self.A):
                stop_msg = Float64()
                stop_msg.data = 0.0
                self.pubs[f"pub{i+1}"].publish(stop_msg)
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
            
            self.pub.publish(self.info)

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