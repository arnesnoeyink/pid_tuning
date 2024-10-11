#!/usr/bin/env python3
# Interface of the Evolutive Methods
from pid_tuning.settings.control_gazebo import *
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import numpy as np
import pandas as pd
from random import choice, randint, random
import json
import time
from evolution_interfaces.msg import EvolutiveInfo

class EvolutiveInterface:
    def __init__(self) -> None:
        pass

    def gen_population(self):
        pass

    def read_json(self):
        pass

    def bounds(self):
        pass

    def error_callback(self):
        pass

    def evaluate(self):
        pass
    
    def get_trajectories(self):
        pass

    def scv(self):
        pass

    def fb_callback(self):
        pass

    def deb(self):
        pass