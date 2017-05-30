

from timeit import default_timer as timer
from random import random
import numpy as np
import yaml


class Kinematics(object):
    """Kinematcis contains everything related to the robot's kinematics model.
    
    This is the interface for all motion, including forward and inverse kinematics.
    """

    def __init__(self, cfg=None, comms=None):
        """Create new Kinematics object.

        Note that this by default, this method signature does not initialize the object. This is to
        allow for deferred loading of config files and connection of the serial port.

        Keyword Arguments:
        cfg: A file object in read mode pointing to a configuration YAML file.
        comms: A scorbot-cv.Communications object
        """ 
        if cfg is not None:
            self.load_config(cfg)
        if comms is not None:
            self.connect_serial(comms)
        super(Kinematics, self).__init__()

    def load_config(self, cfg):
        """Load a configuration YAML file.
        
        cfg: A file object in read mode pointing to a configuration YAML file.
        """
        #TODO: implement.
        data = yaml.load(cfg)

    def connect_serial(self, comms):
        """Attach a serial port"""
        self.comms = comms