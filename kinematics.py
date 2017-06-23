"""
The Kinematics module contains everything related to the robot's motion

"""

from math import sin, cos
import numpy as np
import yaml


class Kinematics(object):


    """Kinematics contains everything related to the robot's kinematics model.
    
    This is the interface for all motion, including forward and inverse kinematics. The object
    stores a kinematic chain, represented as a series of KinematicLink objects.
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
        self.kinematic_chain = []
        super(Kinematics, self).__init__()

    def load_config(self, cfg):
        """Load a configuration YAML file.
        
        cfg: A file object in read mode pointing to a configuration YAML file.
        """

        data = yaml.load(cfg)
        joints = data['robot']['joints']
        for j in joints:
            newlink = KinematicLink(j)
            self.kinematic_chain.append(newlink)
            #TODO: update worldspace matrix
        self.num_links = len(self.kinematic_chain)

    def connect_serial(self, comms):
        """Attach a serial port"""
        #TODO: implement
        self.comms = comms

    def get_link_transform(self, end=-1):
        """Get the full transformation matrix of the chain up to joint 'end'."""

        trans = np.identity(4)
        for jnt in self.kinematic_chain[:end]:
            trans.anp.dot(trans[-1], jnt.get_full_matrix())
        return trans
    def calc_jacobian(self):
        """Calculate the Jacobian matrix for the current position of the robot

        The Jacobian is a 3xn matrix where n is the number of links in the chain. Each column is the
        partial derivative of the position of the end effector with respect to the degree of freedom
        of the link.
        """
        jac = np.empty((3, len(self.num_links)))
        end_pos = self.kinematic_chain[-1].ws_matrix[:3, 3]
        for count, jnt in enumerate(self.kinematic_chain):
            ws_trans = jnt.ws_matrix 
            axis = ws_trans[:3, 2]
            assert sum([x**2 for x  in axis]) == 1 #The axis is normalized
            ws_pos = ws_trans[:3, 3]
            if jnt.type == "revolute":
                #For revolute joints, ds/dtheta = axis x (s-p_j)
                jac[:, count] = np.cross(axis, (end_pos-ws_pos))
            elif jnt.type == "prismatic":
                #For prismatic joints, ds/dtheta = axis
                jac[:, count] = axis
            else:
                jac[:, count] = [0, 0, 0]
        return jac
    def update_worldspace(self):
        """Update the worldspace matrices of the kinematic chain from the """
class KinematicLink(object):


    """The KinematicLink class is a parent class for different types of linkages
    
    The primary kinematic description is via the four Denavit-Hartenberg parameters.
    """
    def __init__(self, joint_cfg):
        """Initialize the link object.
        
        Takes the dictionary object direct from the YAML file.
        """
        super(KinematicLink, self).__init__()
        #extract params from YAML
        self.dh_d = joint_cfg['d']
        self.dh_theta = joint_cfg['theta']
        self.dh_r = joint_cfg['r']
        self.dh_alpha = joint_cfg['alpha']
        self.type = joint_cfg['joint_type']
        #set rest states
        self.dh_d_rest = self.dh_d
        self.dh_theta_rest = self.dh_theta
        #create worldspace matrix
        self.ws_matrix = np.identity(4)

    def reset(self):
        """Reset the joint's state to the rest positions"""

        self.dh_theta = self.dh_theta_rest
        self.dh_d = self.dh_d_rest

    def get_full_matrix(self):
        """Get the 4x4 full transformation matrix"""

        _t = self.get_translation_matrix()
        _r = self.get_rotation_matrix()
        trans = np.concatenate((_t, _r), 1)
        trans = np.concatenate((trans, np.array([[0, 0, 0, 1]])))
        return trans

    def get_translation_matrix(self):
        """Get the 3x1 translation matrix"""

        return np.array([[self.dh_r*cos(self.dh_theta), self.dh_r*sin(self.dh_theta), self.dh_d]]).T
 
    def get_rotation_matrix(self):
        """Get the 3x3 rotation matrix"""
        
        cth = cos(self.dh_theta)
        sth = sin(self.dh_theta)
        cal = cos(self.dh_alpha)
        sal = sin(self.dh_alpha)
        return np.array([[cth, -sth*cal, sth*sal],
                         [sth, cth*cal, -cth*sal],
                         [0, sal, cal]
                        ])
