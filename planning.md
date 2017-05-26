# Planning #
 This document contains outline and planning for the project.
## Modules ##
### Python ###
Rasberry Pi 
- Vision - `vision`
- Kinematics - `kinematics`
    - Model object - `model.py`
        - Store kinematic model and state information.
        - Read model from a YAML config file. 
        - Allow for rotational and prismatic (linear) joints.
        - Define movement limits.
        - Define home positions.
        - Provide interface for reading current state.
    - Controller - `controller.py`
        - Provide unified IK/FK control.
        - Interface with model object.
        - Provide interface for global (XYZ) coordinate motion. (IK)
        - Provide interface for joint angle motion. (FK)
- Communications - `communication.py`
    - Transmit relevant model information to the teensy on startup
    - Keep model object up to date with regular serial pings 
### Teensy ###
- Read encoder information.
- Interface with motor controllers.
- Handle acceleration parameters
- Handle motion