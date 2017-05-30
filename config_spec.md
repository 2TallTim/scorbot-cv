# Config Spec #

The configuration file is a YAML file. It contains one root object, `robot`. The `robot` object encapsulates all of the information about the robot and its kinematic structure.
The `robot` object contains a list object `joints` that describes the joints of the robot. The joints should be listed in order from the base up. Each joint at a minimum must have the `joint_type` attribute, and the four Denavit-Hartenberg parameters: `d`, `theta`, `r` and `alpha`.

`joint_type` can take on any of the following values: 
- `prismatic`: Prismatic joints are linear slide joints. They allow for linear motion along a single axis.
- `revolute`: Revolute joints are rotational joints. They allow for rotation about a single axis.

The DH parameters define the coordinate frames of the robot as follows:
- `d`: The distance along the previous coordinate frame's _Z_ axis.
- `theta`: A rotation about the previous coordinate frame's _Z_ axis to align the new _X_ axis.
- `r`: The length of the common normal, i.e. the radius of a rotational joint
- `alpha`: Angle about the common normal, from old _Z_ axis to new _Z_ axis.

Optionally, the configuration file may also specify a unique `name` for each joint for convenience access.

TODO: Expand this section to include hardware descriptions including movement limits, acceleration, velocity, motor ports, etc. 