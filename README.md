# KD45 Controller Test RQT Plugin

Small GUI that allows sending opening and closing trajectories to the KD45 gripper controller.
Also publishes artificial force data of different kinds:

* zeros
* values v with trigger_threshold < v < max_force
* linear interpolation from trigger_threshold to max_force
