import os
import rospy
import rosbag
import actionlib

from threading import Thread
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from tactile_msgs.msg import TactileState
from sensor_msgs.msg import ChannelFloat32
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

class KD45Test(Plugin):
    def __init__(self, context):
        super(KD45Test, self).__init__(context)
        self.setObjectName('KD45Test')

        # Create QWidget
        self._widget = QWidget()

        # load widget UI layout
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                               'kd45_test.ui')
        loadUi(ui_file, self._widget)

        self._widget.setObjectName('KD45TestUi')

        # Show _widget.windowTitle on left-top of each plugin
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() +
                                        (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)

        # these bags only contain one trajectory each
        open_bag = rosbag.Bag(os.path.join(os.path.dirname(os.path.realpath(__file__)),
                               '../../resources/steel_open.bag'))
        close_bag = rosbag.Bag(os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                           '../../resources/steel_close.bag'))

        open_traj, close_traj = None, None
        for topic, msg, t in open_bag.read_messages():
            open_traj = msg.goal.trajectory
        for topic, msg, t in close_bag.read_messages():
            close_traj = msg.goal.trajectory
        open_bag.close()
        close_bag.close()

        # in retrospect, this could have been done by creating two points. However, this still allows for recorded trajectories to be replayed
        open_traj.points = [open_traj.points[-1]]
        open_traj.points[0].positions = [0.5, 0.5]
        open_traj.points[0].time_from_start.secs = 2
        open_traj.points[0].time_from_start.nsecs = 0

        close_traj.points = [close_traj.points[-1]]
        close_traj.points[0].positions = [0.0, 0.0]
        close_traj.points[0].time_from_start.secs = 2
        close_traj.points[0].time_from_start.nsecs = 0

        rospy.loginfo("Opening trajectory with {} points and closing trajectory with {} points.".format(len(open_traj.points), len(close_traj.points)))
        self.open_traj = open_traj
        self.close_traj = close_traj

        self.traj_ac = None

        import argparse
        parser = argparse.ArgumentParser()
        parser.add_argument('--publish', dest='publish', action='store_true')
        parser.add_argument('--no-publish', dest='publish', action='store_false')
        parser.set_defaults(publish=False)

        args, _ = parser.parse_known_args(context.argv())

        self._widget.check_pub.setChecked(args.publish)

        # register button signals
        self._widget.btn_close.clicked.connect(self.on_btn_close)
        self._widget.btn_open.clicked.connect(self.on_btn_open)

        self._widget.btn_frc_zero.clicked.connect(self.on_frc_zero)
        self._widget.btn_frc_thresh.clicked.connect(self.on_frc_thresh)
        self._widget.btn_frc_raise.clicked.connect(self.on_frc_raise)

        self._widget.check_pub.stateChanged.connect(self.check_pub_changed)

        self.active = True
        self.publishing_enabled = self._widget.check_pub.isChecked()
        self.pub_thread = Thread(target=self.publish_force)

        self.pub = rospy.Publisher('kd45_tactile', TactileState, queue_size=10)
        self.pub_rate = rospy.Rate(50)

        self.current_force = 0.0
        self.force_raise = False

        self.force_thresh = 1.5
        self.force_max = 3.0
        self.force_step = 0.01

        self.pub_thread.start()

    def on_frc_zero(self):
        self.force_raise = False
        self.current_force = 0.0

    def on_frc_thresh(self):
        self.force_raise = False
        self.current_force = self.force_thresh

    def on_frc_raise(self):
        self.force_raise = True
        self.current_force = self.force_thresh

    def check_pub_changed(self, int):
        self.publishing_enabled = not self.publishing_enabled

    def publish_force(self):
        while self.active:
            if self.publishing_enabled:
                if self.force_raise:
                    self.current_force = min(self.current_force+self.force_step, self.force_max)

                s0 = ChannelFloat32()
                s0.name = "right_gripper_tactile"
                s0.values = [self.current_force]

                s1 = ChannelFloat32()
                s1.name = "left_gripper_tactile"
                s1.values = [self.current_force]

                ts = TactileState()
                ts.header.frame_id = "/base_link"
                ts.header.stamp = rospy.Time.now()

                ts.sensors.append(s0)
                ts.sensors.append(s1)

                self.pub.publish(ts)
                self.pub_rate.sleep()

    def on_btn_close(self):
        rospy.loginfo("Closing Gripper")
        self.send_traj('close')

    def on_btn_open(self):
        rospy.loginfo("Opening Gripper")
        self.send_traj('open')

    def send_traj(self, traj_name):
        if not self.traj_ac:
            self.init_ac()

        g = FollowJointTrajectoryGoal()
        if traj_name == 'open':
            g.trajectory = self.open_traj
        elif traj_name == 'close':
            g.trajectory = self.close_traj

        self.traj_ac.send_goal(g)

    def init_ac(self):
        self.traj_ac = actionlib.ActionClient('/gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

        rospy.loginfo("Waiting for gripper_controller ...")
        self.traj_ac.wait_for_server()
        rospy.loginfo("Ready!")

    def shutdown_plugin(self):
        rospy.loginfo("KD45Test plugin shutting down ...")
        self.active = False
        self.pub_thread.join()
        rospy.loginfo("KD45Test plugin exited.")

    def save_settings(self, plugin_settings, instance_settings):
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # v = instance_settings.value(k)
        pass
