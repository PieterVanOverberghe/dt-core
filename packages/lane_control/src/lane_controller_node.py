#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import Twist2DStamped, LanePose, StopLineReading

class SimpleLaneControllerNode(DTROS):
    def __init__(self, node_name):
        super(SimpleLaneControllerNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.CONTROL
        )

        # --- PID waarden (Nu met I!) ---
        self.v_forward = 0.05

        # P-Gains (Reageren op de huidige fout)
        self.k_d = -1.5
        self.k_phi = -1.0

        # I-Gains (Reageren op structurele afwijkingen/scheve hardware)
        # Begin met hele kleine getallen!
        self.k_i_d = -0.2
        self.k_i_phi = -0.2

        # --- State Variables ---
        self.state = "LANE_FOLLOWING"
        self.ignore_stop_line_until = rospy.Time.now()
        self.cross_end_time = rospy.Time.now()

        # --- Integrale Geheugen Variabelen ---
        self.integral_d = 0.0
        self.integral_phi = 0.0
        self.last_time = None

        # Snelheden voor het oversteken
        self.cross_v = 0.0
        self.cross_omega = 0.0

        # --- Publishers ---
        self.pub_car_cmd = rospy.Publisher(
            "~car_cmd", Twist2DStamped, queue_size=1, dt_topic_type=TopicType.CONTROL
        )

        # --- Subscribers ---
        self.sub_lane_pose = rospy.Subscriber("~lane_pose", LanePose, self.cb_lane_pose, queue_size=1)

        veh_namespace = rospy.get_namespace()
        stop_line_topic = f"{veh_namespace}stop_line_filter_node/stop_line_reading"
        self.sub_stop_line = rospy.Subscriber(stop_line_topic, StopLineReading, self.cb_stop_line, queue_size=1)
        self.sub_cmd = rospy.Subscriber("~command", String, self.cb_command, queue_size=1)

        self.log("Simple PI-Controller Initialized! Ready to learn and follow.")

    def cb_stop_line(self, msg):
        if msg.at_stop_line and self.state == "LANE_FOLLOWING":
            if rospy.Time.now() > self.ignore_stop_line_until:
                self.log("Red line detected! Stopping...")
                self.state = "STOPPED"
                self.stop_robot()

    def cb_command(self, msg):
        command = msg.data.lower()
        if self.state == "STOPPED" and command in ["straight", "left", "right"]:
            if command == "straight":
                self.cross_v = 0.04
                self.cross_omega = 0.0
                cross_duration = 2.0
            elif command == "left":
                self.cross_v = 0.04
                self.cross_omega = 1.0
                cross_duration = 2.0
            elif command == "right":
                self.cross_v = 0.04
                self.cross_omega = -1.0
                cross_duration = 2.0

            self.state = "CROSSING"
            self.cross_end_time = rospy.Time.now() + rospy.Duration(cross_duration)
            self.ignore_stop_line_until = rospy.Time.now() + rospy.Duration(cross_duration + 2.0)

            # Reset de integralen bij een nieuw kruispunt, zodat hij met een schone lei begint
            self.integral_d = 0.0
            self.integral_phi = 0.0

    def cb_lane_pose(self, msg):
        current_time = rospy.Time.now()

        if self.last_time is None:
            self.last_time = current_time
            return

        # Bereken tijdsverschil (dt) voor de integraal
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        if self.state == "STOPPED":
            self.stop_robot()
            return

        if self.state == "CROSSING":
            if rospy.Time.now() < self.cross_end_time:
                car_cmd = Twist2DStamped()
                car_cmd.header = msg.header
                car_cmd.v = self.cross_v
                car_cmd.omega = self.cross_omega
                self.pub_car_cmd.publish(car_cmd)
                return
            else:
                self.state = "LANE_FOLLOWING"

        # --- PI Controller Wiskunde ---
        d_err = msg.d
        phi_err = msg.phi

        # Bouw het geheugen (integraal) op
        self.integral_d += d_err * dt
        self.integral_phi += phi_err * dt

        # Anti-Windup: Zorg dat het geheugen niet oneindig groot wordt als je hem oppakt
        max_integral = 1.0
        self.integral_d = max(min(self.integral_d, max_integral), -max_integral)
        self.integral_phi = max(min(self.integral_phi, max_integral), -max_integral)

        # Bereken de uiteindelijke stuurhoek met P en I
        omega_p = (self.k_d * d_err) + (self.k_phi * phi_err)
        omega_i = (self.k_i_d * self.integral_d) + (self.k_i_phi * self.integral_phi)

        omega = omega_p + omega_i

        car_cmd = Twist2DStamped()
        car_cmd.header = msg.header
        car_cmd.v = self.v_forward
        car_cmd.omega = omega

        self.pub_car_cmd.publish(car_cmd)

    def stop_robot(self):
        car_cmd = Twist2DStamped()
        car_cmd.v = 0.0
        car_cmd.omega = 0.0
        self.pub_car_cmd.publish(car_cmd)

if __name__ == "__main__":
    node = SimpleLaneControllerNode("lane_controller_node")
    rospy.spin()