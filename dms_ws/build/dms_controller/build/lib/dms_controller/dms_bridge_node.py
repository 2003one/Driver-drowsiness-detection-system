import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
import requests
import time


# ── configuration ─────────────────────────────────────────────
PI_API_URL      = "http://10.42.0.142:5000/state"
POLL_RATE_HZ    = 1.0      # poll Pi every 1 second
DANGER_TIMEOUT  = 10.0     # seconds before autonomous takeover
FULL_SPEED      = 0.22     # m/s TurtleBot3 max
DROWSY_SPEED    = 0.11     # m/s half speed
PATH_RECORD_HZ  = 1.0      # record position every second

class DMSBridgeNode(Node):

    def __init__(self):
        super().__init__('dms_bridge_node')

        # ── publishers ────────────────────────────────────────
        self.state_pub = self.create_publisher(
            String, '/driver_state', 10)

        self.cmd_pub = self.create_publisher(
            TwistStamped, '/cmd_vel', 10)

        self.teleop_sub = self.create_subscription(
            TwistStamped, '/cmd_vel_teleop',
            self.teleop_callback, 10)

        # ── state tracking ────────────────────────────────────
        self.current_state   = "CALIBRATING"
        self.danger_start    = None
        self.autonomous_mode = False
        self.path_history    = []   # list of PoseStamped waypoints
        self.speed_scale     = 1.0

        

        # ── timer — polls Pi API every second ─────────────────
        self.timer = self.create_timer(
            1.0 / POLL_RATE_HZ,
            self.poll_pi_and_act)

        self.get_logger().info("DMS Bridge Node started")
        self.get_logger().info(f"Polling Pi at: {PI_API_URL}")

    # ── MAIN LOOP — runs every second ─────────────────────────
    def poll_pi_and_act(self):
        # get state from Pi
        try:
            response = requests.get(PI_API_URL, timeout=2.0)
            data     = response.json()
            state    = data.get("state", "CALIBRATING")
            perclos  = data.get("perclos", 0.0)
            votes    = data.get("votes", 0)
        except Exception as e:
            self.get_logger().warn(f"Pi API error: {e}")
            return

        self.current_state = state

        # publish state to ROS2 topic
        msg      = String()
        msg.data = state
        self.state_pub.publish(msg)

        self.get_logger().info(
            f"State: {state} | PERCLOS: {perclos}% | Votes: {votes}")

        # act based on state
        if state == "ACTIVE":
            self.handle_active()

        elif state == "DROWSY":
            self.handle_drowsy()

        elif state == "DANGER":
            self.handle_danger()

    # ── ACTIVE — full teleop speed ─────────────────────────────
    def handle_active(self):
        if self.autonomous_mode:
            self.get_logger().info("Driver awake — resuming teleop")
            self.autonomous_mode = False

        self.danger_start = None
        self.speed_scale  = 1.0
        self.get_logger().info("ACTIVE — full speed")

    # ── DROWSY — half speed ────────────────────────────────────
    def handle_drowsy(self):
        self.danger_start = None
        self.speed_scale  = 0.5
        self.get_logger().info("DROWSY — speed reduced to 50%")

        # publish speed limit warning
        self.publish_speed_limit()

    # ── DANGER — start timer, then autonomous ─────────────────
    def handle_danger(self):
        now = time.time()

        if self.danger_start is None:
            self.danger_start = now
            self.get_logger().warn("DANGER — starting 10s timer")

        elapsed   = now - self.danger_start
        remaining = DANGER_TIMEOUT - elapsed

        # gradually slow down
        self.speed_scale = max(0.0, 1.0 - (elapsed / DANGER_TIMEOUT))

        self.get_logger().warn(
            f"DANGER — stopping in {remaining:.0f}s | speed: {self.speed_scale:.0f}")

        if elapsed >= DANGER_TIMEOUT and not self.autonomous_mode:
            self.autonomous_mode = True
            self.emergency_stop()

    def emergency_stop(self):
        self.get_logger().error("EMERGENCY STOP — driver unresponsive")
        stop = TwistStamped()
        stop.header.stamp = self.get_clock().now().to_msg()
        for _ in range(10):
            self.cmd_pub.publish(stop)
        self.get_logger().error("Robot stopped — waiting for driver")



    def teleop_callback(self, msg):
        if self.autonomous_mode:
            return

        scaled = TwistStamped()
        scaled.header.stamp    = self.get_clock().now().to_msg()
        scaled.header.frame_id = 'base_link'
        scaled.twist.linear.x  = msg.twist.linear.x  * self.speed_scale
        scaled.twist.angular.z = msg.twist.angular.z * self.speed_scale
        self.cmd_pub.publish(scaled)

    # ── publish speed limit on /cmd_vel ───────────────────────
    def publish_speed_limit(self):
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = DMSBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
