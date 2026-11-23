"""
This node spawns a target point where I can move around so I can check (once the inverse differential equations
are in place), the arms moves accordinately to point at the target. 

The target will eventually be published from the head vision node that detect, tracks and predict the drone's movement

Note to self: still gotta account for airsoft gun pellet velocity to ensure it actually hits the drone.
"""

import rclpy
from rclpy.node import Node

# message imports
from std_msgs.msg import Header                                                         # metadata: timestamp of data & reference frame string
from geometry_msgs.msg import PoseStamped, Pose                                         # position & orientiation, stamped wraps header with pose
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker, InteractiveMarkerFeedback  # marker for the target & GUI

# bridge to RViz for interactive markers 
from interactive_markers.interactive_marker_server import InteractiveMarkerServer

class FakeTargetNode(Node):
    def __init__(self):
        super().__init__('fakeTarget')

        # create target position data for robot
        self.target_pub = self.create_publisher(PoseStamped, '/target_point', 10)

        self.server = InteractiveMarkerServer(self, 'target_server')

        self.create_interactive_marker()

        self.get_logger().info("Interactive Target Node is up and running!")


    def create_interactive_marker(self):
        int_target = InteractiveMarker()
        int_target.header.frame_id = 'root'
        int_target.name = 'target sphere'
        int_target.description = "Draggable target where the gun will point"
        int_target.scale = 0.1

        # sphere market for the target
        sphere_target = Marker()
        sphere_target.type = Marker.SPHERE
        sphere_target.scale.x = 0.02
        sphere_target.scale.y = 0.02
        sphere_target.scale.z = 0.02
        sphere_target.color.r = 0.0
        sphere_target.color.g = 0.5
        sphere_target.color.b = 0.5
        sphere_target.color.a = 1.0

        # Make the visual marker interative
        sphere_control = InteractiveMarkerControl()
        sphere_control.always_visible = True
        sphere_control.markers.append(sphere_target)
        int_target.controls.append(sphere_control)

        # X-Axis Drag Arrow (Red)
        control_x = InteractiveMarkerControl()
        control_x.name = "move_x"
        control_x.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control_x.orientation.w = 1.0
        control_x.orientation.x = 1.0  # Rotates around X 
        control_x.orientation.y = 0.0
        control_x.orientation.z = 0.0
        int_target.controls.append(control_x)

        # Y-Axis Drag Arrow (Green)
        control_y = InteractiveMarkerControl()
        control_y.name = "move_y"
        control_y.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control_y.orientation.w = 1.0
        control_y.orientation.x = 0.0
        control_y.orientation.y = 1.0  # Rotates around Y to point along Y
        control_y.orientation.z = 0.0
        int_target.controls.append(control_y)

        # Z-Axis Drag Arrow (Blue)
        control_z = InteractiveMarkerControl()
        control_z.name = "move_z"
        control_z.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control_z.orientation.w = 1.0
        control_z.orientation.x = 0.0
        control_z.orientation.y = 0.0
        control_z.orientation.z = 1.0  # Rotates around Z to point along Z
        int_target.controls.append(control_z)

        self.server.insert(int_target, feedback_callback=self.marker_feedback_callback)
        self.server.applyChanges()

    def marker_feedback_callback(self, feedback):
        # Only updates position if pose is updated
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE: 
            targetMsg = PoseStamped()

            # add time for sync
            targetMsg.header.stamp = self.get_clock().now().to_msg()

            # set frame_id for the marker
            targetMsg.header.frame_id = feedback.header.frame_id

            # add spatial data in the message
            targetMsg.pose.position.x = feedback.pose.position.x
            targetMsg.pose.position.y = feedback.pose.position.y
            targetMsg.pose.position.z = feedback.pose.position.z

            targetMsg.pose.orientation.x = feedback.pose.orientation.x
            targetMsg.pose.orientation.y = feedback.pose.orientation.y
            targetMsg.pose.orientation.z = feedback.pose.orientation.z
            targetMsg.pose.orientation.w = feedback.pose.orientation.w

            # send the message
            self.target_pub.publish(targetMsg)


def main(args = None):
    rclpy.init(args = args)

    node = FakeTargetNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:   
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
