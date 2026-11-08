"""RViz2 markers for the bundled Gazebo/CBF demonstration."""
import math
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from visualization_msgs.msg import Marker, MarkerArray
from safe_obstacle_avoidance_msgs.msg import SafeSetEllipsoidsConfig

def yaw_of(q):
    return math.atan2(2.0*(q.w*q.z+q.x*q.y), 1.0-2.0*(q.y*q.y+q.z*q.z))

class CBFVisualization(Node):
    def __init__(self):
        super().__init__("cbf_visualization")
        self.declare_parameter("marker_topic", "/cbf_visualization/markers")
        self.declare_parameter("ellipse_segments", 32)
        self.odom = None
        self.safe = None
        self.pub = self.create_publisher(MarkerArray, self.get_parameter("marker_topic").value, 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(SafeSetEllipsoidsConfig, "/safe_set_config", self.on_safe, 10)
        self.get_logger().info("Publishing robot, demo obstacles and CBF ellipses for RViz2; odom subscription uses sensor-data BEST_EFFORT QoS")

    def marker(self, frame, stamp, mid, kind):
        m=Marker()
        m.header.frame_id=frame
        m.header.stamp=stamp
        m.ns="cbf_demo"
        m.id=mid
        m.type=kind
        m.action=Marker.ADD
        m.pose.orientation.w=1.0
        return m

    def on_odom(self,msg):
        self.odom=msg
        self.publish()

    def on_safe(self,msg):
        if msg.columns == 5 and len(msg.data) and len(msg.data)%5 == 0:
            self.safe=msg
            self.publish()

    def publish(self):
        if self.odom is None:
            return
        frame=self.odom.header.frame_id or "odom"
        stamp=self.get_clock().now().to_msg()
        out=MarkerArray()
        pose=self.odom.pose.pose
        body=self.marker(frame,stamp,1,Marker.CUBE)
        body.pose=pose
        body.pose.position.z=max(0.12,pose.position.z)
        body.scale.x,body.scale.y,body.scale.z=0.34,0.28,0.10
        body.color.r,body.color.g,body.color.b,body.color.a=0.12,0.38,0.95,1.0
        out.markers.append(body)
        arrow=self.marker(frame,stamp,2,Marker.ARROW)
        arrow.pose=pose
        arrow.pose.position.z=max(0.20,pose.position.z+0.10)
        arrow.scale.x,arrow.scale.y,arrow.scale.z=0.42,0.045,0.045
        arrow.color.r,arrow.color.g,arrow.color.b,arrow.color.a=0.1,0.85,1.0,1.0
        out.markers.append(arrow)
        # These three markers mirror the fixed obstacle poses in the bundled SDF.
        obstacles=[
          (20,Marker.CYLINDER,1.65,0.0,0.40,0.48,0.48,0.80,0.90,0.18,0.10),
          (21,Marker.CYLINDER,3.10,0.68,0.40,0.56,0.56,0.80,0.95,0.58,0.08),
          (22,Marker.CUBE,4.40,-0.55,0.35,0.55,0.55,0.70,0.15,0.65,0.30)]
        for mid,kind,x,y,z,sx,sy,sz,r,g,b in obstacles:
            m=self.marker(frame,stamp,mid,kind)
            m.pose.position.x,m.pose.position.y,m.pose.position.z=x,y,z
            m.scale.x,m.scale.y,m.scale.z=sx,sy,sz
            m.color.r,m.color.g,m.color.b,m.color.a=r,g,b,0.88
            out.markers.append(m)
        if self.safe is not None:
            origin=self.safe.origin_pose
            yaw=yaw_of(origin.orientation)
            c,s=math.cos(yaw),math.sin(yaw)
            n=max(12,int(self.get_parameter("ellipse_segments").value))
            for i in range(len(self.safe.data)//5):
                cx,cy,a,b,phi=self.safe.data[i*5:i*5+5]
                cp,sp=math.cos(phi),math.sin(phi)
                line=self.marker(frame,stamp,1000+i,Marker.LINE_STRIP)
                line.scale.x=0.012
                line.color.r,line.color.g,line.color.b,line.color.a=0.10,0.95,0.38,0.88
                for k in range(n+1):
                    t=2.0*math.pi*k/n
                    lx=cx+a*math.cos(t)*cp-b*math.sin(t)*sp
                    ly=cy+a*math.cos(t)*sp+b*math.sin(t)*cp
                    p=Point()
                    p.x=origin.position.x+lx*c-ly*s
                    p.y=origin.position.y+lx*s+ly*c
                    p.z=0.035
                    line.points.append(p)
                out.markers.append(line)
        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node=CBFVisualization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
