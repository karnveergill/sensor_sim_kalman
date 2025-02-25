import sys
import threading
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QGraphicsView, QGraphicsScene, QGraphicsRectItem, QApplication, QMainWindow
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QPainter
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Constants
ROS2_QOS_CPP_PROFILE = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,  
                                  history=QoSHistoryPolicy.KEEP_LAST,
                                  depth=10)
LATLON_TO_WINDOW_SCALE = 10

###############################################################################
# GPSVisualizer Class Definition
class GPSVisualizer(Node, QMainWindow):
    def __init__(self):
        # Initialize ROS2 Node PyQt QMainWindow
        Node.__init__(self, 'gps_visualizer')
        QMainWindow.__init__(self)

        self.setWindowTitle("Kalman Filter GPS Visualizer")
        self.setGeometry(100, 100, 500, 500) # Window position & size
        
        # ROS 2 Subscribers
        self.gps_sub = self.create_subscription(NavSatFix, 
                                                'gps_pub', 
                                                self.gps_callback, 
                                                ROS2_QOS_CPP_PROFILE)

        self.filter_sub = self.create_subscription(PoseStamped, 
                                                   'filtered_pose', 
                                                   self.filtered_callback, 
                                                   ROS2_QOS_CPP_PROFILE)
        
        # Initialize GPS pose and filtered pose
        self.gps_pos = [0, 0]  
        self.filtered_pos = [0, 0] 
        
        # PyQt5 GUI Setup
        self.scene = QGraphicsScene()
        self.view = QGraphicsView(self.scene)
        self.view.setRenderHint(QPainter.Antialiasing)
        self.view.setFixedSize(500, 500)
        self.setCentralWidget(self.view) # Set as central widget
        
        # Create two rectangles for GPS and filtered data
        self.gps_box = QGraphicsRectItem(-5, -5, 10, 10)
        self.gps_box.setBrush(Qt.red)
        self.gps_box.setOpacity(0.6)
        
        self.filtered_box = QGraphicsRectItem(-5, -5, 10, 10)
        self.filtered_box.setBrush(Qt.blue)
        self.filtered_box.setOpacity(0.6)
        
        # Add created items to QT scene
        self.scene.addItem(self.gps_box)
        self.scene.addItem(self.filtered_box)
        
        # Setup QTimer to call update_scene() and update GUI at 20Hz
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_scene)
        self.timer.start(50) # Update every 50ms (20Hz)
    
    ###########################################################################
    # GPS Data Callback
    def gps_callback(self, msg):
        # print("Receieved GPS Pose Data")
        self.gps_pos = [msg.latitude * LATLON_TO_WINDOW_SCALE, 
                        -msg.longitude * LATLON_TO_WINDOW_SCALE]

    ###########################################################################
    # Kalman Filter Data Callback 
    def filtered_callback(self, msg):
        # print("Received Kalman Filter Pose Data")
        self.filtered_pos = [msg.pose.position.x * LATLON_TO_WINDOW_SCALE, 
                             -msg.pose.position.y * LATLON_TO_WINDOW_SCALE]

    ###########################################################################
    # Function to update QT scene with visuals 
    def update_scene(self):
        print(f"GPS X: {self.gps_pos[0]}, GPS Y: {self.gps_pos[1]}")
        print(f"Filtered X: {self.filtered_pos[0]}, Filtered Y: {self.filtered_pos[1]}")
        self.gps_box.setPos(self.gps_pos[0], self.gps_pos[1])
        self.filtered_box.setPos(self.filtered_pos[0], self.filtered_pos[1])
        self.view.show()
        # print("Display Updated")
    

    ###########################################################################
    # Run function to check for application end and close QT window 
    def run(self):
        sys.exit(self.app.exec_())

def ros_spin(node):
    # Runs ROS2 event loop in separate thread
    rclpy.spin(node)

###############################################################################
# Application main to create visualizer node and run it
def main():
    rclpy.init()
    app = QApplication(sys.argv)
    node = GPSVisualizer()

    # Start ROS2 spinning in separate thread to ensure data callbacks are hit
    ros_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    ros_thread.start()

    # Show PyQt window once at startup 
    node.show()

    # Run PyQt event loop
    sys.exit(app.exec_())

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

