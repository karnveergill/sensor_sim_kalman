import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QGraphicsView, QGraphicsScene, QGraphicsRectItem
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QPainter
from geometry_msgs.msg import PointStamped

LATLON_TO_WINDOW_SCALE = 10

###############################################################################
    # GPSVisualizer Class Definition
class GPSVisualizer(Node):
    ###########################################################################
    # GPSVisualizer Constructor
    def __init__(self):
        super().__init__('gps_visualizer')
        
        # ROS 2 Subscribers
        self.gps_sub = self.create_subscription(PointStamped, 
                                                'gps_pub', 
                                                self.gps_callback, 
                                                10)

        self.filter_sub = self.create_subscription(PointStamped, 
                                                   'filtered_pose', 
                                                   self.filtered_callback, 
                                                   10)
        
        # Initialize GPS pose and filtered pose
        self.gps_pos = [0, 0]  
        self.filtered_pos = [0, 0] 
        
        # PyQt5 GUI Setup
        self.app = QApplication(sys.argv)
        self.scene = QGraphicsScene()
        self.view = QGraphicsView(self.scene)
        self.view.setRenderHint(QPainter.Antialiasing)
        self.view.setFixedSize(500, 500)
        
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
        print("GPS callback hit")
        self.gps_pos = [msg.point.x * LATLON_TO_WINDOW_SCALE, 
                        -msg.point.y * LATLON_TO_WINDOW_SCALE]

    ###########################################################################
    # Kalman Filter Data Callback 
    def filtered_callback(self, msg):
        print("Kalman Filter callback hit")
        self.filtered_pos = [msg.point.x * LATLON_TO_WINDOW_SCALE, 
                             -msg.point.y * LATLON_TO_WINDOW_SCALE]

    ###########################################################################
    # Function to update QT scene with visuals 
    def update_scene(self):
        self.gps_box.setPos(self.gps_pos[0], self.gps_pos[1])
        self.filtered_box.setPos(self.filtered_pos[0], self.filtered_pos[1])
        self.view.show()
    

    ###########################################################################
    # Run function to check for application end and close QT window 
    def run(self):
        sys.exit(self.app.exec_())

###############################################################################
# Application main to create visualizer node and run it
if __name__ == '__main__':
    rclpy.init()
    node = GPSVisualizer()
    node.run()
    rclpy.shutdown()
