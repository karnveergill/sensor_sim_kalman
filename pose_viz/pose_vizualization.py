import sys
import threading
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QGraphicsView, QGraphicsScene, QGraphicsPixmapItem, QGraphicsEllipseItem, QGraphicsLineItem
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QPixmap, QPen, QColor, QBrush
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from math import radians, cos
from pyproj import Proj, Transformer

# Constants
IMAGE_FILE = "map_744x745_600x600ft.png"
MAP_WIDTH_PX = 744      # Image width in pixels
MAP_HEIGHT_PX = 745     # Image height in pixels
MAP_SIZE_KM = 0.18288   # Real-world size (600 ft = ~0.18288 km)
GRID_SPACING_KM = 0.01  # Distance between grid lines in km (10m)
CENTER_LAT = 32.7640826819392   # Center latitude of image
CENTER_LON = -117.22248798518055 # Center longitude of image
METERS_PER_PIXEL =  (MAP_SIZE_KM * 1000) / MAP_WIDTH_PX # ~24.76 meters per pixel
ROS2_QOS_CPP_PROFILE = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,  
                                  history=QoSHistoryPolicy.KEEP_LAST,
                                  depth=10)
# TODO: Fix UTM, not a problem but should figure out
USE_UTM = False # Toggle between UTM or Flat Approximation

###############################################################################
# GPSVisualizer Class Definition
class GPSVisualizer(Node, QGraphicsView):  #QMainWindow):
    def __init__(self):
        # Initialize ROS2 Node PyQt QGraphicsView
        Node.__init__(self, 'gps_visualizer')
        QGraphicsView.__init__(self)
        #QMainWindow.__init__(self)
        
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
        
        # PyQt GUI title, window position & size
        self.setWindowTitle("Kalman Filter GPS Visualizer")
        self.setGeometry(100, 100, MAP_WIDTH_PX, MAP_HEIGHT_PX)

        # Setup graphics scene
        self.scene = QGraphicsScene()
        self.setScene(self.scene)

        # Load the map image
        pixmap = QPixmap(IMAGE_FILE)
        self.map_item = QGraphicsPixmapItem(pixmap)
        self.scene.addItem(self.map_item)

        # Zoom factor for scrolling in and out
        self.zoom_factor = 1.0

        # Create GPS marker (red) & Kalman Filter marker (blue)
        self.gps_marker = self.create_and_add_marker(QColor(255, 0, 0, 150))    
        self.filter_marker = self.create_and_add_marker(QColor(0, 0, 255, 150))

        # Add center marker
        center_marker = self.create_and_add_marker(QColor(0, 0, 0, 150))
        x, y = self.latlon_to_pixels(CENTER_LAT, CENTER_LON)
        center_marker.setPos(x, y)

        # Draw grid lines on map
        self.draw_grid_lines()
        
        # Setup QTimer to call update_scene() and update GUI at 20Hz
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_scene)
        self.timer.start(50) # Update every 50ms (20Hz)

        # Initialize UTM projections for lat/lon covertions
        utm_proj = Proj(proj="utm", zone=11, ellps="WGS84", datum="WGS84")
        self.transformer = Transformer.from_proj(Proj(proj="latlong", datum="WGS84"), 
                                                 utm_proj)
    
    ###########################################################################
    # Create ellipse marker of specified color and add to QT scene
    def create_and_add_marker(self, color):
        marker = QGraphicsEllipseItem(-5, -5, 10, 10) # 10x10 px circle
        marker.setBrush(QBrush(color))
        self.scene.addItem(marker)
        return marker
    
    ###########################################################################
    # Draw grid lines on QT scene
    def draw_grid_lines(self):
        pen = QPen(QColor(200, 200, 200, 100)) # Light gray, semi-transparent
        pen.setWidth(1)

        interval_pixels = GRID_SPACING_KM * 1000 / METERS_PER_PIXEL # Convert km to pixels

        # Draw vertical lines
        SIZE_M = float(MAP_SIZE_KM * 1000 / 2)
        SIZE_M = int(SIZE_M)
        SPACING_M = float(GRID_SPACING_KM * 1000)
        SPACING_M = int(SPACING_M)
        for i in range(-SIZE_M, SIZE_M + 1, SPACING_M):
            x = MAP_WIDTH_PX/2 + (i / METERS_PER_PIXEL)
            line = QGraphicsLineItem(x, 0, x, MAP_HEIGHT_PX)
            line.setPen(pen)
            self.scene.addItem(line)

        # Draw horizontal lines
        for i in range(-SIZE_M, SIZE_M + 1, SPACING_M):
            y = MAP_HEIGHT_PX/2 - (i / METERS_PER_PIXEL)
            line = QGraphicsLineItem(0, y, MAP_WIDTH_PX, y)
            line.setPen(pen)
            self.scene.addItem(line)
    
    ###########################################################################
    # Convert Lat / Lon coordinates into pixel coordinates
    def latlon_to_pixels(self, lat, lon):
        # Check for invalid lat / lon
        if lat is None or lon is None or not (-90 <= lat <= 90) or not (-180 <= lon <= 180):
            return self.map_width_px / 2, self.map_height_px / 2  # Default center position
        
        if USE_UTM:
            # Convert to UTM meters
            try:
                utm_x, utm_y = self.transformer.transform(lon, lat)
            except Exception as e:
                print(f"Error in UTM conversion: {e}")
                return MAP_WIDTH_PX / 2, MAP_HEIGHT_PX /2
            
            center_x, center_y = self.transformer.transform(CENTER_LON, CENTER_LON)
            x_meters = utm_x - center_x
            y_meters = utm_y - center_y
        else:
            # Flat approximation
            lat_diff = (lat - CENTER_LAT) * 111320 
            lon_diff = (lon - CENTER_LON) * 111320 * cos(radians(CENTER_LAT))
            x_meters = lon_diff
            y_meters = lat_diff

        # Convert meters to pixels
        x = (MAP_WIDTH_PX/2) + (x_meters / METERS_PER_PIXEL) * self.zoom_factor
        y = (MAP_HEIGHT_PX/2) - (y_meters / METERS_PER_PIXEL) * self.zoom_factor
        return x, y

    ###########################################################################
    # GPS Data Callback
    def gps_callback(self, msg):
        #print("Receieved GPS Pose Data")
        #print(f"lat:{msg.latitude}, lon:{msg.longitude}")
        try:
            x, y = self.latlon_to_pixels(msg.latitude, msg.longitude)
            self.gps_pos = [x, y]
        except Exception as e:
            print(f"Error converting latlon to pixel: {e}")

    ###########################################################################
    # Kalman Filter Data Callback 
    def filtered_callback(self, msg):
        #print("Received Kalman Filter Pose Data")
        x, y = self.latlon_to_pixels(msg.pose.position.x, msg.pose.position.y)
        self.filtered_pos = [x, y] 

    ###########################################################################
    # Function to update QT scene visuals 
    def update_scene(self):
        self.gps_marker.setPos(self.gps_pos[0], self.gps_pos[1])
        self.filter_marker.setPos(self.filtered_pos[0], self.filtered_pos[1])

    ###########################################################################
    # Scroll wheel event to capture zoom in/out commands
    def wheelEvent(self, event):
        zoom_in = event.angleDelta().y() > 0 # Scoll up to zoom in
        angle_y = event.angleDelta().y()
        angle_x = event.angleDelta().x()
        print(f"angle Y: {angle_y}  angle x: {angle_x}")

        # Adjust zoom factor
        if zoom_in:
            self.zoom_factor = 1.1 # Zoom in (increase size by 10%)
        else:
            self.zoom_factor = 0.9 # Zoom out (decrease size by 10%)

        self.scale(self.zoom_factor, self.zoom_factor)
        print(f"zoom factor: {self.zoom_factor}")
    

    ###########################################################################
    # Run function to check for application end and close QT window 
    def run(self):
        sys.exit(self.app.exec_())

def ros_spin(node):
    # Runs ROS2 event loop in separate thread
    rclpy.spin(node)

###############################################################################
# Application main to create visualizer node thread and start Qt window
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