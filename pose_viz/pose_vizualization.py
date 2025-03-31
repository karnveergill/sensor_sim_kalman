import sys
import threading
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QGraphicsView, QGraphicsScene, QGraphicsPixmapItem, QGraphicsEllipseItem, QGraphicsLineItem
# QGraphicsRectItem, QApplication, QMainWindow
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QPixmap, QPen, QColor, QBrush
# QPainter, 
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from math import radians, cos
from pyproj import Proj, Transformer

# Constants
IMAGE_FILE = "map_715x716px_11x11mi.png"
MAP_WIDTH_PX = 715      # Image width in pixels
MAP_HEIGHT_PX = 716     # Image height in pixels
MAP_SIZE_KM = 17.7028   # Real-world size (11 miles = ~17.7 km)
GRID_SPACING_KM = 5     # Distance between grid lines in km
CENTER_LAT = 37.97154   # Center latitude of image
CENTER_LON = -114.29408 # Center longitude of image
METERS_PER_PIXEL =  (MAP_SIZE_KM * 1000) / MAP_WIDTH_PX # ~24.76 meters per pixel

ROS2_QOS_CPP_PROFILE = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,  
                                  history=QoSHistoryPolicy.KEEP_LAST,
                                  depth=10)
EASTING_OFFSET = 737686.00
NORTHING_OFFSET = 4206112.06

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
        
        # PyQt GUI setup
        self.setWindowTitle("Kalman Filter GPS Visualizer")
        self.setGeometry(100, 100, MAP_WIDTH_PX, MAP_HEIGHT_PX) # Window position & size

        # Setup graphics scene
        self.scene = QGraphicsScene()
        self.setScene(self.scene)

        # Load the map image
        pixmap = QPixmap(IMAGE_FILE)
        self.map_item = QGraphicsPixmapItem(pixmap)
        self.scene.addItem(self.map_item)

        # self.view = QGraphicsView(self.scene)
        # self.view.setRenderHint(QPainter.Antialiasing)
        # self.view.setFixedSize(500, 500)
        # self.setCentralWidget(self.view) # Set as central widget

        # Create GPS marker (red) & Kalman Filter marker (blue)
        self.gps_marker = self.create_and_add_marker(QColor(255, 0, 0, 150))    # Red
        self.filter_marker = self.create_and_add_marker(QColor(0, 0, 255, 150)) # Blue
        
        # Create two rectangles for GPS and filtered data
        # self.gps_box = QGraphicsRectItem(-5, -5, 10, 10)
        # self.gps_box.setBrush(QBrush(QColor(255, 0, 0, 150)))
        
        # self.filtered_box = QGraphicsRectItem(-5, -5, 10, 10)
        # self.filtered_box.setBrush(QBrush(QColor(0, 0, 255, 150)))
        
        # # Add created items to QT scene
        # self.scene.addItem(self.gps_box)
        # self.scene.addItem(self.filtered_box)
        
        # Setup QTimer to call update_scene() and update GUI at 20Hz
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_scene)
        self.timer.start(50) # Update every 50ms (20Hz)

        # Initialize UTM projections for lat/lon covertions
        utm_proj = Proj(proj="utm", zone=11, ellps="WGS84", datum="WGS84")
        self.transformer = Transformer.from_proj(Proj(proj="latlong", datum="WGS84"), utm_proj)

        self.zoom_factor = 1.0
    
    def create_and_add_marker(self, color):
        marker = QGraphicsEllipseItem(-5, -5, 10, 10) # 10x10 px circle
        marker.setBrush(QBrush(color))
        self.scene.addItem(marker)
        return marker
    
    def draw_grid_lines(self):
        pen = QPen(QColor(200, 200, 200, 100)) # Light gray, semi-transparent
        pen.setWidth(1)

        interval_pixels = GRID_SPACING_KM * 1000 / METERS_PER_PIXEL # Convert km to pixels

        # Draw vertical lines
        for i in range(-int(MAP_SIZE_KM/2), int(MAP_SIZE_KM/2) + 1, GRID_SPACING_KM):
            x = MAP_WIDTH_PX/2 + (i * 1000 / METERS_PER_PIXEL)
            line = QGraphicsLineItem(x, 0, x, MAP_HEIGHT_PX)
            line.setPen(pen)
            self.scene.addItem(line)

        # Draw horizontal lines
        for i in range(-int(MAP_SIZE_KM/2), int(MAP_SIZE_KM/2) + 1, GRID_SPACING_KM):
            y = MAP_HEIGHT_PX/2 - (I * 1000 / METERS_PER_PIXEL)
            line = QGraphicsLineItem(0, y, MAP_WIDTH_PX, y)
            line.setPen(pen)
            self.scene.addItem(line)
    
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
        print("Receieved GPS Pose Data")
        #x, y = self.utm_proj(msg.longitude, msg.latitude)
        print(f"lat:{msg.latitude}, lon:{msg.longitude}")
        try:
            x, y = self.latlon_to_pixels(msg.latitude, msg.longitude)
            self.gps_pos = [x, y]  #[x - EASTING_OFFSET, y - NORTHING_OFFSET]
        except Exception as e:
            print(f"Error converting latlon to pixel: {e}")
        
        #self.gps_marker.setPos(x, y)

    ###########################################################################
    # Kalman Filter Data Callback 
    def filtered_callback(self, msg):
        # print("Received Kalman Filter Pose Data")
        #x, y = self.utm_proj(msg.pose.position.y, msg.pose.position.x)
        x, y = self.latlon_to_pixels(msg.pose.position.x, msg.pose.position.y)
        self.filtered_pos = [x, y]  #[x - EASTING_OFFSET, y - NORTHING_OFFSET]
        #self.filter_marker.setPos(x, y)

    ###########################################################################
    # Function to update QT scene with visuals 
    def update_scene(self):
        self.gps_marker.setPos(self.gps_pos[0], self.gps_pos[1])
        self.filter_marker.setPos(self.filtered_pos[0], self.filtered_pos[1])

        # print(f"GPS X: {self.gps_pos[0]}, GPS Y: {self.gps_pos[1]}")
        # print(f"Filtered X: {self.filtered_pos[0]}, Filtered Y: {self.filtered_pos[1]}")
        # self.gps_box.setPos(self.gps_pos[0], self.gps_pos[1])
        # self.filtered_box.setPos(self.filtered_pos[0], self.filtered_pos[1])
        # self.view.show()
        # print("Display Updated")

    def wheelEvent(self, event):
        zoom_in = event.angleDelta().y() > 0 # Scoll up to zoom in

        # Adjust zoom factor
        if zoom_in:
            self.zoom_factor *= 1.1 # Zoom in (increase size by 10%)
        else:
            self.zoom_factor /= 1.1 # Zoom out (decrease size by 10%)

        self.scale(self.zoom_factor, self.zoom_factor)
    

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

