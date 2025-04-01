### Sensor Simulator & Kalman Filter 
This repository uses ROS2 to publish and subscribe to simulated sensor data that is processed through a kalman filter
in order to reduce sensor noise.  This example uses GPS data for the kalman filters measurement updates and IMU data 
in an acceleration model for process updates.

The raw GPS data, filtered GPS position, and actual position are visualized by the `pose_visualization.py` script.
![kalman_viz](https://github.com/user-attachments/assets/22761749-0c7a-425c-a481-bc73130cda4e)
