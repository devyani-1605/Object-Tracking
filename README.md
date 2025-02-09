
# Object Detection & Tracking in ROS2 (Using OpenCV/YOLO)

This project consists of a ROS 2 publisher node (Publisher.py) and a subscriber node (Subscriber.py). The publisher captures images using an Intel RealSense D435 camera (Depth Camera), processes them with YOLOv5 for object detection, and publishes the detected object information, including bounding box coordinates and depth. The subscriber listens for these messages and displays the data in a structured table.




## Prerequisites

- ROS 2 (Foxy, Galactic, or Humble recommended) - Install ROS 2 from ROS official website

- Python 3.8+

- Intel RealSense SDK (librealsense) - Install from Intel RealSense

- YOLOv5 - Install from Ultralytics YOLOv5


## Installation

Clone the Repository and Install Dependencies

```bash
  git clone https://github.com/devyani-1605/Object-Tracking.git
  cd ROS2_WS
  pip install -r requirements.txt
```
## Setup ROS 2 Environment

```bash
  source /opt/ros/<your_ros2_distro>/setup.bash
```
Replace <your_ros2_distro> with foxy, galactic, or humble as per your installation.

## Build the Workspace
```bash
  cd ROS2_WS
  colcon build
  source install/setup.bash
```



    
## Run Locally

Launch the ROS 2 system

```bash
  ros2 launch object_tracking main.launch.py
```



## Expected Output

- The publisher node will show real-time object detection with bounding boxes and depth estimation.

- The subscriber node will print the detected object details in a structured table format.
## Troubleshooting

- Ensure ROS 2 is correctly installed and sourced before running the scripts.

- Make sure the Intel RealSense camera is connected and detected (rs-enumerate-devices).

- If you face permission issues, run:
```bash
  sudo chmod 666 /dev/video*
```
- If any package is missing, install it using pip install <package_name>.
