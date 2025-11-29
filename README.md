# ROS2 Image Converter Package

A ROS2 package for real-time image conversion between color and grayscale modes with dynamic mode switching via service calls.

## 📋 Features

- **Real-time Image Processing**: Converts camera images between color (BGR8) and grayscale (MONO8) formats
- **Dynamic Mode Switching**: Change conversion mode on-the-fly using ROS2 services
- **Configurable Topics**: Customize input and output topics via parameters
- **USB Camera Integration**: Includes launch file for seamless USB camera setup
- **Low Latency**: Efficient processing using OpenCV and cv_bridge

## 🎥 Demo
https://drive.google.com/file/d/1VTpIfmCTp04WUiUjbkPL_xvnMML26ltg/view?usp=sharing

## 🛠️ Requirements

- ROS2 (Humble/Iron/Jazzy)
- Python 3.8+
- OpenCV (`cv2`)
- `cv_bridge`
- `usb_cam` package (for camera support)

## 📦 Installation

1. **Clone the repository into your ROS2 workspace:**
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/vaishman/ros2_image_converter.git
   cd ~/ros2_ws
   ```

2. **Install dependencies:**
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the package:**
   ```bash
   colcon build --packages-select image_conversion_pkg
   source install/setup.bash
   ```

## 🚀 Usage

### Launch with USB Camera

Start the image converter with a USB camera:

```bash
ros2 launch image_conversion_pkg image_conversion.launch.py
```

### Run Node Standalone

If you have your own image source:

```bash
ros2 run image_conversion_pkg image_conversion --ros-args \
  -p input_topic:=/your/image/topic \
  -p output_topic:=/converted/image
```

### Switch Conversion Mode

Use the service to toggle between color and grayscale:

```bash
# Switch to Grayscale (True)
ros2 service call /set_mode std_srvs/srv/SetBool "{data: true}"

# Switch to Color (False)
ros2 service call /set_mode std_srvs/srv/SetBool "{data: false}"
```

### View Output Images

Visualize the converted images using rqt_image_view:

```bash
ros2 run rqt_image_view rqt_image_view
```

Then select `/image_converted` from the dropdown menu.

## 📖 Package Structure

```
image_conversion_pkg/
├── image_conversion.py           # Main conversion node
├── image_conversion.launch.py    # Launch file with USB camera
├── package.xml                   # Package manifest
├── setup.py                      # Python package setup
└── README.md                     # This file
```

## ⚙️ Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `input_topic` | string | `/image_raw` | Input image topic to subscribe to |
| `output_topic` | string | `/image_converted` | Output topic for converted images |

## 🔧 Services

| Service | Type | Description |
|---------|------|-------------|
| `/set_mode` | `std_srvs/srv/SetBool` | Switch conversion mode: `true` = grayscale, `false` = color |

## 📡 Topics

### Subscribed Topics

- `/image_raw` (default) - `sensor_msgs/msg/Image`: Input image stream

### Published Topics

- `/image_converted` (default) - `sensor_msgs/msg/Image`: Converted image stream

## 🎯 How It Works

1. The node subscribes to an image topic (default: `/image_raw`)
2. Images are converted using OpenCV via the cv_bridge
3. Based on the current mode:
   - **Mode 1 (Grayscale)**: Converts BGR8 → MONO8
   - **Mode 2 (Color)**: Passes through as BGR8
4. Converted images are published to the output topic
5. Mode can be changed dynamically via the `/set_mode` service

## 🐛 Troubleshooting

**Camera not detected:**
```bash
# Check available video devices
ls /dev/video*

# Update video_device parameter in launch file if needed
```

**No image output:**
```bash
# Verify topics
ros2 topic list

# Check if images are being published
ros2 topic hz /image_raw
```

**Permission denied for camera:**
```bash
sudo usermod -a -G video $USER
# Then log out and log back in
```



## 👤 Author

**Vaish**
- GitHub: [@vaishman](https://github.com/vaishman)

