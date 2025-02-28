# Visuo-Tactile Sensing

A real-time visualization system for tactile pressure sensors with physics-based pressure mapping and enhanced spatial-temporal processing. This system processes raw tactile sensor data through a series of transformations to create meaningful visualizations of pressure distribution with both spatial and temporal coherence.

![Tactile Sensor Visualization](docs/images/visualization-demo.png)

## Project Description

The Visuo-Tactile Sensing platform processes data from tactile sensor arrays to generate real-time visualizations of pressure distribution. Key features include:

- **Physics-Based Modeling**: Uses a sophisticated mapping between conductivity changes and physical pressure based on the sensor's properties
- **Spatial-Temporal Processing**: Implements both spatial (Gaussian filtering) and temporal (exponential smoothing) techniques to maintain coherence
- **Multi-Modal Visualization**: Provides both 2D heatmaps and 3D surface plots for intuitive pressure distribution analysis
- **Physical Unit Calibration**: Maps normalized readings to physical pressure units (Pascals) through a calibration process

The system is designed for researchers and developers working with tactile sensors in applications such as robotics, human-computer interaction, and biomechanics.

## Installation

### Prerequisites

- Ubuntu 22.04 or compatible Linux distribution
- ROS2 Humble (or later)
- Arduino IDE
- Python 3.8 or later

### ROS2 Installation

```bash
# Set locale
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 packages
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-humble-ros-base
sudo apt install ros-dev-tools
```

### Python Dependencies

```bash
# Install Python dependencies
pip install numpy==1.21.0
pip install scipy==1.7.0
pip install matplotlib==3.4.2
pip install opencv-python==4.5.3.56
pip install pyserial==3.5
```

### Clone Repository

```bash
git clone git@github.com:ameyj17/Visuo-Tactile-Sensing.git
cd Visuo-Tactile-Sensing
```

## Hardware Setup

1. **Arduino Connection**:
   - Connect your Arduino Uno to your computer via USB
   - Open Arduino IDE
   - Load the provided sketch from `arduino/tactile_sensor_read.ino`
   - Set the baud rate to 115200 or 200000 (as specified in the sketch)
   - Upload the sketch to your Arduino

2. **Tactile Sensor Connection**:
   - Connect your 16x16 tactile sensor array to Arduino following the pinout diagram in `docs/hardware_setup.md`
   - Verify connections using the Arduino Serial Monitor to ensure data is being transmitted

## Usage

### Running the Visualization

1. **Determine Arduino port**:
   ```bash
   ls /dev/ttyACM*
   ```
   Note the port (typically `/dev/ttyACM0`)

2. **Run the visualization script**:
   ```bash
   python3 src/read_data_port.py --port /dev/ttyACM0 --baud 115200
   ```

3. **Calibration**:
   - When prompted, ensure the sensor is unloaded
   - The system will perform a baseline calibration (collecting 30 samples)
   - Once calibration is complete, the visualization windows will appear

### Pressure Mapping Calibration

To calibrate the system for physical pressure units:

```bash
python3 src/pressure_calibration.py --port /dev/ttyACM0 --weights 25,370 --areas 78.54,78.54
```

### ROS2 Integration

If using with ROS2:

```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Run the ROS2 node
ros2 run visuo_tactile_sensing tactile_node --ros-args -p port:=/dev/ttyACM0 -p baud:=115200
```

## Visualization Controls

- Press 'q' in any visualization window to exit
- Press 'c' to recalibrate the sensor
- Press 's' to save the current frame as an image

## Physics-Based Modeling

The system uses a sophisticated approach to convert raw sensor data to pressure:

1. **Baseline Calibration**: Establishes a zero-pressure reference
2. **Voltage-to-Conductivity Mapping**: Uses a regularized inversion technique
3. **Conductivity-to-Pressure Conversion**: Applies a logarithmic transformation
4. **Spatial Filtering**: Gaussian filter for noise reduction
5. **Temporal Filtering**: Exponential smoothing for stability

## Project Structure

```
Visuo-Tactile-Sensing/
├── src/                  # Source code
│   ├── visualizer.py     # Main visualization class
│   ├── read_data_port.py # Data acquisition script
│   └── pressure_calibration.py # Calibration utility
├── arduino/              # Arduino sketches
│   └── tactile_sensor_read.ino # Data acquisition firmware
├── examples/             # Example scripts
├── docs/                 # Documentation
│   ├── images/           # Images for documentation
│   └── hardware_setup.md # Hardware setup guide
├── ros/                  # ROS2 integration
├── requirements.txt      # Python dependencies
├── LICENSE               # License file
└── README.md             # This file
```

## Troubleshooting

- **No data received**: Check Arduino connections and baud rate settings
- **Permission denied**: Run `sudo chmod a+rw /dev/ttyACM0` to grant access to the port
- **Noisy visualization**: Adjust the threshold and filtering parameters in `src/visualizer.py`

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- The physics-based conductivity-to-pressure mapping is based on principles from electrical impedance tomography
- Special thanks to our lab members for testing and feedback
