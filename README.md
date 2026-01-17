# DBCM-2kgf Load Cell ROS 2 Driver

This is the `load_cell_publisher` package that reads a DBCM-2kgf load cell over serial and publishes it to a ROS 2 topic.  
It assumes the Arduino (or MCU) sends one numeric value (ASCII float) per line.

## Package Layout

- `load_cell_publisher/load_cell_publisher_node.py`  
  Reads from the serial port and publishes to `/loadcell` as `std_msgs/Float32`.  
  Uses a separate reader thread to minimize blocking.
- `launch/load_cell_publisher.launch.py`  
  Launches the node with port/baud parameters.
- `package.xml`, `setup.py`, `setup.cfg`  
  ROS 2 Python package configuration.

## Dependencies

- ROS 2 (`rclpy`, `std_msgs`)
- Python package: `pyserial`

`pyserial` is included in `install_requires` in `setup.py`.

## Topic

- Publisher: `/loadcell`  
  - Type: `std_msgs/msg/Float32`  
  - Data: load cell measurement

## Parameters

- `port` (string, default: `/dev/ttyUSB0`)  
  Serial port path
- `baud` (int, default: `115200`)  
  Baud rate

## Usage

### Build

```bash
cd /home/jaewoo/ros2_workspace/dbcm_loadcell_ws
colcon build --packages-select load_cell_publisher
source install/setup.bash
```

### Run with launch file

```bash
ros2 launch load_cell_publisher load_cell_publisher.launch.py
```

Edit the `port` and `baud` values in the launch file as needed.

### Run with parameters

```bash
ros2 run load_cell_publisher load_cell_publisher \
  --ros-args -p port:=/dev/ttyUSB0 -p baud:=115200
```

## Assumptions and Notes

- Serial input must be **one numeric string per line**.  
  (e.g., `123.45\n`)
- Non-numeric lines are ignored (boot messages, etc.).
- On serial I/O errors, a warning is logged and the node retries.

## License


