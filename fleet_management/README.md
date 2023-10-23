# Fleet Management ROS2 Application

## Introduction

The Fleet Management ROS2 Application is designed to allocate and route vehicles efficiently by specifying the fleet size. This README provides instructions on how to set up, build, and run the application. It also covers testing scenarios and additional information for users.

## Prerequisites

Before using this application, ensure that you have the following prerequisites installed on your system:

- ROS2 (Foxy Fitzroy or later)
- Python 3.x
- `colcon` for building ROS2 packages
- The `fleet_management_interfaces` package for custom action messages (ensure you've generated it)

## Running the Application

### Action Server

1. Start the ROS2 Master (if not already running):

   ```bash
   ros2 run ros_core ros2_master
   ```

2. Run the Action Server:

   ```bash
   ros2 run fleet_management fleet_management_server.py
   ```

### Action Client CLI

1. Open a new terminal.

2. Navigate to the package directory:

   ```bash
   cd /path/to/your/ros2/workspace/src/fleet_management
   ```

3. Run the Action Client CLI with the desired fleet size:

   ```bash
   python fleet_management_client.py --fleet-size <size>
   ```

   Replace `<size>` with the desired fleet size.

## Using the Professional CLI

The application provides a professional Command Line Interface (CLI) for users to allocate and route vehicles. You can use the `fleet-management` command with the `--fleet-size` option to specify the fleet size.

Example:

```bash
python your_cli_script.py fleet-management --fleet-size 8
```

## Testing Scenarios

### Scenario 1:

**Goal Description:** Allocate vehicles for a suburban package delivery service.
- **Fleet Size:** 8 vehicles
- **Expected Output (Vehicle Routes):**
  - Vehicle 1: Route from Depot A to Customer 5 to Customer 12 and back to Depot A.
  - Vehicle 2: Route from Depot A to Customer 1 to Customer 7 and back to Depot A.
  - Vehicle 3: Route from Depot A to Customer 3 to Customer 8 and back to Depot A.
  - Vehicle 4: Route from Depot A to Customer 9 to Customer 11 and back to Depot A.
  - Vehicle 5: Route from Depot A to Customer 2 to Customer 6 and back to Depot A.
  - Vehicle 6: Route from Depot A to Customer 4 to Customer 10 and back to Depot A.
  - Vehicle 7: Route from Depot A to Customer 13 to Customer 16 and back to Depot A.
  - Vehicle 8: Route from Depot A to Customer 14 to Customer 15 and back to Depot A.

### Scenario 2:

**Goal Description:** Allocate heavy equipment for a construction project.
- **Fleet Size:** 10 heavy-duty vehicles
- **Expected Output (Vehicle Routes):**
  - Vehicle 1: Route from Equipment Yard A to Construction Site 1 and back to Equipment Yard A.
  - Vehicle 2: Route from Equipment Yard A to Construction Site 2 and back to Equipment Yard A.
  - Vehicle 3: Route from Equipment Yard A to Construction Site 3 and back to Equipment Yard A.
  - Vehicle 4: Route from Equipment Yard A to Construction Site 4 and back to Equipment Yard A.
  - Vehicle 5: Route from Equipment Yard A to Construction Site 5 and back to Equipment Yard A.
  - Vehicle 6: Route from Equipment Yard A to Construction Site 6 and back to Equipment Yard A.
  - Vehicle 7: Route from Equipment Yard A to Construction Site 7 and back to Equipment Yard A.
  - Vehicle 8: Route from Equipment Yard A to Construction Site 8 and back to Equipment Yard A.
  - Vehicle 9: Route from Equipment Yard A to Construction Site 9 and back to Equipment Yard A.
  - Vehicle 10: Route from Equipment Yard A to Construction Site 10 and back to Equipment Yard A.

## Additional Information

- You can further customize and extend the application by modifying the action server and client scripts according to your requirements.

- For more information about ROS2 and custom actions, refer to the [ROS2 documentation](https://docs.ros.org/en/galactic/Tutorials.html).
