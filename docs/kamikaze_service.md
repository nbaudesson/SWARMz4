# Kamikaze Service Documentation

The kamikaze service implements a self-destruct mechanism that damages both the initiating robot and any nearby robots within a configurable explosion range.

## Service Information

- **Service Name:** `/kamikaze`
- **Service Type:** `swarmz_interfaces/srv/Kamikaze`
- **Node Name:** `kamikaze_service_server`

## Parameters

- `explosion_damage` (integer, default: 100): Amount of damage dealt to robots in range
- `explosion_range` (double, default: 6.0): Radius in meters within which robots take damage

## Implementation Details

The kamikaze service works as follows:

1. When a kamikaze request is received:
   - Validates the requesting robot's name
   - Retrieves the robot's current position

2. Damage Application:
   - The initiating robot takes full explosion damage
   - All robots within `explosion_range` meters take the same amount of damage
   - Damage is applied through the `update_health` service

3. Position Tracking:
   - Uses `GazeboPosesTracker` to monitor all robot positions
   - Calculates distances between robots to determine damage recipients

## Command Line Usage

Trigger the kamikaze action for a specific robot:

```bash
ros2 service call /kamikaze swarmz_interfaces/srv/Kamikaze "{robot_name: 'robot1'}"
```

You can also view service information:

```bash
# List all available services
ros2 service list

# View service type
ros2 service type /kamikaze

# View service interface
ros2 interface show swarmz_interfaces/srv/Kamikaze
```

## Dependencies

- `swarmz_interfaces`: Custom interface package containing service definitions
- `utils.tools`: Contains helper functions for namespace management
- `utils.gazebo_subscriber`: Provides position tracking functionality

## Error Handling

The service includes several safety checks:
- Validates robot namespace existence
- Ensures robot name is provided
- Handles exceptions during position tracking and health updates
- Logs warnings and errors for debugging

## Example Scenario

1. Robot "robot1" initiates kamikaze:
   ```bash
   ros2 service call /kamikaze swarmz_interfaces/srv/Kamikaze "{robot_name: '/px4_1'}"
   ```

2. Results:
   - "robot1" takes 100 damage (default)
   - All robots within 6.0 meters also take 100 damage
   - Robots outside this range are unaffected
   - The service logs the affected robots and distances
