import sys
import os
import string
# Check if the user has entered enough parameters
if len(sys.argv) < 8:
    print("Not enough arguments")
    sys.exit(1)

# Get the coordinates of the first boat (x, y) and the coordinates of the second boat
x_first = float(sys.argv[1])
y_first = float(sys.argv[2])
z_first = float(sys.argv[3])

x_second = float(sys.argv[4])
y_second = float(sys.argv[5])
z_second = float(sys.argv[6])

world_name = sys.argv[7]
boat_model_name = sys.argv[8]
# Set z coordinate to 0 for all boats
yaw_default = 0.0

# Create XML string for the boats
xml_content = """<?xml version="1.0" ?>
<sdf version="1.9">
  <world name='"""
xml_content += f"""{world_name}'>"""
# Create the first boat at position (x_first, y_first, z_first)
xml_content += f"""    
    <include>
      <name>flag_ship_1</name>
      <pose>{x_first} {y_first} {z_first} 0 0 {yaw_default}</pose>
      <uri>{boat_model_name}</uri>
    </include>

    <include>
      <name>flag_ship_2</name>
      <pose>{x_second} {y_second} {z_second} 0 0 3.1415 </pose>
      <uri>{boat_model_name}</uri>
    </include>

"""

# Add other information to the XML
xml_content += """
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <plugin
      filename="gz-sim-physics-system"
      name="gz::sim::systems::Physics">
    </plugin>
    <plugin
      filename="gz-sim-user-commands-system"
      name="gz::sim::systems::UserCommands">
    </plugin>
    <plugin
      filename="gz-sim-forcetorque-system"
      name="gz::sim::systems::ForceTorque">
    </plugin>
    <plugin
      filename="gz-sim-scene-broadcaster-system"
      name="gz::sim::systems::SceneBroadcaster">
    </plugin>
    <plugin
      filename="gz-sim-contact-system"
      name="gz::sim::systems::Contact">
    </plugin>
    <plugin
      filename="gz-sim-sensors-system"
      name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin
      filename="gz-sim-navsat-system"
      name="gz::sim::systems::NavSat">
    </plugin>
    <plugin
      filename="gz-sim-imu-system"
      name="gz::sim::systems::Imu">
    </plugin>
    <plugin
      filename="gz-sim-magnetometer-system"
      name="gz::sim::systems::Magnetometer">
    </plugin>
    
    <scene>
      <sky></sky>
      <grid>false</grid>
      <ambient>1.0 1.0 1.0</ambient>
      <background>0.8 0.8 0.8</background>
    </scene>

    <!-- Estimated latitude/longitude of sydneyregatta
         from satellite imagery -->
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>43.108075038718866</latitude_deg>
      <longitude_deg>5.91401078017241</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <include>
      <uri>
        https://fuel.gazebosim.org/1.0/OpenRobotics/models/Coast Water
      </uri>
      <pose>0 0 0.8 0 0 0</pose>
    </include>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>500 250</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode/>
            </friction>
            <bounce/>
            <contact/>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>500 250</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
        <pose>0 0 0 0 0 0</pose>
        <inertial>
          <pose>0 0 0 0 -0 0</pose>
          <mass>1</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
        <enable_wind>false</enable_wind>
      </link>
      <pose>250 125 0 0 -0 0</pose>
      <self_collide>false</self_collide>
    </model>

    <!-- The wave field -->
    <plugin filename="libPublisherPlugin.so" name="maritime::PublisherPlugin">
      <message type="gz.msgs.Param" topic="/gazebo/wavefield/parameters"
               every="2.0">
        params {
          key: "direction"
          value {
            type: DOUBLE
            double_value: 0.0
          }
        }
        params {
          key: "gain"
          value {
            type: DOUBLE
            double_value: 0.3
          }
        }
        params {
          key: "period"
          value {
            type: DOUBLE
            double_value: 5
          }
        }
        params {
          key: "steepness"
          value {
            type: DOUBLE
            double_value: 0
          }
        }
      </message>
    </plugin>

  </world>
</sdf>
"""

# Save the result to file
swarmz4_path = os.getenv("SWARMZ4_PATH", "")
file_path = os.path.join(swarmz4_path, "PX4-Autopilot/Tools/simulation/gz/worlds/")
#file_path = os.path.expanduser("$HOME/SWARMz4/PX4-Autopilot/Tools/simulation/gz/worlds/")
full_path = os.path.join(file_path, f"{world_name}.sdf")
# Save the result to file
with open(full_path, "w") as file:
    file.write(xml_content)

print("warships are created.")
