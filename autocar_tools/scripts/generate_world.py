import os

def generate_world():
    world_path = "/home/mahboob-alam/four_wheel_drive/src/autocar_gazebo/worlds/campus.world"
    os.makedirs(os.path.dirname(world_path), exist_ok=True)
    
    xml = []
    xml.append("<sdf version='1.6'>")
    xml.append("  <world name='campus_world'>")
    
    # -------------------------------------------------------------
    # Lighting and Environment
    # -------------------------------------------------------------
    xml.append("    <light name='sun' type='directional'>")
    xml.append("      <cast_shadows>true</cast_shadows>")
    xml.append("      <pose>0 0 50 0 0 0</pose>")
    xml.append("      <diffuse>0.8 0.8 0.8 1</diffuse>")
    xml.append("      <specular>0.1 0.1 0.1 1</specular>")
    xml.append("      <attenuation>")
    xml.append("        <range>1000</range>")
    xml.append("        <constant>0.9</constant>")
    xml.append("        <linear>0.01</linear>")
    xml.append("        <quadratic>0.001</quadratic>")
    xml.append("      </attenuation>")
    xml.append("      <direction>-0.5 0.1 -0.9</direction>")
    xml.append("    </light>")
    
    # Ground plane (large grass field)
    xml.append("    <model name='ground_plane'>")
    xml.append("      <static>true</static>")
    xml.append("      <link name='link'>")
    xml.append("        <collision name='collision'>")
    xml.append("          <geometry>")
    xml.append("            <plane>")
    xml.append("              <normal>0 0 1</normal>")
    xml.append("              <size>1000 1000</size>")
    xml.append("            </plane>")
    xml.append("          </geometry>")
    xml.append("          <surface>")
    xml.append("            <friction>")
    xml.append("              <ode>")
    xml.append("                <mu>0.8</mu>")
    xml.append("                <mu2>0.8</mu2>")
    xml.append("              </ode>")
    xml.append("            </friction>")
    xml.append("          </surface>")
    xml.append("        </collision>")
    xml.append("        <visual name='visual'>")
    xml.append("          <cast_shadows>false</cast_shadows>")
    xml.append("          <geometry>")
    xml.append("            <plane>")
    xml.append("              <normal>0 0 1</normal>")
    xml.append("              <size>1000 1000</size>")
    xml.append("            </plane>")
    xml.append("          </geometry>")
    xml.append("          <material>")
    xml.append("            <script>")
    xml.append("              <uri>file://media/materials/scripts/gazebo.material</uri>")
    xml.append("              <name>Gazebo/Grass</name>")
    xml.append("            </script>")
    xml.append("          </material>")
    xml.append("        </visual>")
    xml.append("      </link>")
    xml.append("    </model>")
    
    # -------------------------------------------------------------
    # Roads Layout
    # -------------------------------------------------------------
    # Main Road East-West split around roundabout at (0,0)
    # West section: from -400 to -20
    xml.append("    <model name='road_main_west'>")
    xml.append("      <static>true</static>")
    xml.append("      <link name='link'>")
    xml.append("        <visual name='visual'>")
    xml.append("          <pose>-210 0 0.002 0 0 0</pose>")
    xml.append("          <geometry><box><size>380 8 0.004</size></box></geometry>")
    xml.append("          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Road</name></script></material>")
    xml.append("        </visual>")
    xml.append("        <collision name='collision'>")
    xml.append("          <pose>-210 0 0.002 0 0 0</pose>")
    xml.append("          <geometry><box><size>380 8 0.004</size></box></geometry>")
    xml.append("        </collision>")
    xml.append("      </link>")
    xml.append("    </model>")
    
    # East section: from 20 to 400
    xml.append("    <model name='road_main_east'>")
    xml.append("      <static>true</static>")
    xml.append("      <link name='link'>")
    xml.append("        <visual name='visual'>")
    xml.append("          <pose>210 0 0.002 0 0 0</pose>")
    xml.append("          <geometry><box><size>380 8 0.004</size></box></geometry>")
    xml.append("          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Road</name></script></material>")
    xml.append("        </visual>")
    xml.append("        <collision name='collision'>")
    xml.append("          <pose>210 0 0.002 0 0 0</pose>")
    xml.append("          <geometry><box><size>380 8 0.004</size></box></geometry>")
    xml.append("        </collision>")
    xml.append("      </link>")
    xml.append("    </model>")
    
    # North Road East-West (at y=200)
    xml.append("    <model name='road_north'>")
    xml.append("      <static>true</static>")
    xml.append("      <link name='link'>")
    xml.append("        <visual name='visual'>")
    xml.append("          <pose>0 200 0.002 0 0 0</pose>")
    xml.append("          <geometry><box><size>800 6 0.004</size></box></geometry>")
    xml.append("          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Road</name></script></material>")
    xml.append("        </visual>")
    xml.append("        <collision name='collision'>")
    xml.append("          <pose>0 200 0.002 0 0 0</pose>")
    xml.append("          <geometry><box><size>800 6 0.004</size></box></geometry>")
    xml.append("        </collision>")
    xml.append("      </link>")
    xml.append("    </model>")

    # South Road East-West (at y=-200)
    xml.append("    <model name='road_south'>")
    xml.append("      <static>true</static>")
    xml.append("      <link name='link'>")
    xml.append("        <visual name='visual'>")
    xml.append("          <pose>0 -200 0.002 0 0 0</pose>")
    xml.append("          <geometry><box><size>800 6 0.004</size></box></geometry>")
    xml.append("          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Road</name></script></material>")
    xml.append("        </visual>")
    xml.append("        <collision name='collision'>")
    xml.append("          <pose>0 -200 0.002 0 0 0</pose>")
    xml.append("          <geometry><box><size>800 6 0.004</size></box></geometry>")
    xml.append("        </collision>")
    xml.append("      </link>")
    xml.append("    </model>")

    # Connectors (North-South)
    # West Connector (x = -350)
    xml.append("    <model name='road_conn_west'>")
    xml.append("      <static>true</static>")
    xml.append("      <link name='link'>")
    xml.append("        <visual name='visual'>")
    xml.append("          <pose>-350 0 0.002 0 0 0</pose>")
    xml.append("          <geometry><box><size>6 400 0.004</size></box></geometry>")
    xml.append("          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Road</name></script></material>")
    xml.append("        </visual>")
    xml.append("        <collision name='collision'>")
    xml.append("          <pose>-350 0 0.002 0 0 0</pose>")
    xml.append("          <geometry><box><size>6 400 0.004</size></box></geometry>")
    xml.append("        </collision>")
    xml.append("      </link>")
    xml.append("    </model>")

    # East Connector (x = 350)
    xml.append("    <model name='road_conn_east'>")
    xml.append("      <static>true</static>")
    xml.append("      <link name='link'>")
    xml.append("        <visual name='visual'>")
    xml.append("          <pose>350 0 0.002 0 0 0</pose>")
    xml.append("          <geometry><box><size>6 400 0.004</size></box></geometry>")
    xml.append("          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Road</name></script></material>")
    xml.append("        </visual>")
    xml.append("        <collision name='collision'>")
    xml.append("          <pose>350 0 0.002 0 0 0</pose>")
    xml.append("          <geometry><box><size>6 400 0.004</size></box></geometry>")
    xml.append("        </collision>")
    xml.append("      </link>")
    xml.append("    </model>")

    # Central Boulevard North-South (split around roundabout)
    # North section: y = 20 to 200
    xml.append("    <model name='road_conn_central_north'>")
    xml.append("      <static>true</static>")
    xml.append("      <link name='link'>")
    xml.append("        <visual name='visual'>")
    xml.append("          <pose>0 110 0.002 0 0 0</pose>")
    xml.append("          <geometry><box><size>8 180 0.004</size></box></geometry>")
    xml.append("          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Road</name></script></material>")
    xml.append("        </visual>")
    xml.append("        <collision name='collision'>")
    xml.append("          <pose>0 110 0.002 0 0 0</pose>")
    xml.append("          <geometry><box><size>8 180 0.004</size></box></geometry>")
    xml.append("        </collision>")
    xml.append("      </link>")
    xml.append("    </model>")

    # South section: y = -200 to -20
    xml.append("    <model name='road_conn_central_south'>")
    xml.append("      <static>true</static>")
    xml.append("      <link name='link'>")
    xml.append("        <visual name='visual'>")
    xml.append("          <pose>0 -110 0.002 0 0 0</pose>")
    xml.append("          <geometry><box><size>8 180 0.004</size></box></geometry>")
    xml.append("          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Road</name></script></material>")
    xml.append("        </visual>")
    xml.append("        <collision name='collision'>")
    xml.append("          <pose>0 -110 0.002 0 0 0</pose>")
    xml.append("          <geometry><box><size>8 180 0.004</size></box></geometry>")
    xml.append("        </collision>")
    xml.append("      </link>")
    xml.append("    </model>")

    # -------------------------------------------------------------
    # Roundabout at (0,0)
    # -------------------------------------------------------------
    # Circular Road (Asphalt Ring)
    xml.append("    <model name='roundabout_road'>")
    xml.append("      <static>true</static>")
    xml.append("      <link name='link'>")
    xml.append("        <visual name='visual'>")
    xml.append("          <pose>0 0 0.001 0 0 0</pose>")
    xml.append("          <geometry><cylinder><radius>20.0</radius><length>0.002</length></cylinder></geometry>")
    xml.append("          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Road</name></script></material>")
    xml.append("        </visual>")
    xml.append("        <collision name='collision'>")
    xml.append("          <pose>0 0 0.001 0 0 0</pose>")
    xml.append("          <geometry><cylinder><radius>20.0</radius><length>0.002</length></cylinder></geometry>")
    xml.append("        </collision>")
    xml.append("      </link>")
    xml.append("    </model>")

    # Traffic Island (Grass center)
    xml.append("    <model name='roundabout_island'>")
    xml.append("      <static>true</static>")
    xml.append("      <link name='link'>")
    xml.append("        <visual name='visual'>")
    xml.append("          <pose>0 0 0.08 0 0 0</pose>")
    xml.append("          <geometry><cylinder><radius>12.0</radius><length>0.16</length></cylinder></geometry>")
    xml.append("          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grass</name></script></material>")
    xml.append("        </visual>")
    xml.append("        <collision name='collision'>")
    xml.append("          <pose>0 0 0.08 0 0 0</pose>")
    xml.append("          <geometry><cylinder><radius>12.0</radius><length>0.16</length></cylinder></geometry>")
    xml.append("        </collision>")
    xml.append("      </link>")
    xml.append("    </model>")

    # Centerpiece decoration (fountain or statue placeholder)
    xml.append("    <model name='roundabout_statue'>")
    xml.append("      <static>true</static>")
    xml.append("      <pose>0 0 0.16 0 0 0</pose>")
    xml.append("      <link name='link'>")
    xml.append("        <visual name='pedestal'>")
    xml.append("          <pose>0 0 0.5 0 0 0</pose>")
    xml.append("          <geometry><box><size>2.0 2.0 1.0</size></box></geometry>")
    xml.append("          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script></material>")
    xml.append("        </visual>")
    xml.append("        <visual name='art'>")
    xml.append("          <pose>0 0 2.0 0 0 0</pose>")
    xml.append("          <geometry><sphere><radius>1.2</radius></sphere></geometry>")
    xml.append("          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/ClearBlueHighlight</name></script></material>")
    xml.append("        </visual>")
    xml.append("        <collision name='collision'>")
    xml.append("          <pose>0 0 1.5 0 0 0</pose>")
    xml.append("          <geometry><cylinder><radius>1.5</radius><length>3.0</length></cylinder></geometry>")
    xml.append("        </collision>")
    xml.append("      </link>")
    xml.append("    </model>")

    # -------------------------------------------------------------
    # Buildings Spawning
    # -------------------------------------------------------------
    buildings = [
        # name, model, x, y, yaw
        ("admin_building", "academic_building", 0, 90, 3.1415),
        ("library", "academic_building", 120, 90, 3.1415),
        ("academic_a", "academic_building", -120, 90, 0.0),
        ("academic_b", "academic_building", -240, 90, 0.0),
        ("auditorium", "academic_building", 120, -90, 0.0),
        ("cafeteria", "academic_building", -120, -90, 3.1415),
        ("laboratories", "academic_building", -240, -90, 3.1415),
        # Hostel blocks
        ("hostel_a", "campus_house", 260, 110, 0.0),
        ("hostel_b", "campus_house", 260, 150, 0.0),
        ("hostel_c", "campus_house", 310, 110, 3.1415),
        ("hostel_d", "campus_house", 310, 150, 3.1415),
    ]
    for name, model, x, y, yaw in buildings:
        xml.append(f"    <include>")
        xml.append(f"      <name>{name}</name>")
        xml.append(f"      <uri>model://{model}</uri>")
        xml.append(f"      <pose>{x} {y} 0 0 0 {yaw}</pose>")
        xml.append(f"    </include>")

    # -------------------------------------------------------------
    # Campus Features / Pedestrian Plazas
    # -------------------------------------------------------------
    # Library Plaza (paved walkway in front of Library and Admin)
    xml.append("    <model name='library_plaza'>")
    xml.append("      <static>true</static>")
    xml.append("      <link name='link'>")
    xml.append("        <visual name='visual'>")
    xml.append("          <pose>60 45 0.005 0 0 0</pose>")
    xml.append("          <geometry><box><size>200 40 0.002</size></box></geometry>")
    xml.append("          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script></material>")
    xml.append("        </visual>")
    xml.append("        <collision name='collision'>")
    xml.append("          <pose>60 45 0.005 0 0 0</pose>")
    xml.append("          <geometry><box><size>200 40 0.002</size></box></geometry>")
    xml.append("        </collision>")
    xml.append("      </link>")
    xml.append("    </model>")

    # Sports Ground (flat green soccer field)
    xml.append("    <model name='sports_ground'>")
    xml.append("      <static>true</static>")
    xml.append("      <link name='link'>")
    xml.append("        <visual name='visual'>")
    xml.append("          <pose>260 -110 0.003 0 0 0</pose>")
    xml.append("          <geometry><box><size>100 80 0.001</size></box></geometry>")
    xml.append("          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Green</name></script></material>")
    xml.append("        </visual>")
    xml.append("      </link>")
    xml.append("    </model>")

    # -------------------------------------------------------------
    # Security Entrance Gate
    # -------------------------------------------------------------
    xml.append("    <model name='entrance_security_cabin'>")
    xml.append("      <static>true</static>")
    xml.append("      <pose>-380 12 0 0 0 0</pose>")
    xml.append("      <link name='link'>")
    xml.append("        <visual name='cabin'>")
    xml.append("          <pose>0 0 1.5 0 0 0</pose>")
    xml.append("          <geometry><box><size>3 3 3</size></box></geometry>")
    xml.append("          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Wood</name></script></material>")
    xml.append("        </visual>")
    xml.append("        <collision name='collision'>")
    xml.append("          <pose>0 0 1.5 0 0 0</pose>")
    xml.append("          <geometry><box><size>3 3 3</size></box></geometry>")
    xml.append("        </collision>")
    xml.append("      </link>")
    xml.append("    </model>")

    # Parking lot at x = -150, y = 25
    xml.append("    <model name='parking_lot'>")
    xml.append("      <static>true</static>")
    xml.append("      <link name='link'>")
    xml.append("        <visual name='visual'>")
    xml.append("          <pose>-150 25 0.003 0 0 0</pose>")
    xml.append("          <geometry><box><size>40 30 0.002</size></box></geometry>")
    xml.append("          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script></material>")
    xml.append("        </visual>")
    xml.append("        <collision name='collision'>")
    xml.append("          <pose>-150 25 0.003 0 0 0</pose>")
    xml.append("          <geometry><box><size>40 30 0.002</size></box></geometry>")
    xml.append("        </collision>")
    xml.append("      </link>")
    xml.append("    </model>")

    # -------------------------------------------------------------
    # Street Lights (every 40m along roads)
    # -------------------------------------------------------------
    light_id = 0
    # Along Main Road West & East
    for x in range(-380, 390, 45):
        if abs(x) > 25:
            xml.append(f"    <include><name>street_light_main_n_{light_id}</name><uri>model://street_light</uri><pose>{x} 6.0 0 0 0 0</pose></include>")
            xml.append(f"    <include><name>street_light_main_s_{light_id}</name><uri>model://street_light</uri><pose>{x} -6.0 0 0 0 3.1415</pose></include>")
            light_id += 1

    # -------------------------------------------------------------
    # Trees (landscape styling, every 20m along sidewalks)
    # -------------------------------------------------------------
    tree_id = 0
    # Along Main Road
    for x in range(-390, 400, 20):
        if abs(x) > 25:
            xml.append(f"    <include><name>tree_main_n_{tree_id}</name><uri>model://campus_tree</uri><pose>{x} 8.0 0 0 0 0</pose></include>")
            xml.append(f"    <include><name>tree_main_s_{tree_id}</name><uri>model://campus_tree</uri><pose>{x} -8.0 0 0 0 0</pose></include>")
            tree_id += 1
    # Around the Library plaza
    for x in range(30, 200, 25):
        xml.append(f"    <include><name>tree_plaza_{tree_id}</name><uri>model://campus_tree</uri><pose>{x} 60.0 0 0 0 0</pose></include>")
        tree_id += 1

    # -------------------------------------------------------------
    # Benches and Traffic Signs
    # -------------------------------------------------------------
    # Benches at plaza
    bench_id = 0
    for x in range(40, 180, 30):
        xml.append(f"    <include><name>bench_plaza_{bench_id}</name><uri>model://campus_bench</uri><pose>{x} 50.0 0 0 0 0</pose></include>")
        bench_id += 1

    # Stop Signs at intersections
    xml.append("    <include><name>stop_sign_west_intersect</name><uri>model://stop_sign</uri><pose>-355 5 0 0 0 0</pose></include>")
    xml.append("    <include><name>stop_sign_east_intersect</name><uri>model://stop_sign</uri><pose>345 -5 0 0 0 3.1415</pose></include>")
    xml.append("    <include><name>stop_sign_central_north</name><uri>model://stop_sign</uri><pose>5 22 0 0 0 -1.5707</pose></include>")

    # Traffic lights at intersections
    xml.append("    <include><name>traffic_light_4way_north</name><uri>model://traffic_light</uri><pose>5 205 0 0 0 -1.5707</pose></include>")
    xml.append("    <include><name>traffic_light_4way_south</name><uri>model://traffic_light</uri><pose>-5 -205 0 0 0 1.5707</pose></include>")

    # -------------------------------------------------------------
    # Dynamic Pedestrian Actors (Walking students/guards)
    # -------------------------------------------------------------
    # Actor 1: Walking on Central Boulevard
    xml.append("    <actor name='student_walking_boulevard'>")
    xml.append("      <skin><filename>model://walk.dae</filename><scale>1.0</scale></skin>")
    xml.append("      <animation name='walking'><filename>model://walk.dae</filename><scale>1.0</scale><interpolate_x>true</interpolate_x></animation>")
    xml.append("      <script>")
    xml.append("        <loop>true</loop>")
    xml.append("        <auto_start>true</auto_start>")
    xml.append("        <trajectory id='0' type='walking'>")
    xml.append("          <waypoint><time>0</time><pose>4 30 0 0 0 1.5707</pose></waypoint>")
    xml.append("          <waypoint><time>15</time><pose>4 150 0 0 0 1.5707</pose></waypoint>")
    xml.append("          <waypoint><time>17</time><pose>4 150 0 0 0 -1.5707</pose></waypoint>")
    xml.append("          <waypoint><time>32</time><pose>4 30 0 0 0 -1.5707</pose></waypoint>")
    xml.append("          <waypoint><time>34</time><pose>4 30 0 0 0 1.5707</pose></waypoint>")
    xml.append("        </trajectory>")
    xml.append("      </script>")
    xml.append("    </actor>")

    # Actor 2: Crossing the road near Library
    xml.append("    <actor name='student_crossing_library'>")
    xml.append("      <skin><filename>model://walk.dae</filename><scale>1.0</scale></skin>")
    xml.append("      <animation name='walking'><filename>model://walk.dae</filename><scale>1.0</scale><interpolate_x>true</interpolate_x></animation>")
    xml.append("      <script>")
    xml.append("        <loop>true</loop>")
    xml.append("        <auto_start>true</auto_start>")
    xml.append("        <trajectory id='0' type='walking'>")
    xml.append("          <waypoint><time>0</time><pose>120 -6 0 0 0 1.5707</pose></waypoint>")
    xml.append("          <waypoint><time>6</time><pose>120 6 0 0 0 1.5707</pose></waypoint>")
    xml.append("          <waypoint><time>10</time><pose>120 6 0 0 0 -1.5707</pose></waypoint>")
    xml.append("          <waypoint><time>16</time><pose>120 -6 0 0 0 -1.5707</pose></waypoint>")
    xml.append("          <waypoint><time>20</time><pose>120 -6 0 0 0 1.5707</pose></waypoint>")
    xml.append("        </trajectory>")
    xml.append("      </script>")
    xml.append("    </actor>")

    # Actor 3: Walking in library plaza loop
    xml.append("    <actor name='student_plaza_loop'>")
    xml.append("      <skin><filename>model://walk.dae</filename><scale>1.0</scale></skin>")
    xml.append("      <animation name='walking'><filename>model://walk.dae</filename><scale>1.0</scale><interpolate_x>true</interpolate_x></animation>")
    xml.append("      <script>")
    xml.append("        <loop>true</loop>")
    xml.append("        <auto_start>true</auto_start>")
    xml.append("        <trajectory id='0' type='walking'>")
    xml.append("          <waypoint><time>0</time><pose>60 45 0 0 0 0</pose></waypoint>")
    xml.append("          <waypoint><time>10</time><pose>140 45 0 0 0 0</pose></waypoint>")
    xml.append("          <waypoint><time>13</time><pose>140 45 0 0 0 1.5707</pose></waypoint>")
    xml.append("          <waypoint><time>17</time><pose>140 55 0 0 0 3.1415</pose></waypoint>")
    xml.append("          <waypoint><time>27</time><pose>60 55 0 0 0 3.1415</pose></waypoint>")
    xml.append("          <waypoint><time>30</time><pose>60 55 0 0 0 -1.5707</pose></waypoint>")
    xml.append("          <waypoint><time>34</time><pose>60 45 0 0 0 0</pose></waypoint>")
    xml.append("        </trajectory>")
    xml.append("      </script>")
    xml.append("    </actor>")

    # -------------------------------------------------------------
    # Boundary Walls (prevents vehicle falling off, reflects LiDAR scans)
    # -------------------------------------------------------------
    # North Wall
    xml.append("    <model name='boundary_wall_north'>")
    xml.append("      <static>true</static>")
    xml.append("      <link name='link'>")
    xml.append("        <collision name='collision'>")
    xml.append("          <pose>0 220 1.5 0 0 0</pose>")
    xml.append("          <geometry><box><size>820 1 3</size></box></geometry>")
    xml.append("        </collision>")
    xml.append("        <visual name='visual'>")
    xml.append("          <pose>0 220 1.5 0 0 0</pose>")
    xml.append("          <geometry><box><size>820 1 3</size></box></geometry>")
    xml.append("          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script></material>")
    xml.append("        </visual>")
    xml.append("      </link>")
    xml.append("    </model>")

    # South Wall
    xml.append("    <model name='boundary_wall_south'>")
    xml.append("      <static>true</static>")
    xml.append("      <link name='link'>")
    xml.append("        <collision name='collision'>")
    xml.append("          <pose>0 -220 1.5 0 0 0</pose>")
    xml.append("          <geometry><box><size>820 1 3</size></box></geometry>")
    xml.append("        </collision>")
    xml.append("        <visual name='visual'>")
    xml.append("          <pose>0 -220 1.5 0 0 0</pose>")
    xml.append("          <geometry><box><size>820 1 3</size></box></geometry>")
    xml.append("          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script></material>")
    xml.append("        </visual>")
    xml.append("      </link>")
    xml.append("    </model>")

    # East Wall
    xml.append("    <model name='boundary_wall_east'>")
    xml.append("      <static>true</static>")
    xml.append("      <link name='link'>")
    xml.append("        <collision name='collision'>")
    xml.append("          <pose>410 0 1.5 0 0 0</pose>")
    xml.append("          <geometry><box><size>1 440 3</size></box></geometry>")
    xml.append("        </collision>")
    xml.append("        <visual name='visual'>")
    xml.append("          <pose>410 0 1.5 0 0 0</pose>")
    xml.append("          <geometry><box><size>1 440 3</size></box></geometry>")
    xml.append("          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script></material>")
    xml.append("        </visual>")
    xml.append("      </link>")
    xml.append("    </model>")

    # West Wall
    xml.append("    <model name='boundary_wall_west'>")
    xml.append("      <static>true</static>")
    xml.append("      <link name='link'>")
    xml.append("        <collision name='collision'>")
    xml.append("          <pose>-410 0 1.5 0 0 0</pose>")
    xml.append("          <geometry><box><size>1 440 3</size></box></geometry>")
    xml.append("        </collision>")
    xml.append("        <visual name='visual'>")
    xml.append("          <pose>-410 0 1.5 0 0 0</pose>")
    xml.append("          <geometry><box><size>1 440 3</size></box></geometry>")
    xml.append("          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script></material>")
    xml.append("        </visual>")
    xml.append("      </link>")
    xml.append("    </model>")

    # -------------------------------------------------------------
    # Physics Settings
    # -------------------------------------------------------------
    xml.append("    <physics type='ode'>")
    xml.append("      <max_step_size>0.001</max_step_size>")
    xml.append("      <real_time_factor>1.0</real_time_factor>")
    xml.append("      <real_time_update_rate>1000</real_time_update_rate>")
    xml.append("    </physics>")
    
    xml.append("  </world>")
    xml.append("</sdf>")
    
    with open(world_path, "w") as f:
        f.write("\n".join(xml))
    print(f"Generated {world_path} successfully.")

if __name__ == '__main__':
    generate_world()
