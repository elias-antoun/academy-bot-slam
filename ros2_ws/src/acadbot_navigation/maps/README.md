# Maps

This folder holds the maps you build in **Session 2** and reuse in **Session 4**.

## Saving a map after SLAM (two formats)

After driving the robot around with `slam_toolbox` running, save the map:

**1. Occupancy-grid format (for AMCL / map_server):**
```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/acadbot_navigation/maps/academy_map
```
This creates `academy_map.yaml` + `academy_map.pgm`.

**2. slam_toolbox serialized format (for slam_toolbox localization mode):**
In the RViz **SlamToolbox** plugin panel, click **Serialize Map**, or call the
service, saving to:
```
academy_map_serial   ->   academy_map_serial.posegraph + academy_map_serial.data
```

`navigation.launch.py localization:=slam` expects the serialized map;
`navigation.launch.py localization:=amcl map:=.../academy_map.yaml` expects the
occupancy grid.
