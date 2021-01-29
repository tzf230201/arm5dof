## how to use

1.open simulator
```bash
roslaunch arm5dof sim.launch
```

2.run controller
```bash
rosrun arm5dof controller
```
3.input target coordinate x,y and z
```bash
rostopic pub -1 /arm5dof/input geometry_msgs/Vector3  '{x: 100.0,y: 0.0,z: 0.0}'
```


