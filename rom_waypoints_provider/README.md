### rom waypoints provider

#### Run Once
```
ros2 run my_nav2_package send_waypoints room1 corner1
```

#### Enable loop mode
```
ros2 run my_nav2_package send_waypoints --loop

```


#### Loop specific waypoints
```
ros2 run my_nav2_package send_waypoints room1 corner1 --loop
```