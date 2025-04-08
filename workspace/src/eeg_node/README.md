# Run
Open a terminal into the loaded container in VSCode using the dev containers extension, and run

```
colcon build --symlink-install
source install/setup.bash
ros2 launch eeg_publisher mock_publisher_launch.py
```

On a different terminal, run

```
source install/setup.bash
ros2 topic list
ros2 topic echo /eeg/raw
```