<div style="text-align: center;" align="center">
  <img src="docs/figs/.svg" alt="" width="600"/>
  <h1> sense_eeg </h1>
</div>

## :eyeglasses: Overview
This repository contains documentation and code for EEG packages. 

## :school_satchel: Getting started
* :computer: [Setting up ROS2 with docker container](https://github.com/sense-base/base/tree/main/docs/docker)

* :octocat: Clone repo under and refer to the [CONTRIBUTING](CONTRIBUTING.md) guideline for detailed instructions on contributing to this repo.
```
git clone git@github.com:sense-base/sense_eeg.git
```

* :nut_and_bolt: Run and debug. Open a terminal into the loaded container in VSCode using the dev containers extension, and run
```
source /opt/ros/humble/setup.bash
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
