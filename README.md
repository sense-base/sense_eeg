<div style="text-align: center;" align="center">
  <img src="docs/figs/.svg" alt="" width="600"/>
  <h1> sense_eeg </h1>
</div>

## :eyeglasses: Overview
This repository contains documentation and code for EEG packages. 

## :school_satchel: Getting started
* :computer: [Setting up ROS2 with docker container](https://github.com/sense-base/base/tree/main/docs/docker)

* :octocat: Clone repo under `sense-base` path and refer to the [CONTRIBUTING](CONTRIBUTING.md) guideline for detailed instructions on contributing to this repo.
```
git clone git@github.com:sense-base/sense_eeg.git
```

* :nut_and_bolt: Run and debug. Open a terminal into the loaded container in VSCode using the dev containers extension, and run
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
ros2 bag record /eeg/raw
ros2 bag info <bag_file_name> (e.g., `rosbag2_data_time`)
ros2 bag play <bag_file_name>
rqt_graph
```

## Setup of hardware 
![fig](docs/hardware-connecton-gtech.svg)
The diagram shows a general setup with the g.tec USBamp, a Linux computer, cables, and electrode connections.

## Setup environment with pyenv

Install Python (3.10 or higher recommended):
```bash
pyenv install 3.10.17 
```

```bash
pyenv virtualenv 3.10.17 sense_EEG-env
pyenv local sense_EEG-env
pip install -e ".[dev]"
```

### Run Pre-commit Hooks
```bash
pre-commit run --all-files
```
