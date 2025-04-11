<div style="text-align: center;" align="center">
  <img src="docs/figs/.svg" alt="" width="600"/>
  <h1> sense_eeg </h1>
</div>

## :eyeglasses: Overview
This repository contains documentation and code for EEG. 

## :school_satchel: Getting started
* :computer: [Setting up ROS2 with docker container](https://github.com/sense-base/base/tree/main/docs/docker)

## :octocat: Cloning repository and contribute to it
* Generate your SSH keys as suggested [here](https://docs.github.com/en/github/authenticating-to-github/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent)
* Setup you commit signature verification as shown [here](https://docs.github.com/en/authentication/managing-commit-signature-verification/about-commit-signature-verification#ssh-commit-signature-verification)
* Clone the repository by typing (or copying) the following lines in a terminal
```
git clone git@github.com:sense-base/sense_eeg.git
```
* Refer to the [CONTRIBUTING](CONTRIBUTING.md) guideline for detailed instructions on contributing to this repo.

## Run
Open a terminal into the loaded container in VSCode using the dev containers extension, and run

```
colcon build --symlink-install
source install/setup.bash
ros2 launch eeg_publisher mock_publisher_launch.py
```

On a different terminal, run

```
source install/setup.bashß
ros2 topic list
ros2 topic echo /eeg/raw
```

### Run Pre-commit Hooks
```bash
pre-commit run --all-files
```
