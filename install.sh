#!/bin/bash
echo "╔══╣ Setup: gz human sim (STARTING) ╠══╗"


# Add Gazebo models to GZ_SIM_RESOURCE_PATH
echo "export GZ_SIM_RESOURCE_PATH=\${GZ_SIM_RESOURCE_PATH}:\${HOME}/colcon_ws/install/gz_human_sim/share" >> ~/.bashrc

# Install Gazebo Harmonic
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install -y \
    gz-harmonic


echo "╚══╣ Setup: gz human sim (FINISHED) ╠══╝"
