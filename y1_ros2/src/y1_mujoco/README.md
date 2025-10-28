# 依赖项
  - Ubuntu 22.04 LTS
  - ROS Humble

# Mujoco Simulation

## 1. Installing Mujoco210 and mujoco-py

### 1.1 Install Mujoco

1. [Download Mujoco210](https://github.com/google-deepmind/mujoco/releases/download/2.1.0/mujoco210-linux-x86_64.tar.gz)

2. Extract the files

    ```bash
    mkdir ~/.mujoco
    cd (Directory containing the package)
    tar -zxvf mujoco210-linux-x86_64.tar.gz -C ~/.mujoco
    ```

3. Add the environment variable

    ```bash
    echo "export LD_LIBRARY_PATH=~/.mujoco/mujoco210/bin:\$LD_LIBRARY_PATH" >> ~/.bashrc
    source ~/.bashrc
    ```

4. Test the installation

    ```bash
    cd ~/.mujoco/mujoco210/bin
    ./simulate ../model/humanoid.xml
    ```

### 1.2 Install mujoco-py

1. Install dependencies
    ```bash
    pip install mujoco_py
    sudo apt-get install libglew-dev patchelf libosmesa6-dev libgl1-mesa-glx
    ```
2. Add the environment variable

    ```bash
    echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/nvidia" >> ~/.bashrc
    echo "export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so" >> ~/.bashrc
    source ~/.bashrc
    ```

## Y1 Mujoco Simulation (Without Gripper)

### 1. Run Mujoco simulation

    ```bash
    source install/setup.bash
    ros2 run y1_mujoco y1_with_rubber.py
    ```

### 2. Control Y1 arm without the gripper via RViz GUI (Run in a new terminal)

    ```bash
    source install/setup.bash
    ros2 launch y1_description display_y1_with_rubber.launch
    ```