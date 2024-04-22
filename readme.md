## Getting Started (Tested with ROS Noetic):
1. Use the installer from the [DepthAI Installation Guide](https://docs.luxonis.com/en/latest/pages/tutorials/first_steps/): <br />
  ``` sudo wget -qO- https://docs.luxonis.com/install_depthai.sh | bash ```
2. Connect the OAK-D camera to a USB 3.0 port and run the DepthAI viewer: <br />
  ``` depthai-viewer ``` or ``` python3 -m depthai_viewer ```
3. After confirming DepthAI is installed successfully, install SpectacularAI and SciPy: <br />
  ``` pip install spectacularai scipy ```
4. Clone this repository into a your Catkin Worskpace /src and catkin_make <br />
  ``` git clone https://github.com/BrenMun/spectacular_ai_ros.git```
6. To run the wrapper, use the following command: <br />
  ``` rosrun spectacular_ai_wrapper oakd_spectacularai_node.py ```
7. To run the wrapper alongside the visualiser, use the launch file: <br />
  ``` roslaunch spectacular_ai_wrapper oak_spectacularai.launch ```
8. The file "oakd_config.yaml" located in the "/config" directory is used to configure parameters for the wrapper
