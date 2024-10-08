
#!/usr/bin/env python3

# Import the subprocess and time modules
import subprocess
import time

# List of commands to run
commands = [
    # Run the Micro XRCE-DDS Agent
    "MicroXRCEAgent udp4 -p 8888",

    # Run the PX4 SITL simulation
    "cd ~/Documents/projects/DroneProject/PX4-Autopilot && make px4_sitl gz_x500",
    
    # Run QGroundControl
    "cd ~/Documents/projects/DroneProject && ./QGroundControl.AppImage",

    #Source the ros2 environment to a new terminal
    "source /opt/ros/humble/setup.bash && source install/local_setup.bash",
]

# Loop through each command in the list
for command in commands:
    # Each command is run in a new tab of the gnome-terminal
    subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", command + "; exec bash"])
    
    # Pause between each command
    time.sleep(1)
