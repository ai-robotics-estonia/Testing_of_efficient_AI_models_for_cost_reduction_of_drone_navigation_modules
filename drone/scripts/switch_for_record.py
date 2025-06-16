import RPi.GPIO as GPIO
import subprocess
import time

SWITCH_GPIO_PIN = 4
GPIO.setmode(GPIO.BCM)
GPIO.setup(SWITCH_GPIO_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

SESSION_NAME = "rosbag_rec"
BAGS_DIRECTORY = "/home/rpi5/ros2_jazzy/src/drone/camera_pkgs/bags/"        # Directory where the bags are located - modify if needed
TMUX_PATH = "/usr/bin/tmux"


previous_state = GPIO.input(SWITCH_GPIO_PIN)
print(f"Initial state: {previous_state}")

try:
    while True:
        current_state = GPIO.input(SWITCH_GPIO_PIN)
        
        if current_state == 0 and previous_state == 1:
            BAG_NAME = f"data_{time.time()}"
            BAG_RECORD_COMMAND = f"ros2 bag record -o {BAGS_DIRECTORY+BAG_NAME} --topics /barometer /cam/bottom/image /cam/front/image /gpsfix /heading /imu /lidar /magnetic_field /robot_description /tf /tf_static"
            
            subprocess.run([TMUX_PATH, "new-session", "-d", "-s", SESSION_NAME, "bash"])
            subprocess.run([TMUX_PATH, "send-keys", "-t", SESSION_NAME, BAG_RECORD_COMMAND, "C-m"])
            
            print("switch ON --> BAG recordign started")
        
        elif current_state == 1 and previous_state == 0:
            
            subprocess.run([TMUX_PATH, "send-keys", "-t", SESSION_NAME, "C-c"])
            time.sleep(2)
            subprocess.run([TMUX_PATH, "kill-session", "-t", SESSION_NAME])
            
            print("switch OFF --> BAG recording stopped")
        
        previous_state = current_state
        
        time.sleep(0.2)

except KeyboardInterrupt:
    print("Keyboard interrupt. Exiting ....")
    
finally:
    GPIO.cleanup()
