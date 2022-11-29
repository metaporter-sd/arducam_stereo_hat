# crude script to record video from camera
import os

def record(): 
	cmd = "nvgstcapture-1.0 \
		--sensor-id=0 \
		--mode=2 \
		--video-res=6 \
		--framerate=10 \
		--orientation=2 \
		--automate \
		--capture-auto \
		--file-name=./data/test_video \
		--capture-time=15 \
		"
	os.system(cmd)
	
if __name__ == "__main__":
	record()
