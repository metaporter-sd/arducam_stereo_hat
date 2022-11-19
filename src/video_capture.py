# capture video from Arducam stereo hat 
import numpy as np
import cv2

def gstreamer_pipeline(capture_width=2592, capture_height=1944, display_width=1440,
    display_height=1080,
    framerate=10,
    flip_method=2,
):
	#credit: JetsonHacks
    return (
		"nvarguscamerasrc sensor-id=0 ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )
 
def run():
	
	print("start\n")
	print(gstreamer_pipeline())
	print("\nstop\n")
	cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
	if not cap.isOpened():
		print("Cannot open camera")
		exit()
	"""
	try:
		width = 1280
		# set width
		cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
	
		height = 400
		# set height
		cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
	except:
		pass
	print("Set frame size")
	"""

	while True:
		# Capture frame-by-frame
		ret, frame = cap.read()
		# if frame is read correctly ret is True
		if not ret:
			print("Can't receive frame (stream end?). Exiting ...")
			break
		# Our operations on the frame come here
		#gray = cv2.cvtColor(frame, cv.COLOR_BGR2GRAY)
	
		# Display the resulting frame
		cv2.imshow('frame', frame)
		keyCode = cv2.waitKey(30) & 0xFF
        # Stop the program on the ESC key
		if keyCode == 27:
			break
	
	# When everything done, release the capture
	cap.release()
	cv2.destroyAllWindows()

if __name__ == "__main__":
	run()
