import cv2
import time

# Open the default camera
cap = cv2.VideoCapture(0)

width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open video capture.")
    exit()

num_frames_to_capture = 100  # Number of frames to capture
print("Capturing {0} frames.".format(num_frames_to_capture))

# Start time
start = time.time()

# Capture the frames
for _ in range(num_frames_to_capture):
    ret, frame = cap.read()
    if not ret:
        print("Error: Can't receive frame (stream end?). Exiting ...")
        break
    cv2.imshow('Frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit
        break

# End time
end = time.time()

# Time elapsed
seconds = end - start
fps  = num_frames_to_capture / seconds

print("Estimated frames per second : {0}".format(fps))
print("width; " ,width)
print("height:",height)

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

