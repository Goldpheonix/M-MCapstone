import cv2

# For USB Webcam (use 0 or another index if there are multiple cameras)
cap = cv2.VideoCapture(0)

# For Raspberry Pi Camera Module (CSI camera) – make sure it's enabled in raspi-config
# cap = cv2.VideoCapture('video4linux2')  # Uncomment for Pi Camera Module

if not cap.isOpened():
    print("Error: Could not open camera.")
else:
    print("Camera is working!")

    while True:
        # Capture a frame from the webcam
        ret, frame = cap.read()

        if not ret:
            print("Failed to capture frame.")
            break
        
        # Display the captured frame in a window
        cv2.imshow("Webcam", frame)

        # Wait for the 'q' key to be pressed to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Exiting...")
            break

    # Release the webcam and close the window
    cap.release()
    cv2.destroyAllWindows()
