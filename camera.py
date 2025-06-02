import cv2
from picamera2 import Picamera2
import numpy as np
import cv2.aruco as aruco
import time
import sysv_ipc
import struct
import math


# Shared memory key must match what your C/C++ code uses
key = 123456
size_test=len( struct.pack('iiiii',1,1,1,1,1))
# Create or attach to the shared memory segment
memory = sysv_ipc.SharedMemory(key, sysv_ipc.IPC_CREAT, size=size_test)


# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6
 
# Calculates rotation matrix to euler angles
def rotationMatrixToEulerAngles(R) :
 
    assert(isRotationMatrix(R))
 
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
 
    singular = sy < 1e-6
 
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
 
    return np.array([x, y, z])

print("starting camera")
# Initialize Picamera2
picam2 = Picamera2()

# Configure the camera
picam2.preview_configuration.main.size = (728, 544)  # Adjust size as needed
picam2.preview_configuration.main.format = "RGB888" # or "XRGB8888"
picam2.preview_configuration.align()
picam2.configure("preview")

# Start the camera
picam2.start()


# Now set manual exposure and gain
picam2.set_controls({
    "ExposureTime": 15000,   # 5 ms
    "AnalogueGain": 4.0
})

time.sleep(0.5)  # Let settings settle
prev = time.time()


# Camera calibration parameters 
camera_matrix = np.array([[860.4784, 0, 329.6632],       # fx, 0, cx
                          [0, 860.8377, 278.2514],       # 0, fy, cy
                          [0, 0, 1]], dtype=np.float32)  # 0, 0, 1
dist_coeffs =  np.array([-0.5735, 0.4192, -0.00216, 0.00507, -0.5190])


# Marker size (in meters)
marker_length = 200  # 5 cm marker size

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters_create()


 #some global variables   
dt=0.0
prev=0.0
sequence_num=0

	
while True:
    # Capture frame as a NumPy array
    current=time.time()

    #grab image from camera
    frame = picam2.capture_array()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
     
    #detect the marker in the image
    corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict,parameters=parameters)

    
    #for all markes
    if ids is not None:

        # Get the 3D coordinates of the marker corners
        marker_corners = np.array([[0, 0, 0],           # Top-left corner
                                   [marker_length, 0, 0], # Top-right corner
                                   [marker_length, marker_length, 0], # Bottom-right corner
                                   [0, marker_length, 0]], dtype=np.float32)  # Bottom-left corner

        #draw markers in frame, turn on for debugging
        #aruco.drawDetectedMarkers(frame,corners)



        
        for i in range(len(ids)):
            # Use solvePnP to get the rotation and translation vectors
            retval, rvec, tvec = cv2.solvePnP(marker_corners, corners[i], camera_matrix, dist_coeffs)

          
            
            #turn on for debuggin
            #cv2.drawFrameAxes(frame,camera_matrix,dist_coeffs,rvec,tvec,100)

            rvec_flipped = rvec*-1
            tvec_flipped = tvec*-1
            rotation_matrix, _ = cv2.Rodrigues(rvec_flipped)
            realworld_tvec=np.dot(rotation_matrix,tvec_flipped)
    
            pitch,roll,yaw=rotationMatrixToEulerAngles(rotation_matrix)


           
            dt=current-prev           
            prev=current

            print("x=%4.0f y=%4.0f z=%4.0f yaw=%4.0f dt=%4.8f %d" % (realworld_tvec[0], realworld_tvec[1],realworld_tvec[2], math.degrees(yaw), dt,sequence_num))
            

            packed_data = struct.pack('iiiii', int(realworld_tvec[0]),int(realworld_tvec[1]),int(realworld_tvec[2]), int(math.degrees(yaw)),int(sequence_num))  # 2 ints, 1 float
            
            # Write to shared memory
            memory.write(packed_data)
            sequence_num=sequence_num+1
            
           
                    
               
    
   
    # Display the frame using OpenCV, turn on for debugging 
    #cv2.imshow("Camera Feed", frame)

    #save the image, turn on for debugging
    #filename = f"image_test.jpg"
    #cv2.imwrite(filename, frame)
    #print(f"Image saved as {filename}")

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
picam2.stop()
cv2.destroyAllWindows()
