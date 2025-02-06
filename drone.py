import numpy as np
from ultralytics import YOLO
from insightface.app import FaceAnalysis
from sklearn.metrics.pairwise import cosine_similarity
from picamera2 import Picamera2
from libcamera import Transform
import smbus2 , time, math , cv2
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
import time, math, cv2, dlib
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)

vehicle = connect("/dev/ttyAMA0", baud= 57600 ,wait_ready=True, timeout = 90)
print("Connection Established")

vehicle.mode = VehicleMode("GUIDED")
while vehicle.mode!= "GUIDED":
    print("Waiting foir mode change..")
    time.sleep(1)

while not vehicle.is_armable:
    print("Waiting for vehicle to initialize...")
    time.sleep(1)

vehicle.armed = True
while not vehicle.armed:
    print("Waiting for vehicle to be armed...")
    time.sleep(1)
print("Vehicle Armed")
time.sleep(2)

vehicle.simple_takeoff(2)
while vehicle.location.global_relative_frame.alt < 0.95 * 2:
    print(vehicle.location.global_relative_frame.alt)
    time.sleep(0.5)
    
time.sleep(3)

def NED_velocity(vX,vY,vZ):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
    0,
    0,0,
    mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, 
    3527, # Bitmask for Position + Velocity
    0,0,0, # Position
    vX,vY,vZ, # Velocity
    0,0,0, # acceleration
    0,0
)
    vehicle.send_mavlink(msg)
def NED(X,Y,Z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0,0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, 
        3576, # Bitmask for Position + Velocity
        X,Y,Z, # Position
        0,0,0, # Velocity
        0,0,0, # acceleration
        0,0
    )
    vehicle.send_mavlink(msg)

def YAW(angle):
    if angle < 0:
        heading = -1
        angle = angle * -1
    else:
        heading = 1
    msg = vehicle.message_factory.command_long_encode(
        0,0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,    
        0,
        angle,
        20, # speed Deg/s
        heading, # CCW or CW
        1,  # Relative or Absolute
        0,0,0
    )
    vehicle.send_mavlink(msg)
    

p_error_x = 0
print('Uploading model to ram')
#model = YOLO("/home/pi/Desktop/drone/drone-test/tree2_ncnn_model")                           #best till now
model = YOLO("/home/pi/Desktop/drone/drone-test/yolov8n_ncnn_model")
# I2C bus and TF-Luna I2C address
I2C_BUS = 1
I2C_ADDRESS = 0x10  # Replace with your detected address
frame_width =1216
frame_height = 1216
# Register addresses (refer to TF-Luna documentation)
DIST_L = 0x00  # Low byte of distance
DIST_H = 0x01  # High byte of distance

# Initialize I2C bus
bus = smbus2.SMBus(I2C_BUS)
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (frame_height, frame_width)})
config["transform"] = Transform( vflip=1)
picam2.configure(config)
picam2.start()
center_area_max = 320 + 40
center_area_min = 320 - 40
fps = 10
out = cv2.VideoWriter('test1.mp4', cv2.VideoWriter_fourcc(*'mp4v'), fps, (640, 640))


detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor('shape_predictor_68_face_landmarks.dat')
face_rec_model = dlib.face_recognition_model_v1('dlib_face_recognition_resnet_model_v1.dat')
known_face_encodings = np.load('Abdullah.npy', allow_pickle=True)
known_face_names = ['abdullah', 'Unknown']  



# Function to compute the Euclidean distance between two encodings
def euclidean_distance(encoding1, encoding2):
    return np.linalg.norm(encoding1 - encoding2)


def read_distance():
    try:
        # Read two bytes from the distance registers
        dist_low = bus.read_byte_data(I2C_ADDRESS, DIST_L)
        dist_high = bus.read_byte_data(I2C_ADDRESS, DIST_H)
       
        # Combine high and low bytes to get distance in cm
        distance = (dist_high << 8) + dist_low
        return distance
    except Exception as e:
        print(f"Error reading data: {e}")
        return None


def set_x(x):
    x=x
    global p_error_x
    kp = 0.08
    kd = 0.005
    comand = ''
    set_point = 320  # Desired center point on the x-axis
    error = set_point - x
    deg = kp * error + kd * (error - p_error_x)
    deg = int(np.clip(deg, -20, 20))  # Limit speed
    p_error_x = error
    if x >= center_area_max:
        print("move left")
        YAW(deg)
    elif x <= center_area_min:
        print("move right")
        YAW(deg)
    else:
        print("object is in center x")
        comand = "center x"
    return comand
   
def set_y(y):
    y=y
    comand = ''
    if y <= center_area_min:
        print("move up")
        NED_velocity(0,0,-0.2)
    elif y >= center_area_max:
        print("move down")
        NED_velocity(0,0,0.2)
    else:
        print("object is in center y")
        comand="center y"
    return comand
   
   
   
def position_set(distance ,x ,y ):
    comand = ''
    comand = set_y(int(y))
    comand =set_x(int(x))
    distance_u1 = ultrasonic(trig1 ,echo1) #front
    distance_u2 = ultrasonic(trig2 ,echo2) #ultrasonic(trig2 ,echo2) #left
    distance_u3 =ultrasonic(trig3 ,echo3)  #ultrasonic(trig3 ,echo3) #right
    print(f"ultra sonic 1 {distance_u1}")
       
    if distance_u1 <= ultra_range :
        print("move backward ultra 1")
        NED_velocity(-0.5,0,0)
        
    elif distance_u2 <= ultra_range :
        print("move right ultra 2")
        NED_velocity(0,0.5,0)
    elif distance_u3 <= ultra_range :
        print("move left ultra 3")
        NED_velocity(0,-0.5,0)
    else:
        if (x<=center_area_max)&(x>=center_area_min)&(y<=center_area_max)&(y>=center_area_min): #means object is at center then move
            if (distance < 150)&(distance!=0):
                print("move backward")
                NED_velocity(-0.5,0,0)
            elif (distance>=150)&(distance <=250):
                print("stop")
                NED_velocity(0,0,0)
            else:
                print("move forward")
                NED_velocity(0.5,0,0)
            
                            
def detection_yolo(frame, class_ ,distance_disp):
    results = model.track(source=frame, conf=0.7, classes=0 )  
    if results:
        text_size = cv2.getTextSize(distance_disp, font, font_scale, thickness)[0]
        text_x = width - text_size[0] - 10  # Offset 10px from right
        text_y = text_size[1] + 10  # Offset 10px from top
        frame = cv2.putText(frame, distance_disp, (text_x, text_y), font, font_scale, color, thickness)
        frame = results[0].plot()
        out.write(frame)
    else:
        NED_velocity(0,0,0)
    detections = []
    ids = []
    if results and len(results) > 0 and results[0].boxes is not None:
        detections = results[0].boxes.xyxy.cpu().numpy()
        if results[0].boxes.id is not None:
            ids = results[0].boxes.id.int().cpu().tolist()
    return detections, ids

def ultrasonic(TRIG ,ECHO):
    # Send a 10us pulse to the trigger
    GPIO.output(TRIG, GPIO.LOW) 
    pulse_strt =0
    pulse_ed =0   
    GPIO.output(TRIG, GPIO.HIGH)
    time.sleep(0.00001)  
    GPIO.output(TRIG, GPIO.LOW)
    while GPIO.input(ECHO) == GPIO.LOW:
        pulse_strt = time.time()
    start_timer = time.time()  # Start the timer after echo goes HIGH
    total_time = 0
    
    # Wait for the echo pin to go LOW or for timeout to reach 0.01 seconds
    while GPIO.input(ECHO) == GPIO.HIGH and total_time <= 0.011:
        pulse_ed = time.time()
        total_time = pulse_ed - start_timer 
    # Calculate the distance in cm
    pulse_duration = pulse_ed - pulse_strt
    distance = pulse_duration * 17150  # Speed of sound is 34300 cm/s, divided by 2 for round-trip
    #distance = round(distance, 2)  # Round to 2 decimal places
    return distance

trig1= 5
echo1= 6
GPIO.setup(trig1, GPIO.OUT)
GPIO.setup(echo1, GPIO.IN)
trig2=25
echo2=16
GPIO.setup(trig2, GPIO.OUT)
GPIO.setup(echo2, GPIO.IN)
trig3=24
echo3=23
GPIO.setup(trig3, GPIO.OUT)
GPIO.setup(echo3, GPIO.IN)
global font_scale , color , thickness ,font ,height ,width ,ultra_range
ultra_range = 50

font_scale = 1
color = (0, 255, 0)  # Green color
thickness = 2
font = cv2.FONT_HERSHEY_SIMPLEX
height, width = 640 ,640
def main():
    exit_= 1
    face_detected = False
    global id_
    while exit_:
        try :
            while not face_detected:
                try:
                    main_img = picam2.capture_array()
                    frame = cv2.resize(main_img, (640, 640))
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    distance = read_distance()
                    distance_disp = f"distance {distance}cm"
                    
                    # Run YOLOv8 for person detection
                    detections ,ids =detection_yolo(frame ,1 ,distance_disp)
                    
                    if detections.size > 0:
                        for box ,id_ in zip(detections ,ids):
                            try:
                                

                                x1, y1, x2, y2   = map(int, box[:4])
                                x1_percent =x1/640
                                x2_percent =x2/640
                                y1_percent =y1/640
                                y2_percent =y2/640
                                x1 = int(x1_percent * 1200)
                                x2 = int(x2_percent * 1200)
                                y1 = int(y1_percent * 1200)
                                y2 = int(y2_percent * 1200)
                                
                                id_ = id_
                                face_crop = main_img[y1:y2, x1:x2] 
                                
                            
                                face_crop = cv2.cvtColor(face_crop, cv2.COLOR_RGB2BGR)
                                
                                gray_face = cv2.cvtColor(face_crop, cv2.COLOR_BGR2GRAY)  
                                
                            
                                
                                faces = detector(gray_face)
                                for face in faces:
                                #rect = dlib.rectangle(x1, y1, x2, y2)  
                                    #face = max(face, key=lambda f: f.width() * f.height())
                                    landmarks = predictor(gray_face, face) 
                                    
                                    encoding = face_rec_model.compute_face_descriptor(frame, landmarks)

                                    if encoding is not None:
                                        print(f"Encoding shape: {np.array(encoding).shape}")
                                        
                                        # Compare the encoding with the known encoding (only one known face)
                                        distance = euclidean_distance(encoding, known_face_encodings[0])
                                        print ('distance is',distance)
                                        # Set a threshold for the match
                                        threshold = 0.7
                                        
                                        # Check if the distance is below the threshold (i.e., it's a match)
                                        if distance < threshold:
                                            print('id is',id_)
                                            name = known_face_names[0]  # Since there's only one known face
                                            face_detected = True
                                            break
                                        else:
                                            name = "Unknown"
                                    else:
                                        print("Face encoding not found!")
                                        name = "Unknown"
                                    '''
                                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                                    cv2.putText(frame, name, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)'''
                                
                            except KeyboardInterrupt:
                                print("Exiting... k recog on box  / calling RTL")
                                exit_ =0
                                NED(0,0,-2)
                                vehicle.mode = VehicleMode("RTL")
                                break
                            except Exception as e:
                                print(f"Exiting... 1  {e}")
                                NED(0,0,-2)
                                vehicle.mode = VehicleMode("RTL")
                                exit_ =0
                                break
                            if face_detected:
                                cv2.destroyAllWindows()
                                break 
                    if face_detected :
                        cv2.destroyAllWindows()
                        break 
                        # Draw the bounding box and put the name label
                except KeyboardInterrupt:
                    print("Exiting...k recog/detect / calling RTL")
                    exit_ =0
                    NED(0,0,-2)
                    vehicle.mode = VehicleMode("RTL")
                    break
                except Exception as e:
                    print(f"Exiting...2  {e}")
                    NED(0,0,-2)
                    vehicle.mode = VehicleMode("RTL")
                    exit_ =0
                    break
            
            id_to_track = int(id_)
            #cv2.imshow("recognition", frame)
            print(f"starting to track abdullagh with id {id_to_track}")
            
            while True:
                start_time =time.time()
                try:
                    frame = picam2.capture_array()
                    frame = cv2.resize(frame, (640, 640))
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    results = model.track(source=frame , conf=0.7 , classes=0 )  
                    if not results[0].boxes.data.tolist():
                        distance_u1 = ultrasonic(trig1 ,echo1) #front
                        distance_u2 = ultrasonic(trig2 ,echo2)#ultrasonic(trig2 ,echo2) #left
                        distance_u3 =ultrasonic(trig3 ,echo3) #ultrasonic(trig3 ,echo3) #right
                        if distance_u1 <= ultra_range :
                            print("move backward ultra 1")
                            NED_velocity(-0.5,0,0)
                        elif distance_u2 <= ultra_range :
                            print("move right ultra 2")
                            NED_velocity(0,0.5,0)
                        elif distance_u3 <= ultra_range :
                            print("move left ultra 3")
                            NED_velocity(0,-0.5,0)
                        else:
                            NED_velocity(0,0,0) #hold position
                            
                    text_size = cv2.getTextSize(distance_disp, font, font_scale, thickness)[0]
                    text_x = width - text_size[0] - 10  
                    text_y = text_size[1] + 10 
                    distance = read_distance()
                    distance_disp = f"distance {distance}cm"
                    frame = cv2.putText(frame, distance_disp, (text_x, text_y), font, font_scale, color, thickness)
                    frame = results[0].plot()
                    out.write(frame)
                   
                    for obj in results[0].boxes.data.tolist():
                        try:
                            x, y, w ,h ,track_id , conf,class_  =obj
                            center_x = (x + w )// 2
                            center_y = (y + h) // 2
                            if id_to_track == int(track_id):
                                print(f"tracking id is {track_id}")
                                distance = read_distance()
                                position_set(distance ,center_x ,center_y) 
                                
                        except KeyboardInterrupt:
                            print("Exiting... tracker inner/ calling RTL")
                            exit_ =0
                            NED(0,0,-2)
                            vehicle.mode = VehicleMode("RTL")
                            break
                        except Exception as e:
                            print(f"Exiting... tracker inner/ calling RTL  {e}")
                            exit_ =0
                            NED(0,0,-2)
                            vehicle.mode = VehicleMode("RTL")
                            break
                   
                except KeyboardInterrupt:
                    print("Exiting... tracker / calling RTL")
                    exit_ =0
                    NED(0,0,-2)
                    vehicle.mode = VehicleMode("RTL")
                    break
                except Exception as e:
                    print(f"Exiting... tracker / calling RTL  {e}")
                    exit_ =0
                    NED(0,0,-2)
                    vehicle.mode = VehicleMode("RTL")
                    break
                end =time.time()
                total_time = end - start_time
                print(f'total time taken {total_time}')
        except KeyboardInterrupt:
            print("Exiting... k main / calling RTL")
            exit_ =0
            NED(0,0,-2)
            vehicle.mode = VehicleMode("RTL")
            break
        except Exception as e:
            print(f"Exiting...3 {e}")
            NED(0,0,-2)
            vehicle.mode = VehicleMode("RTL")
            exit_ =0
            break

        # Exit on pressing 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
main()
picam2.stop()
bus.close()
out.release()
cv2.destroyAllWindows()
