#from Vision import *
from Mobility import *
#from Collector import *
import RPi.GPIO as GPIO
import timeit
import numpy as np
import cv2 as cv
import time
import math


# global variables in SI units
FOV = 62.2
focal = 3.04E-3
rockw = 70E-3
obstaclew = 150E-3
sampled = 42.66E-3
landerheight = 70E-3
wallheight = 450E-3
frame_length = 320 # 1280
frame_width = 480 # 720

prevLoc = [0,0]
freq = 20
Interval = 1/freq
pixelwidth = 1.12E-6
pixelheight = 1.12E-6
lowVel = 10
requiredVel = [0,0]
constVal = 0.8
stateSwitch = False
collectionAmount = 0

state = 0
# state machine
# state 0: locate or relocate sample
# state 1: collect sample
# state 2: collect rock
# state 3: move to next sample/rock

motion_counter = 0
rotation_counter = 0

class sample_col():
    def __init__(self):
        self.pin = 14
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.OUT)
        self.servo = GPIO.PWM(self.pin, 50)
        self.servo.start(0)
    
    def grab_sample(self):
        self.servo.ChangeDutyCycle(2 + 80/18);
        time.sleep(0.5)
        self.servo.ChangeDutyCycle(0);

    def release_sample(self):
        self.servo.ChangeDutyCycle(2 + 35/18);
        time.sleep(0.5)
        self.servo.ChangeDutyCycle(0);

    def close_claws(self):
        self.servo.ChangeDutyCycle(2 + 60/18);
        time.sleep(0.5)
        self.servo.ChangeDutyCycle(0);

    def lift_rock(self):
        self.servo.ChangeDutyCycle(2 + 120/18);
        time.sleep(0.5)
        self.servo.ChangeDutyCycle(0);

    def stop(self):
        self.servo.stop()
        GPIO.cleanup(self.pin)
        
sc = sample_col()

# input - used to calculate bearing (linear interpolation)
def mapping_linear(value, leftMin, leftMax, rightMin, rightMax):
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin
    valueScaled = float(value - leftMin) / float(leftSpan)
    return rightMin + (valueScaled * rightSpan)

class Object_Detection:
    def find_samples(self, hsv_frame):
        # mask of green (36,0,0) ~ (70, 255,255)
        mask1 = cv.inRange(hsv_frame, (0, 91, 39), (10, 255, 255))
        mask2 = cv.inRange(hsv_frame, (175, 91, 39), (180, 255, 255))
        orange_mask = cv.bitwise_or(mask1, mask2)

        # kernal = np.ones((5, 5), np.uint8)
        # opening = cv.morphologyEx(orange_mask, cv.MORPH_OPEN, kernal)

        _, contours, _ = cv.findContours(orange_mask, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
        # contours, hierarchy = cv.findContours(orange_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE) #parameters for how it works
        samples = []

        for cnt in contours:
            area = cv.contourArea(cnt)
            if area > 50:
                x, y, w, h = cv.boundingRect(cnt)  # get sample
                samples.append([x, y, w, h])
                cv.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)  # green bounding box
        return samples

    def find_obstacles(self, hsv_frame):
        # low_green = np.array([39, 71, 30])
        # high_green = np.array([66, 255, 100])

        green_mask = cv.inRange(hsv_frame, (39, 71, 30), (66, 255, 100))

        #kernal = np.ones((5, 5), np.uint8)
        #opening = cv.morphologyEx(orange_mask, cv.MORPH_OPEN, kernal)

        _, contours, _ = cv.findContours(green_mask, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
        # contours, hierarchy = cv.findContours(orange_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE) #parameters for how it works
        obstacles = []

        for cnt in contours:
            area = cv.contourArea(cnt)
            if area > 50:
                x, y, w, h = cv.boundingRect(cnt)  # get sample
                obstacles = [x, y, w, h]
                cv.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2) #green bounding box
        return obstacles

    def find_rocks(self, hsv_frame):
        low_green = np.array([100, 124, 35])
        high_green = np.array([106, 255, 230])
        blue_mask = cv.inRange(hsv_frame, low_green, high_green)
        #blue_mask = cv.inRange(hsv_frame, (79,64,19), (119, 255, 190))

        #kernal = np.ones((5, 5), np.uint8)
        #opening = cv.morphologyEx(orange_mask, cv.MORPH_OPEN, kernal)

        _, contours, _ = cv.findContours(blue_mask, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
        # contours, hierarchy = cv.findContours(orange_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE) #parameters for how it works
        obstacles = []

        for cnt in contours:
            area = cv.contourArea(cnt)
            if area > 100:
                x, y, w, h = cv.boundingRect(cnt)  # get sample
                obstacles.append([x, y, w, h])
                cv.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2) #green bounding box
        return obstacles

    def sample_range_bearing(self,data1): # x,y,w,h
        locsamples= []
        for data in data1:
            x = data[0]
            y = data[1]
            w = data[2]
            h = data[3]

            #return Z,heading
            #r = (w / 2 + h / 2)
            # r = (max(w) / 2 + max(h) / 2)
            Z = focal * sampled / (w * 1.12E-6) /5  # sampled*focal/x
            heading = mapping_linear(x + w / 2, 0, frame_width, -FOV / 2, FOV / 2)
            locsamples.append([Z,heading])
        return locsamples

    def obstacle_range_bearing(self,data2): # x,y,w,h
        locobstacles = []
        if (data2):
            x = data2[0]
            y = data2[1]
            w = data2[2]
            h = data2[3]

            #return Z,heading
            r = (w / 2 + h / 2)
            # r = (max(w) / 2 + max(h) / 2)
            Z = focal * obstaclew / (r * 1.12E-6)/5 # sampled*focal/x
            heading = mapping_linear(x + w / 2, 0, frame_width, -FOV / 2, FOV / 2)
            locobstacles = [Z,heading]
        return locobstacles

    def rock_range_bearing(self,data3): # x,y,w,h
        locrock = []
        for data in data3:
            x = data[0]
            y = data[1]
            w = data[2]
            h = data[3]

            #return Z,heading
            #r = (w / 2 + h / 2)
            # r = (max(w) / 2 + max(h) / 2)
            Z = focal * rockw / (h * 1.12E-6)/5 # sampled*focal/x
            heading = mapping_linear(x + w / 2, 0, frame_width, -FOV / 2, FOV / 2)
            locrock.append([Z,heading])
        return locrock

def generate_image(data1,var1,data2,var2,data3,var3,stringtype,frame):  #string type = sample, obstacle # frame = original capture data1,var1,
        # x = []
        # y = []
        # Z = []
        # heading = []
        for i in range(len(data1)):
            x = data1[i][0]
            y = data1[i][1]
            Z = var1[i][0]
            heading = var1[i][1]
            #frame = cv.putText(frame, f'{stringtype[0]}- Range:{Z:.5f} - Bearing:{heading:.1f}', (x, y), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 1, cv.LINE_AA)
            frame = cv.putText(frame, "SAMPLE - Range:" + str(round(Z,5)) + " - Bearing:" + str(round(heading, 1)), (x, y), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 1, cv.LINE_AA)

        if (data2):
            x = data2[0]
            y = data2[1]
            Z = var2[0]
            heading = var2[1]

            #if x is less than 0 x = 0 if x is greater than frame_width x

            frame = cv.putText(frame, "OBSTACLE - Range:" + str(round(Z,1)) + " - Bearing:" + str(round(heading, 1)), (x, y), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 1, cv.LINE_AA)

        for i in range(len(data3)):
            x = data3[i][0]
            y = data3[i][1]
            Z = var3[i][0]
            heading = var3[i][1]

            # if x is less than 0 x = 0 if x is greater than frame_width x
            frame = cv.putText(frame, "ROCK - Range:" + str(round(Z, 1)) + " - Bearing:" + str(round(heading, 1)), (x, y), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 1, cv.LINE_AA)

        return frame

if __name__ == '__main__':
    capture = cv.VideoCapture(0)
    #capture = cv.VideoCapture(0)
    capture.set(3, frame_width)
    capture.set(4, frame_length)

    obd = Object_Detection()
    while(True):
        ret, frame = capture.read()
        #frame = cv.resize(frame,(frame_width,frame_length))
        height, width, _ = frame.shape


        now = time.time()  # start process time
        frame = cv.rotate(frame, cv.ROTATE_180)
        blurred_frame = cv.GaussianBlur(frame, (5, 5), 0)
        hsv_frame = cv.cvtColor(blurred_frame, cv.COLOR_BGR2HSV)
        data1 = obd.find_samples(hsv_frame)
        data2 = obd.find_obstacles(hsv_frame)
        data3 = obd.find_rocks(hsv_frame)

        sampleRB = obd.sample_range_bearing(data1)
        obstacleRB = obd.obstacle_range_bearing(data2)
        rockRB = obd.rock_range_bearing(data3)

        stringtype = ["sample","obstacle","rock","wall","lander"]
        disp_image = generate_image(data1,sampleRB,data2, obstacleRB, data3,rockRB, stringtype, frame)


        cv.imshow('All', frame)
        if cv.waitKey(20) == ord('d'):  # if d is pressed stop video
            break
        elapsed = time.time() - now  # calculate how long it took
        
        # initialising range and bearing variables
        avoidDist = [0,0]
        avoidAng = [0,0]
        collectDist = [0,0]
        collectAng = [0,0]

        # check to see if the samples are within the camera's FOV
        if sampleRB != []:
            collectDist[0] = sampleRB[0][0]
            collectAng[0] = sampleRB[0][1]
            
        # check to see if the obstacles are within the camera's FOV
        if obstacleRB != []:
            avoidDist[1] = obstacleRB[0]
            avoidAng[1] = obstacleRB[1]
            
        # check to see if the rocks are within the camera's FOV
        if rockRB != []:
            collectDist[1] = rockRB[0][0]
            collectAng[1] = rockRB[0][1]
            
        #print("Sample: "+ str(collectDist[0]) + " Rock: " + str(collectDist[1]) + " Obstacle: " + str(avoidDist[1]))
    

        """
        # check to see if lander is within the camera's FOV
        if landerRB != None:
            for Object in landerRB:
                avoidDist[0] = Object[0]
                avoidAng[0] = Object[1]

        # sample collected conditions
        # reset collection distance
        
        if roverBotSim.CollectSample() == True:
            collectDist = [0,0]
            print("sample 1 collected")

            # if roverBotSim.SampleCollected():
                # return to lander

        """
        # rover locating sample
        # if previous distance lower than 0.1 and previous state = state 1
        # then go straight for a fixed amount of time
        
        if collectDist[0] == 0 and collectDist[1] == 0:
            state = 0
            # move to next sample
            #if rotation_counter > 80:
            #    state = 3

        # 
        if collectDist[0] != 0:
            #state = 1
            stateSwitch = True

        if collectDist[1] != 0:
            state = 2
            
        #Rotate on the spot to find samples
        if state == 0:
            print("state 0 - locate or re-locate sample")
            print("LED red")        
            speed(0, -30)
            time.sleep(0.1)
            speed(30, 0)
            rotation_counter += 1
        
        #Go towards sample and pick it up
        if state == 1:
            print("state 1 - collect sample" + str(collectDist[0]))
            print("LED yellow")
            rotation_counter = 0
            
            #Move towards sample using vision data if range is greater than 0.24
            if collectDist[0] > 0.24:
                speed(lowVel - constVal*(collectAng[0]),lowVel + constVal*(collectAng[0]))
                timer = time.time()
            else:
                #After it is in close range, for 0.5 seconds go at 60,50 speed
                print(time.time() - timer)
                if time.time() - timer < 0.5:
                    speed(60,50)
                    sc.release_sample()
                    
                else:
                    #After 0.5 seconds, try grab the sample with the claw
                    collectionAmount += 1
                    sc.close_claws()
                    time.sleep(0.2)
                    sc.grab_sample()
                    time.sleep(0.2)
                    sc.lift_rock()
                    time.sleep(3)
                    sc.release_sample()
                    time.sleep(3)
                    state = 0
            

        # state 2: collect rock
        if state == 2:
            
            #print("state 2 - collect rock" + str(collectDist[1]))
            
            #Move towards rock using vision data if range is greater than 0.3
            if collectDist[1] > 0.3:
                speed(lowVel - constVal*(collectAng[1]),lowVel + constVal*(collectAng[1]))
                timer = time.time()
            else:
                
                #After it is in close range, for 2.8 seconds go at 25,15 speed
                print(time.time() - timer)
                sc.release_sample()
                if time.time() - timer < 2.8:
                    speed(25,15)
                else:
                    #After 2.8 seconds, try lift the rock with the claw
                    sc.lift_rock()
                    time.sleep(0.2)
                    sc.grab_sample()
                    time.sleep(0.2)
                state = 0

        # state 3: move to next sample
        if state == 3:

            print("state 3 - move to next sample" + str(motion_counter))
            print("LED green")
            if avoidDist[0] == 0 and avoidDist[1] == 0:
                requiredVel = [lowVel,0]

                motion_counter += 1
                if motion_counter > 30:
                    collectDist = [0,0]
                    rotation_counter = 0
                    motion_counter = 0

            else:
                requiredVel[0] = -30
                requiredVel[1] = 30
        
        
    capture.release()
    cv.destroyAllWindows()
    sc.stop()
    cv.waitKey(0)

