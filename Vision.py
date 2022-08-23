import numpy as np
import cv2 as cv
import time
import math


#global variables
#SI Units

FOV = 62.2
focal = 3.04E-3
rockw = 70E-3
obstaclew = 150E-3
sampled = 42.66E-3
landerheight = 70E-3
wallheight = 450E-3
frame_length = 320 #1280
frame_width = 480 #720


freq = 20
Interval = 1/freq
pixelwidth = 1.12E-6
pixelheight = 1.12E-6

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
        print(height,width)

        now = time.time()  # start process time
        frame = cv.rotate(frame, cv.ROTATE_180)
        blurred_frame = cv.GaussianBlur(frame, (5, 5), 0)
        hsv_frame = cv.cvtColor(blurred_frame, cv.COLOR_BGR2HSV)
        data1 = obd.find_samples(hsv_frame)
        data2 = obd.find_obstacles(hsv_frame)
        data3 = obd.find_rocfks(hsv_frame)

        var1 = obd.sample_range_bearing(data1)
        var2 = obd.obstacle_range_bearing(data2)
        print(var2)

        var3 = obd.rock_range_bearing(data3)

        stringtype = ["sample","obstacle","rock","wall","landers"]
        disp_image = generate_image(data1,var1,data2, var2, data3,var3, stringtype, frame)


        cv.imshow('All', frame)
        if cv.waitKey(20) == ord('d'):  # if d is pressed stop video
            break
        elapsed = time.time() - now  # calculate how long it took
        print("Processing Rate after sleep is: {}.".format(1.0 / elapsed))

    capture.release()
    cv.destroyAllWindows()

    cv.waitKey(0)
        #var1 = obd.sample_range_bearindg()
        #var2 = obd.obstacle_range_bearing(x,y,w,h)
        #var3 = obd.rock_range_bearing()

        #display frame, frame with stuff applied for each. 2 eventually
        #disp_image = generate_image(x, y, w, h, Z, heading, stringtype, frame

        #cv.imshow('image',disp_image)
    # way to stop it playing indefinitely

# if cv.waitKey(20) == ord('d'): #if d is pressed stop video
#     break
