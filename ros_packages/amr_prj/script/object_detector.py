import cv2
import numpy as np
import sys

class CircleDetector:
    def __init__(self, dp=1, min_dist=20, param1=50, param2=70, min_radius=10, max_radius=200,
                 hsv_lower_red1=(0, 50, 50), hsv_upper_red1=(10, 255, 255),
                 hsv_lower_red2=(170, 50, 50), hsv_upper_red2=(180, 255, 255),
                 blur_ksize=(5, 5), blur_sigma=0):
        
        # TODO: Assess min, max radius and min distance
        # TODO: Optimize hsv vlaues for OOI
        
        # Circle detection parameters
        self.dp = dp
        self.min_dist = min_dist
        self.param1 = param1
        self.param2 = param2
        self.min_radius = min_radius
        self.max_radius = max_radius
        
        # Red detection parameters in HSV
        self.hsv_lower_red1 = hsv_lower_red1
        self.hsv_upper_red1 = hsv_upper_red1
        self.hsv_lower_red2 = hsv_lower_red2
        self.hsv_upper_red2 = hsv_upper_red2
        
        # Blurring parameters
        self.blur_ksize = blur_ksize
        self.blur_sigma = blur_sigma
        
        # Tracking
        self.tracked_circle = []
        self.tracking = False

    def find_circles(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, self.blur_ksize, self.blur_sigma)
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, self.dp, self.min_dist, param1=self.param1, param2=self.param2, minRadius=self.min_radius, maxRadius=self.max_radius)
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                # Draw the outer circle
                cv2.circle(frame, (i[0], i[1]), i[2], (0, 255, 0), 2)
                # Draw the center of the circle
                cv2.circle(frame, (i[0], i[1]), 2, (255, 0, 0), 3)
        return circles

    def recognize_red(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, self.hsv_lower_red1, self.hsv_upper_red1)
        mask2 = cv2.inRange(hsv, self.hsv_lower_red2, self.hsv_upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        res = cv2.bitwise_and(frame, frame, mask=mask)
        return res

    def mask_circle(self, res, center, radius):
        mask = np.zeros_like(res)
        cv2.circle(mask, center, radius, (255, 255, 255), thickness=-1)
        masked_img = cv2.bitwise_and(res, mask)
        return masked_img

    def count_red_pixels(self, res, circles):
        counts = []
        for circle in circles[0, :]:
            center = (circle[0], circle[1])
            radius = circle[2]
            masked_img = self.mask_circle(res, center, radius)
            count = np.sum(masked_img > 0) / (np.pi * radius ** 2)
            counts.append(count)
        return counts

    def find_most_red_circle(self, counts, circles):
        max_index = np.argmax(counts)
        max_circle = circles[0, max_index]
        return max_circle[0], max_circle[1], max_circle[2]  # x, y, radius


    def recognize_OOI(self, frame):
        circles = self.find_circles(frame)
        if circles is not None:
            res = self.recognize_red(frame)
            counts = self.count_red_pixels(res, circles)
            center_x, center_y, radius = self.find_most_red_circle(counts, circles)
            return center_x, center_y, radius
        
    def circle_in_field_of_regard(self, ref_circle, circles):
        # Get the dimensions of the field of regard
        maxdist=20

        # Get the coordinates of the ref_circle
        xref = ref_circle[0]
        yref = ref_circle[1]

        goodcircles = np.copy(circles)

        #print(circles)
        i=0
        delindex = []

        for circle in circles[0, :]:
            x = circle[0]
            y = circle[1]
            if (xref-x)**2+(yref-y)**2>maxdist**2:
                #print((xref-x)**2+(yref-y)**2)
                delindex.append(i)
            i+=1

        goodcircles = np.delete(goodcircles, delindex, axis=1)

        if(goodcircles.size==0):
            return None

        #print(goodcircles)
        #print(circles)
        #cv2.waitKey(0)
        return goodcircles
    
    def get_circle(self):
        if self.tracking = True:
            self.track_OOI()
        return self.tracked_circle
    
    def track_OOI(self, ref_circle, frame):
        # # Create a VideoCapture object
        # cap = cv2.VideoCapture(0)

        lastcircle = ref_circle
        failcount = 0
        max_circle = None

        while True:
            # Capture frame-by-frame
            max_circle = None
            temp_frame = frame.copy()
            circles=self.find_circles(temp_frame)

            if circles is not None:
                print(circles)
                goodcircles = self.circle_in_field_of_regard(lastcircle, circles)
                if goodcircles is not None:
                    #print("if")
                    res = self.recognize_red(frame)
                    counts = self.count_red_pixels(res, goodcircles)
                    max_circle = self.find_most_red_circle(counts, goodcircles)           
                    #print('\n')
                    lastcircle= max_circle
                    failcount = 0
                    self.tracked_circle = lastcircle
                else:
                    #print("else")
                    failcount += 1
                    print(failcount)
                    if failcount>10:
                        break
            else:
                failcount += 1
                print(failcount)
                if failcount>10:
                    break
            
            #print(failcount)
            print(max_circle)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break




def main():
    # Initialize camera
    cap = cv2.VideoCapture(0)  # '0' is typically the default camera
    if not cap.isOpened():
        print("Error: Camera could not be accessed.")
        return
    
    detector = CircleDetector()
    # Set frame dimensions (optional)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    while True:
        print("Recognizing OOI")
        # Read frame
        ret, frame = cap.read()
        if not ret:
            print("Error: Cannot receive frame (stream end?). Exiting ...")
            break
        max_circle = detector.recognize_OOI(frame)
        
        print(max_circle)
        if max_circle is not None:
            print("Tracking OOI")
            detector.track_OOI(max_circle, frame)

        # Press 'q' to close the window
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        cv2.destroyAllWindows()
        cv2.waitKey(1)