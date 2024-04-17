import cv2
import numpy as np
import sys

class CircleDetector:
    def __init__(self, dp=1, min_dist=20, param1=50, param2=70, min_radius=240, max_radius=480,
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
        cv2.imshow("Webcam", frame) # This will open an independent window
        return circles

    def recognize_red(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, self.hsv_lower_red1, self.hsv_upper_red1)
        mask2 = cv2.inRange(hsv, self.hsv_lower_red2, self.hsv_upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        res = cv2.bitwise_and(frame, frame, mask=mask)
        
        cv2.imshow("Filtered", res)
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


    def process_frame(frame, detector):
        circles = detector.find_circles(frame)
        if circles is not None:
            res = detector.recognize_red(frame)
            counts = detector.count_red_pixels(res, circles)
            center_x, center_y, radius = detector.find_most_red_circle(counts, circles)
            print(center_x, center_y, radius)
            return center_x, center_y, radius

def main(source='red_ball_img.jpg'):
    detector = CircleDetector()

    if source == 'camera':
        # Use camera
        cap = cv2.VideoCapture(0)  # Default camera
        cap.set(3, 640)  # Width
        cap.set(4, 480)  # Height
    elif source.endswith('.mp4') or source.endswith('.avi'):
        # Use video file
        cap = cv2.VideoCapture(source)
    else:
        # Use image file
        frame = cv2.imread(source)
        if frame is None:
            print("Failed to load image.")
            sys.exit(1)
        detector.process_frame(frame, detector)
        cv2.waitKey(0)  # Wait until a key is pressed
        cv2.destroyAllWindows()
        return

    # Video or camera input
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        detector.process_frame(frame, detector)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    cv2.waitKey(1)

if __name__ == "__main__":
    try:
        main()
    except Exception as e:  
        cv2.destroyAllWindows()
        cv2.waitKey(1)