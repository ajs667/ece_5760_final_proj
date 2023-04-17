import cv2
import numpy as np

img = cv2.imread("bruce.jpg")

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

integral_img = cv2.integral(gray)

feature_cascade = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")

objects = feature_cascade.detectMultiScale(gray, scaleFactor = 1.3, minNeighbors = 5)

for(x,y,w,h) in objects:
    cv2.rectangle(img, (x,y), (x+w, y+h), (0,255,0), 2)

cv2.imshow('image', img)

cv2.waitKey(0)

cv2.destroyAllWindows()

def calculate_sum(integral_image, x, y, width, height):
    # Get the sum of the pixels in the region
    A = integral_image[y, x]
    B = integral_image[y, x + width]
    C = integral_image[y + height, x]
    D = integral_image[y + height, x + width]
    region_sum = D - B - C + A
    return region_sum

def calculate_feature_value(integral_image, feature):
    # Get the coordinates and size of the feature's regions
    (x1, y1, w1, h1), (x2, y2, w2, h2), (x3, y3, w3, h3) = feature

    # Calculate the sum of the pixels in each region
    A = calculate_sum(integral_image, x1, y1, w1, h1)
    B = calculate_sum(integral_image, x2, y2, w2, h2)
    C = calculate_sum(integral_image, x3, y3, w3, h3)

    # Calculate the feature value
    feature_value = A - B - C

    return feature_value

def classify_region(integral_image, cascade):
    for stage in cascade:
        stage_sum = 0
        for feature in stage:
            feature_value = calculate_feature_value(integral_image, feature)
            if feature_value < feature[3]:
                stage_sum += feature[2]
            else:
                break
        if stage_sum < stage[-1]:
            return False
    return True



if __name__ == "__main__":
    haar_cascade = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")
    image = cv2.imread("image.jpg")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Calculate the integral image
    integral_image = cv2.integral(gray)

    # Set the window size and step size for sliding the window
    window_size = (24, 24)
    step_size = 4

    # Slide the window over the integral image and classify each region
    for y in range(0, gray.shape[0] - window_size[1], step_size):
        for x in range(0, gray.shape[1] - window_size[0], step_size):
            region_sum = calculate_sum(integral_image, x, y, window_size[0], window_size[1])
            if region_sum < 50:
                continue
            if classify_region(integral_image[y:y+window_size[1], x:x+window_size[0]], haar_cascade):
                cv2.rectangle(image, (x, y), (x + window_size[0], y + window_size[1]), (0, 255))