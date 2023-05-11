import cv2
import numpy as np
import subprocess

#ask for data
lin_source_dir = 'root@10.253.17.24:/home/root/video/data.dat'
win_dest_dir = r'C:\Users\ajsco\ece5760\ece_5760_final_proj\model'
ssh_key = r'C:\Users\ajsco\.ssh\id_rsa'
command_get = "scp -i {source_dir} {dest_dir}"
password = "snap"
command = f'echo {password} | scp -i {ssh_key} {lin_source_dir} {win_dest_dir}' 
subprocess.run(command, shell = True)




#process data
data = np.fromfile(r'C:\Users\ajsco\ece5760\ece_5760_final_proj\model\data.dat', dtype=np.uint8)
rows = 240  # Replace with the actual number of rows of the array
cols = 320  # Replace with the actual number of columns of the array
gray = data.reshape(rows, cols)
# print(img)

cv2.imshow('Image', gray)
cv2.waitKey(0)
cv2.destroyAllWindows()




# img2 = cv2.imread("grace.jpg")
# img2 = cv2.resize(img2, (320, 240))
# gray = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
# print("comp")
# print(gray)
# integral_img = cv2.integral(gray)



#run feature detection for corners
feature_cascade = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")
objects = feature_cascade.detectMultiScale(gray, scaleFactor = 1.3, minNeighbors = 5) #used to be gray
objects.flatten()
print("face corners")
print(objects)

# #run feature detection for eyes
# eye_cascade = cv2.CascadeClassifier("haarcascade_eye.xml")
# eyes = eye_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=2, minSize=(30, 30))

# print("eye corners")
# print(eyes)
# eyes = eyes.flatten()
# print(eyes)
eye_cascade = cv2.CascadeClassifier("haarcascade_eye.xml")
for (x, y, w, h) in objects:
    roi_gray = gray[y:y+h, x:x+w]
    eyes = eye_cascade.detectMultiScale(roi_gray, scaleFactor=1.1, minNeighbors=5)

print(eyes)
eyes2 = eyes.flatten()

for (ex, ey, ew, eh) in eyes:
    cv2.rectangle(roi_gray, (ex, ey), (ex+ew, ey+eh), (0, 255, 0), 2)

cv2.imshow('image', gray)
cv2.waitKey(0)
cv2.destroyAllWindows()

#save data
with open(r'C:\Users\ajsco\ece5760\ece_5760_final_proj\model\corners.dat', 'wb') as f:
    objects.tofile(f)



with open(r'C:\Users\ajsco\ece5760\ece_5760_final_proj\model\eyes.dat', 'wb') as f:
    eyes2.tofile(f)
    


#send file back
lin_dest_dir = 'root@10.253.17.24:/home/root/video'
win_source_dir = r'C:\Users\ajsco\ece5760\ece_5760_final_proj\model\corners.dat'
command = f'echo {password} | scp -i {ssh_key} {win_source_dir} {lin_dest_dir}' 
subprocess.run(command, shell = True)

#send file back
lin_dest_dir = 'root@10.253.17.24:/home/root/video'
win_source_dir = r'C:\Users\ajsco\ece5760\ece_5760_final_proj\model\eyes.dat'
command = f'echo {password} | scp -i {ssh_key} {win_source_dir} {lin_dest_dir}' 
subprocess.run(command, shell = True)


#send file to say other is done
lin_dest_dir = 'root@10.253.17.24:/home/root/video'
win_source_dir = r'C:\Users\ajsco\ece5760\ece_5760_final_proj\model\temp.txt'
command = f'echo {password} | scp -i {ssh_key} {win_source_dir} {lin_dest_dir}' 
subprocess.run(command, shell = True)


# for(x,y,w,h) in objects:
#     cv2.rectangle(img, (x,y), (x+w, y+h), (0,255,0), 2)

# cv2.imshow('image', img)

# cv2.waitKey(0)

# cv2.destroyAllWindows()










# def calculate_sum(integral_image, x, y, width, height):
#     # Get the sum of the pixels in the region
#     A = integral_image[y, x]
#     B = integral_image[y, x + width]
#     C = integral_image[y + height, x]
#     D = integral_image[y + height, x + width]
#     region_sum = D - B - C + A
#     return region_sum

# def calculate_feature_value(integral_image, feature):
#     # Get the coordinates and size of the feature's regions
#     (x1, y1, w1, h1), (x2, y2, w2, h2), (x3, y3, w3, h3) = feature

#     # Calculate the sum of the pixels in each region
#     A = calculate_sum(integral_image, x1, y1, w1, h1)
#     B = calculate_sum(integral_image, x2, y2, w2, h2)
#     C = calculate_sum(integral_image, x3, y3, w3, h3)

#     # Calculate the feature value
#     feature_value = A - B - C

#     return feature_value

# def classify_region(integral_image, cascade):
#     for stage in cascade:
#         stage_sum = 0
#         for feature in stage:
#             feature_value = calculate_feature_value(integral_image, feature)
#             if feature_value < feature[3]:
#                 stage_sum += feature[2]
#             else:
#                 break
#         if stage_sum < stage[-1]:
#             return False
#     return True



# if __name__ == "__main__":
#     haar_cascade = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")
#     image = cv2.imread("image.jpg")
#     gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

#     # Calculate the integral image
#     integral_image = cv2.integral(gray)

#     # Set the window size and step size for sliding the window
#     window_size = (24, 24)
#     step_size = 4

#     # Slide the window over the integral image and classify each region
#     for y in range(0, gray.shape[0] - window_size[1], step_size):
#         for x in range(0, gray.shape[1] - window_size[0], step_size):
#             region_sum = calculate_sum(integral_image, x, y, window_size[0], window_size[1])
#             if region_sum < 50:
#                 continue
#             if classify_region(integral_image[y:y+window_size[1], x:x+window_size[0]], haar_cascade):
#                 cv2.rectangle(image, (x, y), (x + window_size[0], y + window_size[1]), (0, 255))