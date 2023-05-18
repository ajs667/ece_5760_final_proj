import cv2
import numpy as np
import subprocess

#ask for data
lin_source_dir = 'root@10.253.17.24:/home/root/video/data.dat'# change to 24 
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




#run feature detection for corners
feature_cascade = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")
objects = feature_cascade.detectMultiScale(gray, scaleFactor = 1.3, minNeighbors = 5) #used to be gray
print(objects)
objects.flatten()
print("face corners")


eye_cascade = cv2.CascadeClassifier("haarcascade_eye.xml")
for (x, y, w, h) in objects:
    roi_gray = gray[y:y+h, x:x+w]
    eyes = eye_cascade.detectMultiScale(roi_gray, scaleFactor=1.1, minNeighbors=5)

print(eyes)
eyes2 = eyes.flatten()

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
