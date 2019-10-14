import os

with open('tl_dataset.txt',"w") as out:
    for path, subdirs, files in os.walk(r'/home/student/catkin_ws/CarND-Capstone/ros/tl_image_files/annotations'):
        for filename in files:
            out.write(os.path.splitext(filename)[0]+os.linesep)