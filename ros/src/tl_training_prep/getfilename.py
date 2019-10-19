import os

# get all the .xml files in the annotation folder and save to a text file for used in create_tf_record.py
with open('tl_dataset.txt',"w") as out:
    for path, subdirs, files in os.walk(r'/home/student/catkin_ws/CarND-Capstone/ros/tl_dataset/annotations'):
        for filename in files:
            out.write(os.path.splitext(filename)[0]+os.linesep)