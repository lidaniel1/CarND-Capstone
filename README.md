This is the project repo to complete the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project instructions, see the project introduction on the Udacity Github [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

### Installation

1.	The best way is to install all the software below on a native Linux PC if available. I installed a VirtualBox on a Windows 10 PC to load the virtual machine provided by Udacity which has Ubuntu and ROS already installed. The password is udacity-nd. Set up port forwarding in the VM Settings -> Network -> Adapter 1 Adavanced -> Port Forwarding. Add a rule for Host IP 127.0.0.1, Host Port 4567 and Guest Port 4567.

	Use the following configuration as minimum:
	* 2 CPU
	* 2 GB system memory
	* 25 GB of free hard drive space
	
2. Create a "catkin_ws" folder in the virutal machine home directory. Clone the project repository with the following command in this folder
   * git clone https://github.com/udacity/CarND-Capstone.git
   
   Install Python Dependencies:
   * cd CarND-Capstone
   * pip install -r requirements.txt
   
3. Install a data labeling app on the virtual machine. This is used to manually label the camera images logged from the simulator. This can be downloaded [here](https://github.com/tzutalin/labelImg). For this project, I used the option "Python 2 + Qt4". However, labelImg.py was not able to run initially. I downloaded pyqt5-dev-tools and replaced import PyQt4 to import PyQt5 in a few places, which fixed the issue.

4. Download and install the applications needed for the Tensorflow object detection API which has pre-trained object detection model. I used the pre-trained model as a starting point to train the traffic light images for classifying the traffic light state. The Tensorflow model is [here](https://github.com/tensorflow/models). Initially, I installed the model on the virtual machine. However, I found out later the training could not start due to out of memory. Instead, I installed it on my host PC with a GPU to train the model.

5.	Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases) and save to the host PC. The Simulator runs in the host PC and the control codes run in the virtual machine.

### Working flow
1. Complete the waypoint_update.py (partial) without traffic light detection following the project instructions and walkthrough. The publish rate for the topic "final_waypoints" was set to 10Hz to reduce some latency. 

2. Complete the DBW node by updating [dbw_node.py](https://github.com/lidaniel1/CarND-Capstone/tree/master/ros/src/twist_controller/dbw_node.py) and [twist_controller.py](https://github.com/lidaniel1/CarND-Capstone/tree/master/ros/src/twist_controller/twist_controller.py)

3. Record the traffic light images for the training and validation data set. Manually drove the car around each traffic light going through all three states: green, yellow and red in the simulator and record the two topics "/vehicle/traffic_lights" and "/image_color" to ROS bag files. Since the car was manually driven slow, I used ROS "throttle" commmand to reduce the rate of the two topics to 1Hz and then run rosbag record command.
	* rosrun topic_tools throttle messages /vehicle/traffic_lights 1.0
	* rosrun topic_tools throttle messages /image_color 1.0
	* rosbag record -O run.bag --split --size=1024 /vehicle/traffic_lights_throttle /image_color_throttle

4. Create a [script](https://github.com/lidaniel1/CarND-Capstone/blob/master/ros/src/tl_training_prep/readbagfile.py) to read the recorded bag files and output recorded image to green, red,yellow and unknown folerders.

5. Run the data labeling app to manually label all the images. The app created a lable file (xml) for each image. Totally I labelled 829 images which are [here](https://github.com/lidaniel1/CarND-Capstone/tree/master/ros/image). 
	* python labelImg.py
	
6. Create a TF record file for traffic light classification training. All the labels (xml files) were copied to a single folder [annotations](https://github.com/lidaniel1/CarND-Capstone/tree/master/ros/tl_dataset/annotations). The list of annotations files is saved in [tl_dataset.txt](https://github.com/lidaniel1/CarND-Capstone/blob/master/ros/tl_dataset/annotations/tl_dataset.txt). All the images were copied to another folder [images](https://github.com/lidaniel1/CarND-Capstone/tree/master/ros/tl_dataset/images). Manually create a [tl_label_map.pbtxt](https://github.com/lidaniel1/CarND-Capstone/blob/master/ros/tl_dataset/data/tl_label_map.pbtxt) file. A script [create_tf_record.py](https://github.com/lidaniel1/CarND-Capstone/blob/master/ros/src/tl_training_prep/create_tf_record.py) was run to generate training data set [tl_train.record](https://github.com/lidaniel1/CarND-Capstone/blob/master/ros/tl_dataset/data/tl_train.record) and validation data set [tl_val.record](https://github.com/lidaniel1/CarND-Capstone/blob/master/ros/tl_dataset/data/tl_val.record).

7. Train the traffic light classification using the "ssd_inception_v2_coco" model downloaded from the [Tensorflow detection model zoo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md) on the host PC. The ssd_inception_v2_coco has reasonable accuracy with the fastest execution time. The config file for the training is [here](https://github.com/lidaniel1/CarND-Capstone/blob/master/ros/tl_dataset/tl.config). I had to reduce the batch size to resolve the out of memory issue during the training. The model was trained for 20000 steps as shown in the [figure](https://github.com/lidaniel1/CarND-Capstone/tree/master/ros/src/tl_detector/light_classification/tl_training_progress.png]. After the training was complete, the Tensorflow graph was frozen and saved in [tl_classification_final](https://github.com/lidaniel1/CarND-Capstone/tree/master/ros/src/tl_detector/light_classification/tl_classification_final) folder. A random select of sample images were run through the Tensorflow graph to verify the accuracy. The trained model was able to detect the traffic light state for all the images. The results are [here](https://github.com/lidaniel1/CarND-Capstone/tree/master/ros/src/tl_detector/light_classification/tl_classification_valid. I made another [script](https://github.com/lidaniel1/CarND-Capstone/blob/master/ros/src/tl_detector/tl_classification_class_test.py) to verify classification directly using the rosbag file.

8. Create [tl_classifier.py](https://github.com/lidaniel1/CarND-Capstone/blob/master/ros/src/tl_detector/light_classification/tl_classifier.py) for traffic light classification and complete the [traffic light detection node](https://github.com/lidaniel1/CarND-Capstone/blob/master/ros/src/tl_detector/tl_detector.py) per the project instructions and walkthrough.  The path for the graph file in the script was absolute. To run this code in another machine, the path need be upated.

9. Complete the waypoint updater node (full) per the project instructions and walkthrough. Here is the file [waypoint_updater.py]( https://github.com/lidaniel1/CarND-Capstone/tree/master/ros/src/waypoint_updater/waypoint_updater.py)

10. Test the completed project in the simulator. However, the latency between the virtual machine and host PC was too large once the camera mode was turned on. It was not possible to verify the car driving for a loop. However, the detection of the traffic light status was verified and the command velocity when the traffic light detection was also verified. The car was verified to drive the entire loop when the camera mode was off and without the traffic detection node.
	* roslaunch launch/styx.launch