from styx_msgs.msg import TrafficLight
from sensor_msgs.msg import Image
import tensorflow as tf 
import numpy as np 
import os 
import rospy

class TLClassifier(object):
    def __init__(self):
        #TODO load froze graph
        path = os.path.dirname(os.path.realpath(__file__))
        #graph_file = r'/home/student/catkin_ws/CarND-Capstone/ros/src/tl_detector/light_classification/tl_classification_final/frozen_inference_graph.pb'
        graph_file = path + '/tl_classification_final/frozen_inference_graph.pb'
        print("graph file is {}").format(graph_file)
        self.graph = tf.Graph()
        with self.graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(graph_file,'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def,name='')

            self.image_tensor = self.graph.get_tensor_by_name('image_tensor:0')
            self.detect_boxes = self.graph.get_tensor_by_name('detection_boxes:0')
            self.detect_scores = self.graph.get_tensor_by_name('detection_scores:0')
            self.detect_classes = self.graph.get_tensor_by_name('detection_classes:0')
            self.num_detections = self.graph.get_tensor_by_name('num_detections:0')

    def get_classification(self, msg_image):
        """Determines the color of the traffic light in the image
        Args:
            image: /camera_image
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        img = np.fromstring(msg_image.data,dtype=np.uint8)
        img = img.reshape(msg_image.height,msg_image.width,3)
        img_expanded = np.expand_dims(img, axis=0)
       
        #TODO implement light color prediction
        with tf.Session(graph=self.graph) as sess:
            (boxes,scores,classes, num) = sess.run([self.detect_boxes,self.detect_scores,self.detect_classes,self.num_detections],feed_dict={self.image_tensor:img_expanded})
            detection = 4
            max_score = 0

            for i in range(len(scores[0])):
                # only classify is score is greater than 50%
                if scores[0][i] > max_score and scores[0][i]>0.5:
                    detection = classes[0][i] 
                    max_score = scores[0][i]

        if detection ==1:
            return TrafficLight.GREEN
        elif detection ==2:
            return TrafficLight.RED
        elif detection ==3:
            return TrafficLight.YELLOW
        elif detection ==4:
            return TrafficLight.UNKNOWN