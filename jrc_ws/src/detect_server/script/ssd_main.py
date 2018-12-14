#!/usr/bin/env python
import os
import sys
import cv2
import numpy as np
import time
import math

import rospy
from jrc_srvs.srv import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

caffe_root = './'
os.chdir(caffe_root)
sys.path.insert(0, os.path.join(caffe_root, 'python'))
import caffe
from google.protobuf import text_format
from caffe.proto import caffe_pb2

model_def = '/home/agv/jrc_ws/src/detect_server/ssd_model/deploy.prototxt'
model_weights = '/home/agv/jrc_ws/src/detect_server/ssd_model/weight.caffemodel'
labelmap_file = '/home/agv/jrc_ws/src/detect_server/ssd_model/labelmap.prototxt'
image_resize = 300
conf_thresh= 0.7
topn = 16
seg_image = np.zeros((640, 480,3),np.uint8)
final_bbx_table = []
camera_fx = 570.3
camera_fy = 570.3
camera_cx = 319.5
camera_cy = 239.5

class BBOX(object):
    def update(self,label,xmin,ymin,xmax,ymax,dpt_image):
        self.label = label
        if xmin<0:
            xmin = 0
        if ymin<0:
            ymin = 0
        if xmax>640:
            xmax = 640
        if ymax>480:
            ymax = 480
        self.xmin = xmin
        self.ymin = ymin
        self.xmax = xmax
        self.ymax = ymax
        roi_dpt_image = dpt_image[ymin:ymax,xmin:xmax]
        roi_cloud_dis = np.zeros((ymax-ymin+1,xmax-xmin+1))
        # roi_dpt_image = roi_dpt_image[~np.isnan(roi_dpt_image)]
        for i in range(0,ymax-ymin):
            for j in range(0,xmax-xmin):
                if roi_dpt_image[i,j]<0:
                    continue;
                cloud_z = roi_dpt_image[i,j]/float(1000)
                cloud_x = (j+xmin-camera_cx)*cloud_z/camera_fx
                cloud_y = (i+ymin-camera_cy)*cloud_z/camera_fy
                roi_cloud_dis[i,j] = math.sqrt(cloud_x*cloud_x + cloud_y*cloud_y + cloud_z*cloud_z)
        self.average_depth = roi_cloud_dis.mean()
        print "average_dpt: %f " %self.average_depth

def get_labelname(labelmap, labels):
    num_labels = len(labelmap.item)
    labelnames = []
    if type(labels) is not list:
        labels = [labels]
    for label in labels:
        found = False
        for i in xrange(0, num_labels):
            if label == labelmap.item[i].label:
                found = True
                labelnames.append(labelmap.item[i].display_name)
                break
        assert found == True
    return labelnames

def send_bbox(req):

    caffe.set_device(0)
    caffe.set_mode_gpu()
    
    time_start=time.time()
    res = np.float32(rgb_image)
    scaling_factor = 1.0/255
    res = res*scaling_factor

    transformed_image = transformer.preprocess('data', res)

    net.blobs['data'].data[0] = transformed_image

    # Forward pass.
    detections = net.forward()['detection_out']
    time_end=time.time()
    print "cost: %f s " %(time_end-time_start)

    # Parse the outputs.
    det_label = detections[0,0,:,1]
    det_conf = detections[0,0,:,2]
    det_xmin = detections[0,0,:,3]
    det_ymin = detections[0,0,:,4]
    det_xmax = detections[0,0,:,5]
    det_ymax = detections[0,0,:,6]

    top_indices = [i for i, conf in enumerate(det_conf) if conf >= conf_thresh]

    top_conf = det_conf[top_indices]
    top_label_indices = det_label[top_indices].tolist()
    top_labels = get_labelname(labelmap, top_label_indices)
    top_xmin = det_xmin[top_indices]
    top_ymin = det_ymin[top_indices]
    top_xmax = det_xmax[top_indices]
    top_ymax = det_ymax[top_indices]

    result = []
    for i in xrange(min(topn, top_conf.shape[0])):
        xmin = top_xmin[i] # xmin = int(round(top_xmin[i] * image.shape[1]))
        ymin = top_ymin[i] # ymin = int(round(top_ymin[i] * image.shape[0]))
        xmax = top_xmax[i] # xmax = int(round(top_xmax[i] * image.shape[1]))
        ymax = top_ymax[i] # ymax = int(round(top_ymax[i] * image.shape[0]))
        score = top_conf[i]
        label = int(top_label_indices[i])
        label_name = top_labels[i]
        result.append([xmin, ymin, xmax, ymax, label, score, label_name])
    width = 640
    height = 480

    bbox_table = []
    for i in range(0,17):
        bbox_table.append([])
    object_counter = [0]*17
    for item in result:
        print item[-1]
        xmin = int(round(item[0] * width))
        ymin = int(round(item[1] * height))
        xmax = int(round(item[2] * width))
        ymax = int(round(item[3] * height))
        item_num = 0
        if item[-1] == "leshi":
            item_num = 1
        elif item[-1] == "milk":
            item_num = 2
        elif item[-1] == "zhencui":
            item_num = 3
        elif item[-1] == "yagao":
            item_num = 4
        elif item[-1] == "coke":
            item_num = 5
        elif item[-1] == "gaipian":
            item_num = 6
        elif item[-1] == "juzi":
            item_num = 7
        elif item[-1] == "yajiao":
            item_num = 8
        elif item[-1] == "taiping":
            item_num = 9
        elif item[-1] == "zhijin":
            item_num = 10
        elif item[-1] == "book":
            item_num = 11
        elif item[-1] == "aoliao":
            item_num = 12
        elif item[-1] == "guozhen":
            item_num = 13
        elif item[-1] == "xifalu":
            item_num = 14
        elif item[-1] == "xiangchang":
            item_num = 15
        elif item[-1] == "yashua":
            item_num = 16
        else:
            item_num = 0
        object_counter[item_num] +=1
        global seg_image
        seg_image = np.zeros(([rgb_image.shape[0], rgb_image.shape[1],rgb_image.shape[2]]),np.uint8)
        seg_image[ymin-15:ymax+15, xmin-15:xmax+15, :] = np.uint8(item_num)
        if item_num > 0:
            object_bbox = BBOX()
            object_bbox.update(item_num,xmin,ymin,xmax,ymax,dpt_image)
            bbox_table[item_num].append(object_bbox)
        #print xmin, ymin, xmax, ymax

        cv2.rectangle(rgb_image,(xmin,ymin),(xmax,ymax),(0,255,0),2)
        cv2.putText(rgb_image, '{:s} {:.3f}'.format(item[-1], item[-2]),(xmin, ymin),0,0.8,(0,255,0),2)
        tmp_rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
#        final_bbx_table = []
    if len(final_bbx_table)>0:
        del final_bbx_table[:]
    for i in range(0,17):
        final_bbx_table.append([])
    for i in range(0,17):
        if len(bbox_table[i])>0:
            min_index = 0
            for j in range(0,len(bbox_table[i])):
                if bbox_table[i][j].average_depth < bbox_table[i][min_index].average_depth:
                    min_index = j
            final_bbx_table[i] = bbox_table[i][min_index]

    for i in range(0,len(final_bbx_table)):
        if final_bbx_table[i]:
            print "label: %d, xmin: %d, ymin: %d, depth: %f" \
            %(final_bbx_table[i].label,final_bbx_table[i].xmin,final_bbx_table[i].ymin,final_bbx_table[i].average_depth)

    # cv2.imshow("result", rgb_image)
    # cv2.waitKey(1)

    object_info = []
    for i in range(0,17):
        object_info.append([])
    for i in range(0,17):
        if final_bbx_table[i]:
            object_info[i] = [final_bbx_table[i].label,final_bbx_table[i].xmin,final_bbx_table[i].ymin,\
            final_bbx_table[i].xmax,final_bbx_table[i].ymax]
    return [object_info[1],object_info[2],object_info[3],object_info[4],object_info[5],object_info[6],object_info[7],object_info[8],\
            object_info[9],object_info[10],object_info[11],object_info[12],object_info[13],object_info[14],object_info[15],object_info[16]]

def main():
    rospy.init_node('ssd_main', anonymous=True)
    bridge = CvBridge()
    image_service = rospy.ServiceProxy("kinect_server", rgbd_image)
    # image_service = rospy.ServiceProxy("realsense2_server", rgbd)
    bbox_service = rospy.Service('bbox', bbox_msgs, send_bbox)

    caffe.set_device(0)
    caffe.set_mode_gpu()
    global net
    net = caffe.Net(model_def,      # defines the structure of the model
                   model_weights,  # contains the trained weights
                   caffe.TEST)
    net.blobs['data'].reshape(1, 3, image_resize, image_resize)
    global transformer
    transformer = caffe.io.Transformer({'data': net.blobs['data'].data.shape})
    transformer.set_transpose('data', (2, 0, 1))
    transformer.set_mean('data', np.array([104, 117, 123]))
    transformer.set_raw_scale('data', 255)
#    transformer.set_channel_swap('data', (2, 1, 0))
    global file
    file = open(labelmap_file, 'r')
    global labelmap
    labelmap = caffe_pb2.LabelMap()
    text_format.Merge(str(file.read()), labelmap)

    global dpt_image
    global rgb_image

    while not rospy.is_shutdown():
        resp1 = image_service(True)
        msg_rgb = resp1.rgb_image
        msg_dpt = resp1.depth_image
        #rgb_image = np.ndarray(shape=(480, 640, 3), dtype='int')
        dpt_image = bridge.imgmsg_to_cv2(msg_dpt,desired_encoding="passthrough")
        rgb_image = bridge.imgmsg_to_cv2(msg_rgb, desired_encoding="passthrough")
        # rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
        # cv2.imshow("rgb",rgb_image)
        # cv2.waitKey(1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass