#!/usr/bin/env python

import rospy
from std_msgs.msg import ColorRGBA
from hector_worldmodel_msgs.msg import ImagePercept
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import MarkerArray, Marker
from hector_worldmodel_msgs.msg import ObjectModel, Object
import time
import tf
from rospkg import RosPack
import os.path

SUB_TOPIC = "/worldmodel/image_percept"
PUB_QR_MARKERS = "/qr_markers"
PUB_QR_INFO = "/myqrs"

class QrcodeToCSV():
    """ Read QR code detections and write them to CSV file.
    """
    def __init__(self, mission_name = "preliminar4"):
        self.last_marker_array = MarkerArray()
        self.last_obj_info = ObjectModel()
        self.last_obj_info.header.frame_id = "/map"
        self.pub_read_qrs = rospy.Publisher(PUB_QR_MARKERS, MarkerArray, latch=True, queue_size=10)
        self.pub_qr_info = rospy.Publisher(PUB_QR_INFO, ObjectModel, latch=True, queue_size=10)
        self.qrcode_sub = rospy.Subscriber(SUB_TOPIC, ImagePercept, self.qrcode_cb, queue_size=10)
        self.tf_l = tf.TransformListener()
        self.read_codes_dict = {}
        self.curr_object_id = 1
        self.filename = "RC2015_rescube_" + str(mission_name) + "_qr.csv"
        rp = RosPack()
        self.path_for_csv = rp.get_path("hector_qrcode_detection") + "/"
        self.path_for_csv += self.filename
        if os.path.isfile(self.path_for_csv): # if file exists already
            # we add the date at the end
            self.path_for_csv += str(time.strftime("%H:%M:%S"))
            # and cross fingers you dont execute this twice in the same second
        print "The path where we will store the csv is: " + str(self.path_for_csv)

        self.header = self.get_header_string(mission_name)
        print "Header is:"
        print self.header
        self.write_on_report(self.header)

    def create_qr_marker(self, text, point):
        """
        :param text: string with the text
        :param point: PointStamped with where this text should go
        :return: a Marker msg
        """
        m = Marker()
        m.action = Marker.ADD
        m.type = Marker.TEXT_VIEW_FACING
        m.header = point.header
        m.text = text
        m.pose.position = point.point
        m.scale.x = m.scale.y = m.scale.z = 0.1
        m.color = ColorRGBA(0.0, 1.0, 0.0, 0.7)
        return m

    def write_on_report(self, text):
        f = open(self.path_for_csv, "a")
        f.write(str(text))
        f.close()

    def get_header_string(self, mission_name):
        header = '''"qr_codes"
"1.0"
"rescube"
"hungary"
'''
        ## dd/mm/yyyy format
        header += '"' + str(time.strftime("%d/%m/%Y")) + '"\n'
        header += '"' + str(time.strftime("%H:%M:%S")) + '"\n'
        header += '"' + str(mission_name) + '"' + '\n'
        header += "\n"
        header += "id,time,text,x,y,z\n"
        return header


    def qrcode_cb(self, data):
        """:type data: ImagePercept"""
# A detection looks like:
# 1,14:28:01,Y_1_1_chair_yoked,-8.29994,-2.29014,0.45610
# 2,14:28:02,Y_1_2_chair_yokel,-8.29994,-2.29014,0.45610
        detected_name = data.info.name
        if detected_name not in self.read_codes_dict:
            # add it to the dict to know that we already read it
            self.read_codes_dict[detected_name] = True
            # Write to the CSV file
            string_to_csv = str(self.curr_object_id) + "," + str(time.strftime("%H:%M:%S")) + ","
            string_to_csv += detected_name
            # We try to estimate the pose of the qrcode by getting
            # the current pose of the robot (head_camera transform over map)
            ps = PointStamped()
            ps.header.stamp = self.tf_l.getLatestCommonTime("head_cam1_link", "map")
            ps.header.frame_id = "head_cam1_link"
            ps.point.x = 0.5
            ps.point.z = 0.2
            transform_ok = False
            while not transform_ok and not rospy.is_shutdown():
                try:
                    head_in_map_frame = self.tf_l.transformPoint("/map", ps)
                    transform_ok = True
                except tf.ExtrapolationException as e:
                    rospy.logwarn("Exception on transforming point head to map... trying again \n(" + str(e) + ")")
                    rospy.sleep(0.01)
                    ps.header.stamp = self.tf_l.getLatestCommonTime("head_cam1_link", "map")
            string_to_csv += "," + str(head_in_map_frame.point.x) + ","
            string_to_csv += str(head_in_map_frame.point.y) + ","
            string_to_csv += str(head_in_map_frame.point.z) + "\n"
            self.curr_object_id += 1
            print "Writing: \n" + str(string_to_csv)
            self.write_on_report(string_to_csv)
            # Also now publish a markerarray with it
            qr_m = self.create_qr_marker(detected_name, head_in_map_frame)
            self.last_marker_array.markers.append(qr_m)
            self.pub_read_qrs.publish(self.last_marker_array)
            curr_o = Object()
            curr_o.header = qr_m.header
            curr_o.pose.pose = qr_m.pose
            self.last_obj_info.objects.append(curr_o)
            self.pub_qr_info.publish(self.last_obj_info)

        else:
            print "Read " + str(detected_name) + " but we already read it before."

    def keep_publishing(self):
        while not rospy.is_shutdown():
            self.pub_read_qrs.publish(self.last_marker_array)
            self.pub_qr_info.publish(self.last_obj_info)
            rospy.sleep(2.0)




if __name__ == '__main__':
    rospy.init_node('qrcode_to_csv', anonymous=True)
    qtcsv = QrcodeToCSV()
    qtcsv.keep_publishing()
    #rospy.spin()

