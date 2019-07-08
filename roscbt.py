#!/usr/bin/python
import rospy
import math
from PIL import Image
import numpy as np

'''
ROS communication benchmarking tool (ROSCBT) is a simulator of a communication link between communication devices. 
ROSCBT is ROS node that is implemeted by this class, using settings that are specified by the user in the configuration file

All parameters are read from package/param/roscbt_config.yaml file that is added to your package

For this node to run, the following parameters MUST be available:
- robot_ids: IDs of robots that make up the multirobot communication network
- robot_map: all the various interconnections between robots as defined in the configuration file
- topics: all the topics through which the robots are exchanging messages
- topic remaps: a list of topics that have been remapped before use


@Author: Kizito Masaba and Alberto Quattrini Li
'''

# ids to identify the robot type
BS_TYPE = 1
RR_TYPE = 2
FR_TYPE = 3

WIFI_RANGE = 100
BLUETOOTH_RANGE = 10


class roscbt:
    def __init__(self):
        # data structures to store incoming messages that are pending forwarding

        self.publisher_map = {}

        rospy.init_node('roscbt', anonymous=True)

        # ROSCBT needs these two topics to implement the communication model
        self.robot_pose = {}
        self.pose_topic = rospy.get_param("/roscbt/pose_topic", [])

        # we can load the map as an image to determine the location of obstacles in the environment
        self.map_topic = rospy.get_param("map_topic", '')

        self.robot_ids = rospy.get_param("/roscbt/robot_ids", [])
        self.robot_types = rospy.get_param("/roscbt/robot_types", {})
        self.topics = rospy.get_param("/roscbt/topics", [])

        # new parameter -- specifying how information is shared among robots
        self.shared_topics = rospy.get_param("/roscbt/shared_topics", {})

        # processing groundtruth about the map
        map_image_path = rospy.get_param("/roscbt/map_image_path", '')
        self.map_size, self.map_pixels = self.read_map_image(map_image_path)
        if not self.map_pixels:
            rospy.loginfo("File not found on path: {}".format(map_image_path))
            exit(1)
        self.world_scale = rospy.get_param("/roscbt/world_scale", 1)
        self.map_pose = rospy.get_param("/roscbt/map_pose", [])
        self.world_center = rospy.get_param("/roscbt/world_center", [])

        # difference in center of map image and actual simulation
        self.dx = self.world_center[0] - self.map_pose[0]
        self.dy = self.world_center[1] - self.map_pose[1]

        # import message types
        for topic in self.topics:
            msg_pkg = topic["message_pkg"]
            msg_type = topic["message_type"]
            topic_name = topic["name"]
            exec("from {}.msg import {}\n".format(msg_pkg, msg_type))

            # creating publishers data structure
            self.publisher_map[topic_name] = {}

        for i in self.robot_ids:
            exec("def a_{}(self, data): self.robot_pose[{}] = data".format(i, i))
            exec("setattr(roscbt, 'callback_pos_teammate" + str(i) + "', a_" + str(i) + ")")
            exec("rospy.Subscriber('/robot_{}/{}', {}, self.callback_pos_teammate{}, queue_size = 100)".format(i,
                                                                                                               self.pose_topic[
                                                                                                                   0],
                                                                                                               self.pose_topic[
                                                                                                                   1],
                                                                                                               i))

        for robot_id, topic_map in self.shared_topics.items():
            for id, topic in topic_map.items():
                exec("def a_{0}_{1}(self, data):self.main_callback({0},{1},data)".format(robot_id, id))
                exec("setattr(roscbt, 'message_callback{0}_{1}', a_{0}_{1})".format(robot_id, id))
                exec(
                    "rospy.Subscriber('/robot_{0}_{1}/{2}', {3}, self.message_callback{0}_{1}, queue_size = 100)".format(
                        robot_id, id, topic[0], topic[1]))
                rospy.loginfo(
                    "Subscriptions: rospy.Subscriber('/robot_{0}_{1}/{2}', {3}, self.message_callback{0}_{1}, queue_size = 100)".format(
                        robot_id, id, topic[0], topic[1]))
                # populating publisher datastructure
                exec('pub=rospy.Publisher("/roscbt/robot_{}/{}", {}, queue_size=10)'.format(robot_id, topic[0],
                                                                                            topic[1]))
                exec('self.publisher_map[{}]=pub'.format(robot_id, id, i))

        rospy.loginfo("ROSCBT Initialized Successfully!")

    def spin(self):
        r = rospy.Rate(0.1)
        while not rospy.is_shutdown():
            try:
                r.sleep()
            except Exception as e:
                print('interrupted!')

    '''
    Convert the world  pose into a pixel position in the image
    :param pose - the x,y coordinate in the map, which is mapped to a pixel in the actual image of the map
    '''

    def pose_to_pixel(self, pose):
        # rescale negative values to start from 0
        rescaled_world_pose_x = pose.x + self.world_center[0]
        rescaled_world_pose_y = pose.y + self.world_center[1]

        # convert world pose to pixel positions
        row = self.map_size[1] - (rescaled_world_pose_y + self.dy) * self.world_scale
        col = (rescaled_world_pose_x + self.dx) * self.world_scale
        pixel_pose = (row, col)
        return pixel_pose

    '''
     Convert a pixel position into pixel position into a world pose
     :param pixel_pose - pixel position that shall be converted into a world position
     :return pose - the pose in the world that corresponds to the given pixel position
    '''

    def pixel_to_pose(self, pixel_pose):
        rescaled_world_pose_x = (pixel_pose[1] / self.world_scale) - self.dx
        rescaled_world_pose_y = ((self.map_size[1] - pixel_pose[0]) / self.world_scale) - self.dy

        x = rescaled_world_pose_x - self.world_center[0]
        y = rescaled_world_pose_y - self.world_center[1]

        return (x, y)

    '''
     Get pixel positions of all the neighboring pixels
     :param pixel_pos - center pixel from which to get the neighbors
     :param distance - this defines the width of the neighborhood
     :return neighbors - all valid pixel positions that are neighboring the given pixel within the image

    '''

    def get_pixel_neighbors(self, pixel_pos, distance=1):
        x = pixel_pos[0]
        y = pixel_pos[1]
        neighbors = []
        pixels = []
        for i in range(1, distance + 1):
            east = (x, y + i)
            north = (x, y + i)
            west = (x - i, y)
            south = (x - 1, y)
            ne = (x + i, y + i)
            nw = (x - i, y + i)
            se = (x + i, y - i)
            sw = (x - i, y - i)
            possible_neigbors = [east, north, west, south, ne, nw, se, sw]
            for n in possible_neigbors:
                if (n[1], n[0]) in self.map_pixels:
                    neighbors.append(n)
                    pixels.append(self.map_pixels[n])
        return neighbors, pixels

    def read_map_image(self, image_path):
        im = Image.open(image_path, 'r')
        pix_val = list(im.getdata())
        size = im.size
        pixel_values = {}
        for index in range(len(pix_val)):
            i = int(np.floor(index / size[0]))
            j = index % size[0]
            pixel_values[(i, j)] = pix_val[index][0]

        return size, pixel_values

    def main_callback(self, robot_id1, robot_id2, data):
        rospy.loginfo("New Message Received in main callback!")
        # handle all message types
        if self.can_communicate(robot_id1, robot_id2):
            self.publisher_map[robot_id2].publish(data)

    # method to check the constraints for robot communication
    def can_communicate(self, robot_id1, robot_id2):

        rospy.loginfo("CAN COMMUNICATE CALLED!")
        # change this depending on the structure of your pose message
        robot1_pose = self.robot_pose[robot_id1].pose.pose.position
        robot2_pose = self.robot_pose[robot_id2].pose.pose.position

        robot_1_pixel_pos = self.pose_to_pixel(robot1_pose)
        robot_2_pixel_pos = self.pose_to_pixel(robot2_pose)

        pixel1_neighbors, pixels1 = self.get_pixel_neighbors(robot_1_pixel_pos, distance=2)
        pixel2_neighbors, pixels2 = self.get_pixel_neighbors(robot_2_pixel_pos, distance=2)

        signal_path = self.compute_signal_path(self, robot1_pose, robot2_pose)
        # now get all the

        rospy.loginfo("Signal Path: {}".format(signal_path))
        rospy.loginfo(
            "Robot 1 Pose: ({},{}), Pixel Position: {} Neighboring Pixel Poses: {}, Pixels: {}".format(robot1_pose.x,
                                                                                                       robot1_pose.y,
                                                                                                       robot_1_pixel_pos,
                                                                                                       pixel1_neighbors,
                                                                                                       pixels1))

        rospy.loginfo(
            "Robot 2 Pose: ({},{}), Pixel Position: {} Neighboring Pixel Poses: {}, Pixels: {}".format(robot2_pose.x,
                                                                                                       robot2_pose.y,
                                                                                                       robot_2_pixel_pos,
                                                                                                       pixel2_neighbors,
                                                                                                       pixels2))
        robot1_type = self.robot_types[str(robot_id1)]
        robot2_type = self.robot_types[str(robot_id2)]

        return self.robots_inrange(robot1_pose, robot2_pose, robot1_type, robot2_type)

    '''
     Determine the points through which the signal is transmitted from the transmitter to the receiver (only considering line of sight)
    '''

    def compute_signal_path(self, transmitter_id, receiver_id):
        tx_pose = self.robot_pose[transmitter_id]
        rx_pose = self.robot_pose[receiver_id]
        x1 = tx_pose.x
        y1 = tx_pose.y

        x2 = rx_pose.x
        y2 = rx_pose.y
        way_points = []
        dy = y2 - y1
        dx = x2 - y2
        m = dy / dx
        if m > 1:
            x1 = rx_pose.y
            y1 = rx_pose.x
            x2 = rx_pose.y
            y2 = rx_pose.x

            dx = x1 - x2
            if dx < 0:
                points = self.bresenham_path(x1, y1, x2, y2)
            else:
                points = self.bresenham_path(x2, y2, x1, y1)

            # flip the raster so that bresenham algorithm can work normally. PS: remember to flip the resulting
            # coordinates
            way_points += [(v[1], v[0]) for v in points]
        elif m < 1:
            dx = x1 - x2
            if dx < 0:
                way_points += self.bresenham_path(x1, y1, x2, y2)
            else:
                # reverse the points for the sake of bresenham algorithm
                way_points += self.bresenham_path(x2, y2, x1, y1)

        else:
            # just pick all points along the straight line since the gradient is 1
            dx = x1 - x2
            if dx < 0:
                way_points += [(v, v) for v in range(x1, x2 + 1)]
            else:
                way_points += [(v, v) for v in range(x2, x1 + 1)]

        return way_points

    '''
    Get all the points traversed by a line that connects 2 points
    '''

    def bresenham_path(self, x1, y1, x2, y2):
        points = []

        x = x1
        y = y1
        dx = x2 - x1
        dy = y2 - y1
        p = 2 * dx - dy
        while (x <= x2):
            points.append((x, y))
            x += 1
            if p < 0:
                p = p + 2 * dy
            else:
                p = p + 2 * dy - 2 * dx
                y += 1
        return points

    # computes euclidean distance between two cartesian coordinates
    def robots_inrange(self, loc1, loc2, robot1_type, robot2_type):
        distance = math.sqrt(((loc1.x - loc2.x) ** 2) + ((loc1.y - loc2.y) ** 2))
        if robot1_type != FR_TYPE:
            return distance <= WIFI_RANGE
        return distance <= BLUETOOTH_RANGE


if __name__ == '__main__':
    cbt = roscbt()
    cbt.spin()