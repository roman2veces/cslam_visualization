# import threading
import rclpy
import json
import os.path
from rclpy.node import Node

from cslam_common_interfaces.msg import PoseGraph
from cslam_common_interfaces.msg import PoseGraphValue
from cslam_common_interfaces.msg import PoseGraphEdge
from cslam_common_interfaces.msg import MultiRobotKey
from visualization_msgs.msg import MarkerArray, Marker
from distinctipy import distinctipy

from geometry_msgs.msg import Pose # TODO: remove this

class PoseGraphVisualizer():

    def __init__(self, node, params):
        self.node = node
        self.params = params
        self.pose_graph_markers_publisher = self.node.create_publisher(
            MarkerArray, "/cslam/viz/pose_graph_markers", 10)
        self.nb_colors = self.params["nb_colors"]
        self.visualizer_update_period_ms_ = self.params["visualization_update_period_ms"]  
        self.colors = distinctipy.get_colors(self.nb_colors)
        self.pose_graph_subscriber = self.node.create_subscription(
            PoseGraph, '/cslam/viz/pose_graph', self.pose_graph_callback, 10)
        self.robot_pose_graphs = {}
        self.robot_pose_graphs_edges = {}
        self.pose_graph_to_store = {}
        self.origin_robot_ids = {}
        self.timer = self.node.create_timer(
            self.visualizer_update_period_ms_ / 1000.0,
            self.visualization_callback)
        
        # timer = threading.Timer(5.0, self.retrieve_pose_graph)
        # timer.start()

    def retrieve_pose_graph(self):
        pose_graph_path = self.params['map_path'] + '/pose_graph.json'

        # TODO: recycle code
        # TODO: bug is path doesnt exist
        with open(pose_graph_path, 'r') as file:
            global_pose_graph = json.load(file)
            for robot_id, robot_pose_graph in global_pose_graph.items():
                robot_id_int = int(robot_id)
                self.origin_robot_ids[robot_id_int] = robot_id_int
                
                if robot_id_int not in self.robot_pose_graphs:
                    self.robot_pose_graphs[robot_id_int] = {}

                for keyframe_id, pose_dict in robot_pose_graph["values"].items():
                    keyframe_id_int = int(keyframe_id)
                    pose_graph_value = PoseGraphValue()
                    pose_graph_value.key = MultiRobotKey()
                    pose_graph_value.key.robot_id = robot_id_int
                    pose_graph_value.key.keyframe_id = keyframe_id_int
                    pose_graph_value.pose = self.dict_to_pose(pose_dict)
                    self.robot_pose_graphs[robot_id_int][keyframe_id_int] = pose_graph_value

                if robot_id_int not in self.robot_pose_graphs_edges:
                    self.robot_pose_graphs_edges[robot_id_int] = []

                for edge_dict in robot_pose_graph["edges"]: 
                    pose_graph_edge = PoseGraphEdge()
                    pose_graph_edge.key_from = MultiRobotKey()
                    pose_graph_edge.key_from.robot_id = int(edge_dict["key_from"]["robot_id"])
                    pose_graph_edge.key_from.keyframe_id = int(edge_dict["key_from"]["keyframe_id"])
                    pose_graph_edge.key_to = MultiRobotKey()
                    pose_graph_edge.key_to.robot_id = int(edge_dict["key_to"]["robot_id"])
                    pose_graph_edge.key_to.keyframe_id = int(edge_dict["key_to"]["keyframe_id"])
                    
                    pose_graph_edge.measurement = self.dict_to_pose(edge_dict["measurement"])
                    pose_graph_edge.noise_std = edge_dict["noise_std"]
                    self.robot_pose_graphs_edges[robot_id_int].append(pose_graph_edge)
                

    def store_pose_graph(self, msg):
        # Make sure that intermediate directories exist
        os.makedirs(self.params["map_path"], exist_ok=True)

        pose_graph_path = self.params["map_path"] + "/pose_graph.json"
        with open(pose_graph_path, "w+") as json_file:
            # Initialize pose graph to be stored
            # pose_graph_to_store = {}

            # Read file is not empty
            # if os.path.getsize(pose_graph_path) != 0:
            #     pose_graph_to_store = json.load(json_file)

            # TODO: handle case when there is a previous different data 
            json.dump(self.pose_graph_to_store, json_file)

    def pose_graph_callback(self, msg):
        self.origin_robot_ids[msg.robot_id] = msg.origin_robot_id
        if msg.robot_id not in self.robot_pose_graphs:
            self.robot_pose_graphs[msg.robot_id] = {}
            self.pose_graph_to_store[msg.robot_id] = {
                "edges": {},
                "values": {}
            }

        for pose in msg.values:
            self.robot_pose_graphs[msg.robot_id][pose.key.keyframe_id] = pose
            self.pose_graph_to_store[msg.robot_id]["values"][pose.key.keyframe_id] = self.pose_graph_value_to_dict(pose)

        self.robot_pose_graphs_edges[msg.robot_id] = msg.edges
        self.pose_graph_to_store[msg.robot_id]["edges"] = list(map(self.pose_graph_edge_to_dict, msg.edges)) 
    
        self.store_pose_graph(msg)

    def robot_pose_graphs_to_marker_array(self):
        """Converts a PoseGraph messages to a MarkerArray message"""
        marker_array = MarkerArray()

        # Nodes (poses)
        for robot_id, pose_graph in self.robot_pose_graphs.items():
            color = self.colors[robot_id % self.nb_colors]
            marker = Marker()
            marker.header.frame_id = "robot" + str(self.origin_robot_ids[robot_id]) + "_map"
            marker.header.stamp = rclpy.time.Time().to_msg()
            marker.ns = "poses"
            marker.id = robot_id
            marker.type = Marker.SPHERE_LIST
            marker.action = Marker.ADD
            marker.scale.x = self.params["pose_graph_markers_size"]
            marker.scale.y = self.params["pose_graph_markers_size"]
            marker.scale.z = self.params["pose_graph_markers_size"]
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 1.0
            marker.frame_locked = True
            for key, node in pose_graph.items():
                marker.points.append(node.pose.position)
            marker_array.markers.append(marker)

        # Edges (constraints)
        for robot_id, pose_graph in self.robot_pose_graphs.items():
            color = self.colors[robot_id % self.nb_colors]
            marker = Marker()
            marker.header.frame_id = "robot" + str(self.origin_robot_ids[robot_id]) + "_map"
            marker.header.stamp = rclpy.time.Time().to_msg()
            marker.ns = "edges"
            marker.id = robot_id
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD
            marker.scale.x = self.params["pose_graph_markers_size"] / 2
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 1.0
            marker.frame_locked = False
            for edge in self.robot_pose_graphs_edges[robot_id]:
                if edge.key_from.robot_id in self.robot_pose_graphs and edge.key_to.robot_id in self.robot_pose_graphs:
                    if edge.key_from.keyframe_id in self.robot_pose_graphs[edge.key_from.robot_id] and edge.key_to.keyframe_id in self.robot_pose_graphs[edge.key_to.robot_id]:
                        marker.points.append(
                            self.robot_pose_graphs[edge.key_from.robot_id][edge.key_from.keyframe_id].pose.position)
                        marker.points.append(
                            self.robot_pose_graphs[edge.key_to.robot_id][edge.key_to.keyframe_id].pose.position)
            marker_array.markers.append(marker)

        return marker_array

    # TODO: Document this method
    def pose_graph_value_to_dict(self, pose_graph_value):
        return {
            "position": {
                "x": pose_graph_value.pose.position.x,
                "y": pose_graph_value.pose.position.y,
                "z": pose_graph_value.pose.position.z
            },
            "orientation": {
                "x": pose_graph_value.pose.orientation.x,
                "y": pose_graph_value.pose.orientation.y,
                "z": pose_graph_value.pose.orientation.z,
                "w": pose_graph_value.pose.orientation.w
            }
        }
    
    # TODO: Document this method
    def pose_graph_edge_to_dict(self, edge):
        return {
            "key_from": {
                "robot_id": edge.key_from.robot_id,
                "keyframe_id": edge.key_from.keyframe_id
            },
            "key_to": {
                "robot_id": edge.key_to.robot_id,
                "keyframe_id": edge.key_to.keyframe_id
            },
            "measurement": {
                "position": {
                    "x": edge.measurement.position.x,
                    "y": edge.measurement.position.y,
                    "z": edge.measurement.position.z
                },
                "orientation": {
                    "x": edge.measurement.orientation.x,
                    "y": edge.measurement.orientation.y,
                    "z": edge.measurement.orientation.z,
                    "w": edge.measurement.orientation.w,
                },
            },
            "noise_std": edge.noise_std.tolist()
        }

    def dict_to_pose(self, dict):
        pose = Pose()
        pose.position.x = dict['position']['x']
        pose.position.y = dict['position']['y']
        pose.position.z = dict['position']['z']
        pose.orientation.x = dict['orientation']['x']
        pose.orientation.y = dict['orientation']['y']
        pose.orientation.z = dict['orientation']['z']
        pose.orientation.w = dict['orientation']['w']
        return pose

    def visualization_callback(self):
        marker_array = self.robot_pose_graphs_to_marker_array()
        self.pose_graph_markers_publisher.publish(marker_array)
