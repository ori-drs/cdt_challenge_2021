#!/usr/bin/env python
import rospy
import rospkg
from cdt_msgs.msg import Frontiers, Graph, GraphNode, Object, ObjectList
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped

# Global variables
rospack = rospkg.RosPack()
path_pkg = rospack.get_path('gazebo_worlds_cdt')

# Offset for visualization markers
z_offset = 0.25
# List of meshes currently published
meshes = []

# Publishers
frontiers_vis_pub     = rospy.Publisher('/frontiers_marker', Marker, queue_size=10)
graph_vis_pub         = rospy.Publisher('/exploration_graph_marker', MarkerArray, queue_size=10)
explorer_goal_vis_pub = rospy.Publisher('/explorer_goal_marker', Marker, queue_size=10)

class Mesh: 
    def __init__(self, mesh_type, name, is_opaque=False, x=None, y=None, z=None, pub_topic=None):
        self.type = mesh_type
        self.name = name
        self.path = path_pkg + '/models/' + self.type + '/meshes/' + self.type +'.dae'
        
        self.is_opaque = is_opaque

        if pub_topic is None:
            self.pub = rospy.Publisher('/mesh_' + self.type + '_marker', Marker, queue_size=10)
        else:
            self.pub = rospy.Publisher(pub_topic, Marker, queue_size=10)
        
        self.msg = Marker()
        self.x = x
        self.y = y
        self.z = z

    def is_enabled(self):
        return rospy.get_param('/' + self.type + '/enabled', True)
    
    def update_position(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def build_mesh_msg(self):
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'cdt'
        marker.id = 0
        marker.action = Marker.ADD
        if self.is_opaque:
            marker.color.a = 1.0
        else:
            marker.color.a = 0.1
        marker.mesh_use_embedded_materials = True
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = 'file://' + self.path
        marker.pose.orientation.w = rospy.get_param('/mesh_' + self.type + '/ow', 1.)
        marker.pose.orientation.x = rospy.get_param('/mesh_' + self.type + '/ox', 0.)
        marker.pose.orientation.y = rospy.get_param('/mesh_' + self.type + '/oy', 0.)
        marker.pose.orientation.z = rospy.get_param('/mesh_' + self.type + '/oz', 0.)

        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1

        if self.x is None:
            marker.pose.position.x = rospy.get_param('/mesh_' + self.type + '/px', 0.)
        else:
            marker.pose.position.x = self.x
        
        if self.y is None:
            marker.pose.position.y = rospy.get_param('/mesh_' + self.type + '/py', 0.)
        else:
            marker.pose.position.y = self.y

        if self.z is None:
            marker.pose.position.z = rospy.get_param('/mesh_' + self.type + '/pz', 0.)
        else:
            marker.pose.position.z = self.z
        
        
        self.msg = marker


def objects_callback(msg):
    for obj in msg.objects:
        obj_in_meshes = False
        obj_name = obj.id + '_object'
        for mesh in meshes:
            if (obj_name == mesh.name):
                mesh.update_position(x=obj.position.x, y=obj.position.y, z=obj.position.z)
                obj_in_meshes = True
                break
        # Add mesh
        if not obj_in_meshes:
            meshes.append(Mesh(mesh_type=obj.id, name=obj_name, is_opaque=True, x=obj.position.x, y=obj.position.y, z=obj.position.z))


def graph_callback(msg):
    graph_markers = MarkerArray()

    marker_nodes = Marker()
    marker_nodes.header.frame_id = 'odom'
    marker_nodes.id = 0
    marker_nodes.ns = 'nodes'
    marker_nodes.type = Marker.SPHERE_LIST
    marker_nodes.action = Marker.ADD
    marker_nodes.scale.x = 0.2
    marker_nodes.scale.y = 0.2
    marker_nodes.scale.z = 0.2
    marker_nodes.color.a = 1.0
    marker_nodes.color.r = 0.0 # will show in blue
    marker_nodes.color.g = 0.0
    marker_nodes.color.b = 1.0
    marker_nodes.pose.orientation.w = 1.0
    marker_nodes.pose.position.x = 0
    marker_nodes.pose.position.y = 0
    marker_nodes.pose.position.z = z_offset

    marker_edges = Marker()
    marker_edges.header.frame_id = 'odom'
    marker_edges.id = 1
    marker_edges.ns = 'edges'
    marker_edges.type = Marker.LINE_LIST
    marker_edges.action = Marker.ADD
    marker_edges.scale.x = 0.08
    marker_edges.scale.y = 0.25
    marker_edges.scale.z = 0.25
    marker_edges.color.a = 1.0
    marker_edges.color.r = 0.0 # will show in blue
    marker_edges.color.g = 0.1
    marker_edges.color.b = 1.0
    marker_edges.pose.orientation.w = 1.0
    marker_edges.pose.position.x = 0
    marker_edges.pose.position.y = 0
    marker_edges.pose.position.z = z_offset

    marker_neighbors = Marker()
    marker_neighbors.header.frame_id = 'odom'
    marker_neighbors.id = 1
    marker_neighbors.ns = 'neighbors'
    marker_neighbors.type = Marker.LINE_LIST
    marker_neighbors.action = Marker.ADD
    marker_neighbors.scale.x = 0.05
    marker_neighbors.scale.y = 0.2
    marker_neighbors.scale.z = 0.2
    marker_neighbors.color.a = 1.0
    marker_neighbors.color.r = 0.0 # will show in blue
    marker_neighbors.color.g = 1.0
    marker_neighbors.color.b = 1.0
    marker_neighbors.pose.orientation.w = 1.0
    marker_neighbors.pose.position.x = 0
    marker_neighbors.pose.position.y = 0
    marker_neighbors.pose.position.z = z_offset

    first = True
    for node in msg.nodes:
        # Add nodes
        marker_nodes.points.append(node.pose.position)

        # Add edges
        marker_edges.points.append(node.pose.position)
        marker_edges.points.append(node.pose.position)

        # Add neighbors
        for id in node.neighbors_id:
            marker_neighbors.points.append(node.pose.position)
            marker_neighbors.points.append(msg.nodes[id.data].pose.position)

    # Remove initial and last edge marker to be consistent (2 points per edge)
    marker_edges.points.pop(0)
    marker_edges.points.pop(-1)

    # Add markers
    graph_markers.markers.append(marker_nodes)
    graph_markers.markers.append(marker_edges)
    graph_markers.markers.append(marker_neighbors)

    # Publish
    graph_vis_pub.publish(graph_markers)

def frontiers_callback(msg):
    marker = Marker()
    marker.header.frame_id = 'odom'
    marker.id = 0
    marker.ns = 'frontiers'
    marker.type = Marker.SPHERE_LIST
    marker.action = Marker.ADD
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1.0
    marker.color.r = 1.0 # will show in yellow
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = z_offset
    for frontier in msg.frontiers:
        marker.points.append(frontier.point)
    
    # Publish
    frontiers_vis_pub.publish(marker)

def explorer_goal_callback(msg):
    marker = Marker()
    marker.header.frame_id = 'odom'
    marker.id = 0
    marker.ns = 'explorer_goal'
    marker.type = Marker.SPHERE_LIST
    marker.action = Marker.ADD
    marker.scale.x = 0.3
    marker.scale.y = 0.3
    marker.scale.z = 0.3
    marker.color.a = 1.0
    marker.color.r = 1.0 # will show in red
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.points.append(msg.pose.position)
    
    # Publish
    explorer_goal_vis_pub.publish(marker)

def publish_meshes():
    for mesh in meshes:
        if mesh.is_enabled():
            mesh.build_mesh_msg()
            mesh.pub.publish(mesh.msg)


if __name__ == '__main__':
    rospy.init_node('cdt_visualizations')

    # Read parameter with mesh
    world_option = rospy.get_param('~world', 'arena_simple')
    meshes.append(Mesh(mesh_type=world_option, name=world_option,    is_opaque=True,  pub_topic='/mesh_world_marker'))
    meshes.append(Mesh(mesh_type='barrel',   name='barrel_actual',   is_opaque=False, pub_topic='/mesh_barrel_actual_marker'))
    meshes.append(Mesh(mesh_type='barrow',   name='barrow_actual',   is_opaque=False, pub_topic='/mesh_barrow_actual_marker'))
    meshes.append(Mesh(mesh_type='computer', name='computer_actual', is_opaque=False, pub_topic='/mesh_computer_actual_marker'))
    meshes.append(Mesh(mesh_type='dog',      name='dog_actual',      is_opaque=False, pub_topic='/mesh_dog_actual_marker'))


    # Objects
    objects_topic = rospy.get_param('~objects_topic', '/detected_objects')
    rospy.Subscriber(objects_topic, ObjectList, objects_callback, queue_size=10)

    # Frontiers
    frontiers_topic = rospy.get_param('~frontiers_topic', '/frontiers')
    rospy.Subscriber(frontiers_topic, Frontiers, frontiers_callback, queue_size=10)

    # Exploration graph
    graph_topic = rospy.get_param('~graph_topic', '/exploration_graph')
    rospy.Subscriber(graph_topic, Graph, graph_callback, queue_size=10)

    # World explorer goal
    explorer_goal = rospy.get_param('~world_explorer_goal', '/world_explorer/goal')
    rospy.Subscriber(explorer_goal, PoseStamped, explorer_goal_callback, queue_size=10)
    
    while not rospy.is_shutdown():
        publish_meshes()
        rospy.sleep(0.5)
