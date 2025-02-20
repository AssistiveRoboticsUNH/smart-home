
# Run stretch driver first to send motion commands
# ros2 launch stretch_core stretch_driver.launch.py 
import hello_helpers.hello_misc as hm

class MyNode(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)

    def main(self):
        hm.HelloNode.main(self, 'my_node', 'my_node', wait_for_first_pointcloud=False)

        # Tablet talking pose
        # self.move_to_pose({'joint_lift': 0.7, 'joint_arm': 0.0, 'joint_wrist_yaw': 1.85, 'joint_wrist_pitch': 1.9}, blocking=True)
        
        # Tablet talking pose without moving lift
        self.move_to_pose({'joint_arm': 0.0, 'joint_wrist_yaw': 1.7, 'joint_wrist_pitch': 1.9}, blocking=True)
        

        # Docking camera pose
        # self.move_to_pose({'joint_head_pan': -3.1416, 'joint_head_tilt': -0.8}, blocking=True)

        # Forward down pose
        # self.move_to_pose({'joint_head_pan': 0.1, 'joint_head_tilt': -0.4}, blocking=True)

node = MyNode()
node.main()
