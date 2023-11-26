import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from rcl_interfaces.msg import Log
from datetime import datetime
import os 
import json 
import firebase_admin
from firebase_admin import credentials, firestore
from firebase_admin import db
import time 

"""
upload to firebase.

setup: put the firebase json file to ~/.carl_firebase/secret/<filename.json>

"""

print('connecting to firebase...')

secret_file=os.path.expanduser('~')+'/.carl_firebase/secret/weblog-76d97-firebase-adminsdk-z1c71-80ee12d0de.json'

cred = credentials.Certificate(secret_file)
firebase_admin.initialize_app(cred)

db = firestore.client()
# ref = db.collection(u'hello')
 
# jackal_ref = ref.document(u'jackal')

collection_ref = db.collection('jackal')


class LogSubscriber(Node):

    def __init__(self):
        super().__init__('log_rosout')
        print('log_rosout node started...')
        self.subscription = self.create_subscription(
            Log,
            'rosout',
            self.listener_callback,
            10)



    def listener_callback(self, msg):
        # print('info:', msg) 
        stamp=msg.stamp
        name=msg.name
        file=msg.file 
        data=msg.msg 
        function=msg.function

        # db.collection('hello').document('jackal').set({
        #     "time": td,
        #     "name": name,
        #     "file": file,
        #     "data": data
        # }, merge=True)

        # db.collection("hello").document().set(packat);

        magic_key='weblog='
        if data.startswith(magic_key):
            data=data.replace(magic_key,'')
            timestamp = datetime.now().strftime("%Y-%m-%d")
            stamp=str(stamp.sec)+'_'+str(stamp.nanosec)
            print('stamp=',stamp)
            dref = collection_ref.document(timestamp)
            dref.set({stamp: {'name':name,'file':file, "data":data , "function":function } }, merge=True )




def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = LogSubscriber() 
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
