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
        self.subscription    


    def listener_callback(self, msg):
        # print('info:', msg) 
        stamp=msg.stamp
        name=msg.name
        file=msg.file 
        data=msg.msg 
        
        td=datetime.fromtimestamp(stamp.sec).strftime("%m/%d/%Y : %H:%M:%S")

        date=datetime.fromtimestamp(stamp.sec).strftime("%m/%d/%Y")
        # print(f'\ntime={td}')
        # print(f'name={name}')
        # print(f'file={file}')
        # print(f'data={data}')

        packat={}
        packat['time']=td
        packat['name']=name
        packat['file']=file
        packat['data']=data

        # data=json.dumps(packat)
        # data={td:data} 
        # jackal_ref.set(data)
        print("td=",td)

        # db.collection('hello').document('jackal').set({
        #     "time": td,
        #     "name": name,
        #     "file": file,
        #     "data": data
        # }, merge=True)

        # db.collection("hello").document().set(packat);
        today=time.strftime("%m-%d-%Y", time.gmtime())
        stamp=str(stamp.sec)+'_'+str(stamp.nanosec)
        print('stamp=',stamp)
        dref = collection_ref.document(today)
        dref.set({stamp: {'name':name,'file':file, "data":data} }, merge=True )




def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = LogSubscriber() 
    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
