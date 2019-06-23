#!/usr/bin/env python
import os
import rospy
import cv2
import face_recognition
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import rospkg



class FaceRecogniser(object):

    def __init__(self, cameraTopic, isGazebo):
        rospy.loginfo("Start FaceRecogniser Init process...")
        # get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()
        # get the file path for face_recognition_pkg
        self.path_to_package = rospack.get_path('face_recognition_pkg')
    
        self.bridge_object = CvBridge()

        self.names = []
        self.photos = []

        if isGazebo :
            self.names = ['Bobeye','Naoko','Standing_person','Momotaz','Sajay','Dain']
            self.photos = ["person_img/bobeye.png",
                          "person_img/naoko.png",
                          "person_img/standing_person.png",
                          "person_img/momotaz.png",
                          "person_img/sajay.png",
                          "person_img/dain.png"]
        else :
            self.names = ['Tianyi','Alex','Dongpeng','Momotaz','Saja','Dain']
            self.photos = ["person_img/tianyi.png",
                          "person_img/alex.png",
                          "person_img/dongpeng.png",
                          "person_img/momotaz.png",
                          "person_img/sajay.png",
                          "person_img/dain.png"]


        rospy.loginfo("Start training...")
        self.known_faces = self.training()

        rospy.loginfo("Start camera suscriber...")
        self.image_sub = rospy.Subscriber(cameraTopic,Image,self.camera_callback)
        rospy.loginfo("Finished FaceRecogniser Init process...Ready")

            
    def camera_callback(self,data):
        
        self.recognise(data)

    def training(self):
        known_faces = []

        for i in range(5):
            print(self.photos[i])

            # Load a sample picture of each person you want to recognise.
            img_file = os.path.join(self.path_to_package,self.photos[i])
            p_image = face_recognition.load_image_file(img_file)

            # Get the face encodings for each face in each image file
            # Since there could be more than one face in each image, it returns a list of encordings.
            # But since I know each image only has one face, 
            # I only care about the first encoding in each image, so I grab index 0.

            known_faces.append(face_recognition.face_encodings(p_image)[0])

        return known_faces


    def recognise(self,data):
        
        # Get a reference to webcam #0 (the default one)
        try:
            # We select bgr8 because its the OpneCV encoding by default
            video_capture = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
                
        # Initialize some variables
        face_locations = []
        face_encodings = []
        face_names = []
        process_this_frame = True

        # Grab a single frame of video
        #ret, frame = video_capture.read()
        
        # Resize frame of video to 1/4 size for faster face recognition processing
        small_frame = cv2.resize(video_capture, (0, 0), fx=0.5, fy=0.5)
        
        # Only process every other frame of video to save time
        if process_this_frame:
            # Find all the faces and face encodings in the current frame of video
            face_locations = face_recognition.face_locations(small_frame)
            face_encodings = face_recognition.face_encodings(small_frame, face_locations)
    
            if not face_encodings:
                rospy.logwarn("No Faces found, please get closer...")
    
            face_names = []
            for face_encoding in face_encodings:
                # See if the face is a match for the known face(s)
                match = face_recognition.compare_faces(self.known_faces, face_encoding)
                name = "Unknown"
    
                if match[0] and not match[1] and not match[2] and not match[3] and not match[4]:
                    name = self.names[0]
                elif not match[0] and match[1] and not match[2] and not match[3] and not match[4]:
                    name = self.names[1]
                elif not match[0] and not match[1] and match[2] and not match[3] and not match[4]:
                    name = self.names[2]
                elif not match[0] and not match[1] and not match[2] and match[3] and not match[4]:
                    name = self.names[3]
                elif not match[0] and not match[1] and not match[2] and not match[3] and match[4]:
                    name = self.names[4]
                elif not match[0] and not match[1] and not match[2] and not match[3] and not match[4]:
                    name = ""
                else:
                    name = "SOMETHING_IS_WRONG"
                rospy.loginfo(name)
    
                face_names.append(name)
    
        process_this_frame = not process_this_frame
    
        
        # Display the results
        for (top, right, bottom, left), name in zip(face_locations, face_names):
            # Scale back up face locations since the frame we detected in was scaled to 1/4 size
            top *= 2
            right *= 2
            bottom *= 2
            left *= 2
    
            # Draw a box around the face
            cv2.rectangle(video_capture, (left, top), (right, bottom), (0, 0, 255), 2)
    
            # Draw a label with a name below the face
            cv2.rectangle(video_capture, (left, bottom - 35), (right, bottom), (0, 0, 255))
            font = cv2.FONT_HERSHEY_DUPLEX
            cv2.putText(video_capture, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)
            
        
        # Display the resulting image
        cv2.imshow("Image window", video_capture)
        cv2.waitKey(1)
        



def main():
    rospy.init_node('face_recognising_python_node', anonymous=True)

    isGazebo = rospy.get_param('~isGazebo')
    cameraTopic = rospy.get_param('~cameraTopic')
   
    line_follower_object = FaceRecogniser(cameraTopic, isGazebo)

    rospy.spin()
    cv2.destroyAllWindows()

    
if __name__ == '__main__':
    main()
