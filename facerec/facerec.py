#!/usr/bin/env python 

from __future__ import print_function

import cv2
import face_recognition
import rospy
import time
from std_msgs.msg import Int8

video_capture = cv2.VideoCapture(0)

viren_image = face_recognition.load_image_file("viren.jpg")
viren_face = face_recognition.face_encodings(viren_image)[0]

tsig_image = face_recognition.load_image_file("tsig.jpg")
tsig_face = face_recognition.face_encodings(tsig_image)[0]

tracked_faces = []

# Create arrays of known face encodings and their names
known_face_encodings = [
    viren_face,
    tsig_face,
]
known_face_names = [
    'viren',
    'tsigabu',
]

scale_factor = 3

# Initialize some variables
face_locations = []
face_encodings = []
face_names = []
process_this_frame = True

frame_rate = {}

pub = rospy.Publisher('faceloc', Int8, queue_size=1)
rospy.init_node('facerec', anonymous=True)

while True:
    timer = int(time.time())

    frame_rate[timer] = frame_rate.get(timer, 0.0) + 1

    # Grab a single frame of video
    ret, frame = video_capture.read()

    # Resize frame of video to 1/4 size for faster face recognition processing
    small_frame = cv2.resize(frame, (0, 0), fx=1.0/scale_factor, fy=1.0/scale_factor)

    # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
    rgb_small_frame = small_frame[:, :, ::-1]

    # Only process every other frame of video to save time
    if process_this_frame:
        # Find all the faces and face encodings in the current frame of video
        face_locations = face_recognition.face_locations(rgb_small_frame)
        face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)

        face_names = []
        for face_encoding in face_encodings:
            # See if the face is a match for the known face(s)
            matches = face_recognition.compare_faces(known_face_encodings, face_encoding, tolerance=0.0)
            name = "Unknown"

            # If a match was found in known_face_encodings, just use the first one.
            if True in matches:
                first_match_index = matches.index(True)
                name = known_face_names[first_match_index]

            face_names.append(name)

    process_this_frame = not process_this_frame

    # Display the results
    for (top, right, bottom, left), name in zip(face_locations, face_names):
        # Scale back up face locations since the frame we detected in was scaled to 1/4 size
        top *= scale_factor
        right *= scale_factor
        bottom *= scale_factor
        left *= scale_factor

        #print(name)
        if name in tracked_faces:
            msg = Int8(int(left*100.0/580))
            rospy.loginfo(msg)
            pub.publish(msg)

        # Draw a box around the face
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)

        # Draw a label with a name below the face
        cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), -1)
        font = cv2.FONT_HERSHEY_DUPLEX
        cv2.putText(frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)

    # Display the resulting image
    cv2.imshow('Video', frame)

    # Hit 'q' on the keyboard to quit!
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release handle to the webcam
video_capture.release()
cv2.destroyAllWindows()

print('Average framerate: {}'.format(sum(frame_rate.values())/len(frame_rate.values())))
print(frame_rate)
