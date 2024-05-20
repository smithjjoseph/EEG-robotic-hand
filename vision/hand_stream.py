#!/usr/bin/python3

"""
:file: hand_stream.py
:brief: Identifies hand landmarks from a video stream using MediaPipe and 
        calculates a level of closedness for the hand
:TODO: Reduce resolution of changes to 2-4%
"""

import sys
import cv2
import mediapipe as mp
from pathlib import Path

# Add project top level to path if run from this file
if __name__ == '__main__':
    path = str(Path(__file__, '..', '..').resolve())
    sys.path.append(path)
from ROS.complex_control import HandControl

WINDOW_NAME = 'Hand Detection'
WEBCAM_RES = (960, 720)
# As defined in:
# https://developers.google.com/mediapipe/solutions/vision/hand_landmarker#models
INDEX_FINGER_TIP = 8

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_draw_styles = mp.solutions.drawing_styles
hands = mp_hands.Hands(max_num_hands=1,
                       min_detection_confidence=0.8,
                       min_tracking_confidence=0.5)


class HandStream:
    def __init__(self, hand_connected:bool) -> None:
        self.hand_connected = hand_connected
        if hand_connected:
            self.control = HandControl()


    def find_percent_closed(self, image, hand_landmarks) -> None:
        if not hand_landmarks:
            return

        # Get all landmarks of the first visible hand
        landmarks = hand_landmarks[0].landmark

        # For demo purposes only necessary to use index finger
        # wrist -> mcp -> pip -> dip -> tip
        wrist_y = landmarks[0].y
        tip_y = landmarks[INDEX_FINGER_TIP].y
        mcp_y = landmarks[INDEX_FINGER_TIP - 3].y

        if tip_y > wrist_y:
            cv2.putText(image, f'Rotate hand upright', (40, 50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            return
        
        palm_reference = wrist_y - mcp_y
        tip_reference = wrist_y - tip_y

        ratio_closed = palm_reference/tip_reference
        ratio_closed = min(1.5, max(0, ratio_closed))
        percent_closed = min(1, 1.5-ratio_closed) *-100 + 100

        if self.hand_connected:
            self.control.movement(percent_closed)

        cv2.putText(image, f"{percent_closed=:.1f}%", (40, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)


    def draw_hand_landmarks(self, image, hand_landmarks) -> None:
        if not hand_landmarks:
            return

        for landmarks in hand_landmarks:
            mp_drawing.draw_landmarks(
                image, 
                landmarks, 
                mp_hands.HAND_CONNECTIONS,
                mp_draw_styles.get_default_hand_landmarks_style(),
                mp_draw_styles.get_default_hand_connections_style())


    def main(self) -> None:
        cap = cv2.VideoCapture(0)

        while cap.isOpened():
            success, image = cap.read()

            # Avoids empty images from causing errors
            if not success or image is None:
                continue

            image = cv2.flip(image, 1)
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = hands.process(image_rgb)

            hand_landmarks = results.multi_hand_landmarks
            self.draw_hand_landmarks(image, hand_landmarks)
            self.find_percent_closed(image, hand_landmarks)

            cv2.namedWindow(WINDOW_NAME ,cv2.WINDOW_GUI_NORMAL)
            cv2.resizeWindow(WINDOW_NAME, int(WEBCAM_RES[0]/1.5), 
                            int(WEBCAM_RES[1]/1.5))
            cv2.imshow(WINDOW_NAME, image)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    hand_stream = HandStream(hand_connected=False)
    hand_stream.main()
