# vision

## `hand_img.py`
- Input: Image containing some number of hands
- Output: CV2 frame showing hand landmarks

## `hand_stream.py`
- Input: Webcam input
- Output: Updating CV2 frame showing hand landmarks as well as a calculation of the openess a single given hand

## `facial_stream.py`
- Input: Webcam input
- Output: Updating CV2 frame showing face landmarks as well as an action for whether the hand robot should be opening or closing

## Hand Neural Network Method
1. Image in (3x 8-bit colour channels)
2. Hand detection neural network to crop image to hand
3. Convolutional neural network to identify hand landmarks
4. Feed-forward neural network to classify hand shapes
5. Classification out