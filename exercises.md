# OpenCV in ROS

## Preparations

Install usb_cam ros package

    sudo apt-get install ros-indigo-usb-cam
    dpkg -L ros-indigo-usb-cam

dpkg -L ros-indigo-vision-opencv

Install package to read images from USB camera

    cd ~/catkin_ws/src
    git clone github.com/dortmans/usb_camera 

Install opencv support package

    git clone github.com/dortmans/ros_opencv
    chmod +x `rospack find ros_opencv`/scripts/*

## Starting the USB camera node

Start USB camera node

    roslaunch usb_camera usb_camera.launch testimage.jpg

Camera images are published on topic */usb_cam/image_raw*

Check if camera is working:

    roslaunch ros_opencv facedetect.launch image:=/usb_cam/image_raw

Impressed? This shows you how powerful the combination of Python and OpenCV is. 

## Starting a fake camera node

For testing purposes we have prepared a fake camera node. It reads an image from a file and publishes this image as if it were a real camera. The fake camera output is published on topic */fake_cam/image_raw*

Let us test it:

    roslaunch ros_opencv fake_camera.launch image:=testimage.jpg 

Check if camera is working:

    rosrun image_view image_view image:=/fake_cam/image_raw

## A basic image processing node

We have prepared a simple image processing node, to be used as a start for building customized image processing nodes.
Our basic image processing node *image_processor.py* receives images from the camera topic, shows them in a window on the screen just like the *image_view* node, and publishes the image unchanged on the */processed_image* topic.

Try it:

    roslaunch ros_opencv image_processing.launch image:=/usb_cam/image_raw

Stop the node (ctrl-c or close terminal window).

Study the sourcecode of the node. Do you understand how it works?
Discovered where you are supposed to add your OpenCV code?
At this location in the code add the following line to *blur* the image:

    processed_image = cv2.GaussianBlur(processed_image,(15,15),0)

Save and launch the node.

As you can see we applied way too much blur. In fact we specified that the image must be convolved (i.e. apply *convolution*) with a special Gaussian *kernel*, (usually called *brush* in picture editing programs like Photoshop) of 15 by 15 pixels. If you just want to remove some camera noise you could replace (15,15) by (3,3) or (5, 5). In this case it works as a mild low-pass filter. Edit you image processing node accordingly and test it again.


## Detecting industrial objects

In a lot of robotic applications wthere is a need to detect objects, for instance to pick them up. let us try to do that using OpenCV.

Make a copy of the *image_processor* node:

    roscd ros_opencv/scripts
    cp image_processor.py object_detector_1.py
    chmod +x object_detector_1.py

Open this new node in a text editor, e.g.:

    gedit object_detector_1.py

<TODO: describe how to do: grayscale/thresholding>

As you can see this procedure is not ideal for detecting objects, although often used in examples. In a lot of cases there is no clear intensity threshold that makes it possible to separate object from background. This certainly holds for shiny, dirty industrial objects on industrial backgrounds.

## Detecting colored objects or removing colored backgrounds

In the following situations we can use thresholding based on color:

1. The background has a specific color that is different from the (colorless) objects
2. The objects we are looking for have a specific color, different from the background

<TODO: describe how to do: rgb2hsv/inrange>

Now that we have a clear image with blobs we can easily derive contours of the object(s) we were looking for. We will deal with contours in the following sections.

The image with blobs can also be used as a mask for the original image. This way we can remove certain information from the image.

<TODO: describe how to do: masking>

## Better way of detecting industrial objects

If detection based on color is not possible we need another way of detection. Instead of bruteforce thresholding we will use edge detection. We will look for steep gradients in the grayscale image. From these edges we can then derive contours of the objects we are looking for.

<TODO: describe how to do: grayscale/edgedetection>

# Contours

Now that we have either an image with blobs or with edges we are not done yet. We need to find contours along the edges (of the blobs).

<TODO: describe how to do: findcontours>

# Contour selection

Not all contours are interesting for our application.

<TODO: describe how to do: selection of contours (size, circularity, polygonsides,...)>

## Features from contours

<TODO: describe how to do: feature extraction from contours (pose, sizes) using moments, minarea rectangle, elipsefitting>

## Camera calibration

<TODO: describe camera calibration based on http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration

## Markers

<TODO: work with Alvar markers in ROS>







