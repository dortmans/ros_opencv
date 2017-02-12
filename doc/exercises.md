# OpenCV in ROS

> Author: Eric Dortmans (e.dortmans@fontys.nl)

This document contains exercises for a course on *OpenCV in ROS*.

## Preparations

Check if ROS package *vision_opencv* is installed. 

```
rospack find cv_bridge
```

It probably is installed, but if not install it:

```
sudo apt-get install ros-indigo-vision-opencv
```

Install the *usb_cam* and the *usb_camera* packages to read images from USB camera

```
sudo apt-get install ros-indigo-usb-cam
cd ~/catkin_ws/src
git clone https://github.com/dortmans/usb_camera.git
cd ~/catkin_ws
catkin_make
```

Install the *ros_opencv* course support package

```
cd ~/catkin_ws/src
git clone --depth 1 https://github.com/dortmans/ros_opencv.git
chmod +x ros_opencv/scripts/*
cd ~/catkin_ws
catkin_make
```

## Starting the USB camera node

Start USB camera node

```
roslaunch usb_camera usb_camera.launch
```

Camera images are published on topic */usb_cam/image_raw*

**NOTE**: If you have problems getting your usb camera to work (probably because you are running Ubuntu in a Virtual Machine) just skip the rest of this exercise.

Check if your camera is working:

```
roslaunch ros_opencv facedetect.launch image:=/usb_cam/image_raw
```

This shows you how powerful the combination of Python and OpenCV is. Impressed? 

## Starting a fake camera node

For testing purposes we have prepared a fake camera node. It reads an image from a file and publishes this image as if it were a real camera. The fake camera output is published on topic */fake_cam/image_raw*

Let us test it:

```
roslaunch ros_opencv fake_camera.launch image:=testimage1.jpg
```

**NOTE**: All available image files are located in the *ros_opencv/images* directory.

**NOTE**: In the *ros_opencv/launch/fake_camera.launch* file you can see which parameters are supported by the fake_camera node. You can adapt the parameters if you like.

Use the ROS imageviewer to check if the fake camera is working:

```
rosrun image_view image_view image:=/fake_cam/image_raw
```

It should show the image that you loaded in the fake camera.

If you are done, stop the ROS imageviewer node (enter ctrl-c or close terminal window). 

We are now going to build our own image processor nodes.

## A basic image processing node

We have prepared a simple image processing node, to be used as a start for building customized image processing nodes. This basic image processing node *image_processor.py* receives images from the camera topic, shows them in a window on the screen just like the *image_view* node, and publishes the image unchanged on the */processed_image* topic.

Try it:

```
roslaunch ros_opencv image_processing.launch image:=/fake_cam/image_raw
```

You should see the image that is broadcasted by the fake camera in two windows, called *image* and *processed_image*. Both images are the same because we did not implement any image processing yet.

If you are done, stop the node (ctrl-c or close terminal window).

Study the sourcecode of the *image_processor* node. You can find it in the *ros_opencv/scripts* directory. Open it for editing:

```
roscd ros_opencv/scripts
gedit image_processor.py
```

In particular look at the functions with *TODO* in comments.

Do you understand how the node works?
Discovered where you are supposed to add your OpenCV code?

## Camera noise removal

Real live camera images usually contain some noise. Common practice in image processing is to apply some low-pass filtering on the image. Therefor add the following line in the TODO section of the *process_image()* function of your image processing node:

```
        self.image = cv2.GaussianBlur(self.image, (15, 15), 0)
```

Save and launch the node. What do you see?

In fact we specified that the image must be *blurred* using convolution with a *Gaussian kernel* of 15 by 15 pixels. You will notice that the image becomes quite vague. If you just want to remove some camera noise you should replace (15, 15) by (3, 3) in which case it works as a mild low-pass filter. Edit your image processing node accordingly and test it again.

## Detecting objects using color

In a lot of robotic applications there is a need to detect objects, for instance to pick them up. Let us try to do that using OpenCV.

In the following situations we can use color:

1. The background has a specific color that is different from the (colorless) object
2. The object we are looking for has a specific color, different from the background

Let us load a testimage in our fake camera. First stop the previous fake camera instance (e.g.by ctrl-c in the associated command window). Then restart it using a specific image file:

```
roslaunch ros_opencv fake_camera.launch image:=red_ball_green_large.jpg
```

Now make a copy of the *image_processor* node, and call it *object_detector_color*:

```
roscd ros_opencv/scripts
cp image_processor.py object_detector_color.py
chmod +x object_detector_color.py
```

We are now going to modify the image processing Python code. 
Open the new node in a text editor:

```
gedit object_detector_color.py
```

Suppose we would like to inspect the color value of the image pixel under the mouse pointer. To realize this add the following lines of code in the TODO section of the *on_mouse_event()* function:

```
        rgb = self.image[y:y + 1, x:x + 1]

        image_info_window = 255 * np.ones((100, 500, 3))

        cv2.putText(image_info_window, 'BGR (b=%d,g=%d,r=%d)' % (
            rgb[0, 0, 0], rgb[0, 0, 1], rgb[0, 0, 2]), (5, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0))
        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
        cv2.putText(image_info_window, 'HSV (h=%d,s=%d,v=%d)' % (
            hsv[0, 0, 0], hsv[0, 0, 1], hsv[0, 0, 2]), (5, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0))

        cv2.imshow('image_info', image_info_window)
        cv2.waitKey(3)
```

Try to start this node to check that you have a good working node:

```
roslaunch ros_opencv object_detector_color.launch image:=/fake_cam/image_raw
```

Move your mouse to the *image* window and see the pixel value under the mouse being printed in a separate *image_info* window.

Stop the node and open it again for editing.

We are now going to add some color processing code in the TODO section of the *process_image()* function.

RGB is not an really good color space for selecting colors, unless they are mostly red, green or blue. The HSV color space is more suited to do this. The H component specifies the color, the S component the saturation of the color and V the value (i.e. intensity) of the color. Conversion from RGB to HSV is simple. Just add following line of the code:

```
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
```

To apply color range filtering using the OpenCV inRange function add following lines:

```
        COLOR_MIN = np.array([self.h_min, self.s_min, self.v_min], np.uint8)
        COLOR_MAX = np.array([self.h_max, self.s_max, self.v_max], np.uint8)
        blobs = cv2.inRange(hsv, COLOR_MIN, COLOR_MAX)
        if (blobs[0, 0] == 255): blobs = cv2.bitwise_not(blobs)
        cv2.imshow('blobs', blobs)
```

The image with blobs can also be used as a mask for the original image. Add following line to achieve this:

```
        self.processed_image = cv2.bitwise_and(self.image, self.image, mask=blobs)
```

We need two HSV color thresholds to select a range of colors. In above piece of code we have called those thresholds *COLOR_MIN* and *COLOR_MAX*. 
Let us create trackbars to be able to select the right values. 

Add following code in the TODO section of the *process_image_setup()* function:

```
        self.h_min, self.h_max = 0, 179  # H range
        self.s_min, self.s_max = 0, 255  # S range
        self.v_min, self.v_max = 0, 255  # V range

        cv2.namedWindow('trackbars')
        cv2.createTrackbar('H_min', 'trackbars', self.h_min, 179, self.on_trackbar_change)
        cv2.createTrackbar('H_max', 'trackbars', self.h_max, 179, self.on_trackbar_change)
        cv2.createTrackbar('S_min', 'trackbars', self.s_min, 255, self.on_trackbar_change)
        cv2.createTrackbar('S_max', 'trackbars', self.s_max, 255, self.on_trackbar_change)
        cv2.createTrackbar('V_min', 'trackbars', self.v_min, 255, self.on_trackbar_change)
        cv2.createTrackbar('V_max', 'trackbars', self.v_max, 255, self.on_trackbar_change)
```

Add following code in the TODO section of the *on_trackbar_change()* function:

```
        self.h_min = cv2.getTrackbarPos('H_min', 'trackbars')
        self.s_min = cv2.getTrackbarPos('S_min', 'trackbars')
        self.v_min = cv2.getTrackbarPos('V_min', 'trackbars')
        self.h_max = cv2.getTrackbarPos('H_max', 'trackbars')
        self.s_max = cv2.getTrackbarPos('S_max', 'trackbars')
        self.v_max = cv2.getTrackbarPos('V_max', 'trackbars')
```

Now save the new object detector code and launch it:

```
roslaunch ros_opencv object_detector_color.launch image:=/fake_cam/image_raw
```

Play with the trackbars to find good values for your color range filter. Notice that you can either use a range that contains the color of the object or use a range that contains the color of the background.
   
Stop your color based object detector node now (e.g.by ctrl-c in the associated command window). Also stop the fake camera node.

## Detecting objects using thresholding

Industrial objects usually do not have colors. Also the background (table or belt) is usually not colored but black or gray. This means we can not use object detection based on color.

First approach we will try is to simply convert our image to a black & white image using *thresholding*.

Let us start the fake camera with a new test image:

```
roslaunch ros_opencv fake_camera.launch image:=all_objects.jpg
```

Now make a fresh copy of the *image_processor* node:

```
roscd ros_opencv/scripts
cp image_processor.py object_detector_threshold.py
chmod +x object_detector_threshold.py
```

Open the new node in a text editor, e.g.:

```
gedit object_detector_threshold.py
```

In order to apply an intensity threshold on the pixels we first need to convert our color image to a grayscale image. A color image has 3 image planes (Red, Green, Blue). A grayscale image has just one plane where each pixel has a values between 0 and 255, representing the light intensity. To convert a RGB image to grayscale add these lines of code:

```
        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        cv2.imshow('grayscale', gray)
```

Next step is to convert this grayscale image into a black & white image where each pixel is either 0 (black) or 255 (white).
In order to do this we need to *threshold* the grayscale image.

```
        _, blobs = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        if (blobs[0, 0] == 255): blobs = cv2.bitwise_not(blobs)
        cv2.imshow('blobs', blobs)

        self.processed_image = cv2.bitwise_and(self.image, self.image, mask=blobs)
```

Now save the new object detector node and start it:

```
roslaunch ros_opencv object_detector_threshold.launch image:=/fake_cam/image_raw
```

What do you think about the result?

## Detecting objects using edge detection

Thresholding is usually not ideal for detecting industrial objects. In a lot of cases there is no clear intensity threshold that makes it possible to separate object from background. This certainly holds for shiny, dirty industrial objects on industrial backgrounds.

We need another way of detection. Instead of bruteforce grayscale thresholding we will use edge detection. We will look for steep gradients in the grayscale image. From these edges we can then derive contours of the objects we are looking for.

Make another fresh copy of the *image_processor* node to make an object detector using edge detection:

```
roscd ros_opencv/scripts
cp image_processor.py object_detector_edges.py
chmod +x object_detector_edges.py
```

Open the new detector node in a text editor, e.g.:

```
gedit object_detector_edges.py
```

Add code to convert the RGB image to grayscale:

```
        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        cv2.imshow('grayscale', gray)
```

Next step is to detect edges in the grayscale image.

Detecting edges can be done this way:

```
        m = np.median(gray)
        sigma=0.33
        lower = int(max(0, (1.0 - sigma) * m))
        upper = int(min(255, (1.0 + sigma) * m))
        edges = cv2.Canny(gray, lower, upper)
        cv2.imshow("edges", edges)
```

Start this node to check that you have a good working node:

```
roslaunch ros_opencv object_detector_edges.launch image:=/fake_cam/image_raw
```



# Contours

Now that we have either an image with blobs or with edges we are not done yet. We need to find contours along the edges (of the blobs).

**NOTE**:  Edge detection can also be applied to find edges of blobs. Black & white images in OpenCV are grayscale images in which only the values 0 and 255 have been used.

Let us add contour detection functionality to our edge based detector.

In most cases it is useful to first close holes in the edges before further processing

```
        kernel = np.ones((5,5),np.uint8)
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, 1)
```

Then add the contour detection code:

```
        if cv2.__version__.startswith('2.'): # if OpenCV2
            (cnts, _) = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        else: # if OpenCV3
            (_, cnts, _) = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)   
        
        contoured = self.image.copy()
        cv2.drawContours(contoured, cnts, -1, (0, 0, 255), 3)
        cv2.imshow("contours", contoured)
```

We can also generate blobs from contours and use it as a mask like we did before:

```
        blobs = np.zeros(self.image.shape[:2], dtype=np.uint8)
        cv2.drawContours(blobs, cnts, -1, (255, 255, 255), -1)
        self.processed_image = cv2.bitwise_and(self.image, self.image, mask=blobs)
```


## Features from contours

What can we do with contours:

1. Extract features (e.g. centroid, areasize, bounding-rectangle)
1. Based on these features select contours that could represent what we are looking for

Add following code to your node to extract and show some features:

```
        featured = self.image.copy()
        for cnt in cnts:

            area = cv2.contourArea(cnt)
            if area > 1000:  # we are not interested in very small contours

                # centroid
                M = cv2.moments(cnt)
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                cv2.circle(featured, (cx, cy), 7, (0, 0, 255), -1)
                cv2.putText(featured, "({}, {})".format(cx, cy), (cx - 40, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

                # rotated bounding rectangle
                rect = cv2.minAreaRect(cnt)
                if cv2.__version__.startswith('2.'):  # if OpenCV2
                    corners = cv2.cv.BoxPoints(rect)
                else:   # if OpenCV3
                    corners = cv2.boxPoints(rect)
                corners = np.int0(corners)
                cv2.drawContours(featured, [corners], 0, (0, 0, 255), 3)

        cv2.imshow("features", featured)
    
```

Stop and restart your detector node to see the new output.

## Determine size of object

Restart the fake camera with a new testimage:

```
roslaunch ros_opencv fake_camera.launch image:=red_ball_tag.jpg
```

Now write an object detector node that can determine the diameter of the red ball, given the size of the marker in the same image. The size of the marker is 5 x 5 cm.

## Extra: Camera calibration

We assume the USB camera node is still running. If not start it.

Execute the procedure as described on following URL:
 
    http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration

To see the output of the calibration process look in the directory *~/.ros/camera_info*:

```
cat ~/.ros/camera_info/usb_camera0.yaml
```

## Extra: Markers

Install the "ar_markers" package:


```
cd ~/catkin_ws/src
git clone https://github.com/dortmans/ar_markers.git
cd ~/catkin_ws
catkin_make
```

The package contains a PDF file called with Alvar markers. This file *Markers_0-1-2.pdf* can be found in the *ar_markers/markers/* directory. Print it.

We assume the USB camera node is still running. If not start it.

Launch the Alvar AR tracking node:

```
roslaunch ar_markers ar_indiv_no_kinect.launch
```

Start rviz and add TF.

Move a piece of paper with one or more Alvar tags (as printed before) in front of the camera and see what is happing in TF.








