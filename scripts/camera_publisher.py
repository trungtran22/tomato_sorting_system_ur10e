#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
import cv2
import depthai as dai
import time
import numpy as np
from std_msgs.msg import String, Int32


def make_pose(x,y,z):
        pose = Pose()

        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        pose.orientation.x = 0
        pose.orientation.y = 1.0
        pose.orientation.z = 0
        pose.orientation.w = 0

        return pose

def remap_pose(x,y,z):

        x = (x/1000)*1.305 + 0.69
        y = (y/1000)*1.052 +0.33
        z = (z/1000) - 0.1

        return x,y,z

def detecting():
        pipeline = dai.Pipeline()   
        FRAME_SIZE = (640, 640)
        # DET_INPUT_SIZE = (640, 640)
        blob_path = "https://github.com/trungtran22/tomato_sorting_system_ur10e/blob/main/tomato_data/tomato.blob"
        labelMap = ["green","pink","redripe","turning"]
        syncNN = True
        # Define a source - color camera
        cam = pipeline.createColorCamera()
        cam.setPreviewSize(FRAME_SIZE[0], FRAME_SIZE[1])
        cam.setInterleaved(False)
        cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam.setBoardSocket(dai.CameraBoardSocket.RGB)

        mono_left = pipeline.createMonoCamera()
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        mono_right = pipeline.createMonoCamera()
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        
        stereo = pipeline.createStereoDepth()
        # setting node configs
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        # Align depth map to the perspective of RGB camera, on which inference is done
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        stereo.setOutputSize(mono_left.getResolutionWidth(), mono_left.getResolutionHeight())
        stereo.setSubpixel(True)

        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)



        # Define a neural network model
        tomato = pipeline.createYoloSpatialDetectionNetwork()
        tomato.setConfidenceThreshold(0.5)
        tomato.input.setBlocking(False)
        tomato.setBlobPath(blob_path)
        tomato.setDepthLowerThreshold(100)
        tomato.setDepthUpperThreshold(5000)

        tomato.setNumClasses(4)
        tomato.setCoordinateSize(4)
        tomato.setAnchors([10,14, 23,27, 37,58, 81,82, 135,169, 344,319])
        tomato.setAnchorMasks({ "side26": [1,2,3], "side13": [3,4,5] })
        tomato.setIouThreshold(0.5)

        xoutRgb = pipeline.create(dai.node.XLinkOut)
        xoutNN = pipeline.create(dai.node.XLinkOut)
        xoutDepth = pipeline.create(dai.node.XLinkOut)
        nnNetworkOut = pipeline.create(dai.node.XLinkOut)

        nnNetworkOut.setStreamName("nnNetwork")
        xoutRgb.setStreamName("rgb")
        xoutNN.setStreamName("detections")
        xoutDepth.setStreamName("depth")
        nnNetworkOut.setStreamName("nnNetwork")

        cam.preview.link(tomato.input)
        if syncNN:
            tomato.passthrough.link(xoutRgb.input)
        else:
            cam.preview.link(xoutRgb.input)

        tomato.out.link(xoutNN.input)

        stereo.depth.link(tomato.inputDepth)
        tomato.passthroughDepth.link(xoutDepth.input)
        tomato.outNetwork.link(nnNetworkOut.input)

        pub = rospy.Publisher('tomato_position', Pose, queue_size=10)
        pub_label = rospy.Publisher('tomato_label', String, queue_size=10)
        pub_rad = rospy.Publisher('tomato_rad', Int32, queue_size=10)
        rospy.init_node('oak_d', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        with dai.Device(pipeline) as device:
    
            previewQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            detectionNNQueue = device.getOutputQueue(name="detections", maxSize=4, blocking=False)
            depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
            networkQueue = device.getOutputQueue(name="nnNetwork", maxSize=4, blocking=False)

            startTime = time.monotonic()
            counter = 0
            fps = 0
            color = (255, 255, 255)
            bg_color = (0,0,0)
            printOutputLayersOnce = True

            while True:
            
                inPreview = previewQueue.get()
                inDet = detectionNNQueue.get()
                depth = depthQueue.get()
                inNN = networkQueue.get()
            
                if printOutputLayersOnce:
                    toPrint = 'Output layer names:'
                    for ten in inNN.getAllLayerNames():
                        toPrint = f'{toPrint} {ten},'
                    print(toPrint)
                    printOutputLayersOnce = False

                frame = inPreview.getCvFrame()
                depthFrame = depth.getFrame() # depthFrame values are in millimeters

                depth_downscaled = depthFrame[::4]
                if np.all(depth_downscaled == 0):
                    min_depth = 0  # Set a default minimum depth value when all elements are zero
                else:
                    min_depth = np.percentile(depth_downscaled[depth_downscaled != 0], 1)
                max_depth = np.percentile(depth_downscaled, 99)
                depthFrameColor = np.interp(depthFrame, (min_depth, max_depth), (0, 255)).astype(np.uint8)
                depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)

                counter+=1
                current_time = time.monotonic()
                if (current_time - startTime) > 1 :
                    fps = counter / (current_time - startTime)
                    counter = 0
                    startTime = current_time

                detections = inDet.detections

                # If the frame is available, draw bounding boxes on it pipeline = dai.Pipeline()nd show the frame
                height = frame.shape[0]
                width  = frame.shape[1]
                # global x,y,z
                x_ = 0
                y_ = 0
                z_ = 0
                label_ = ""
                radius = 0
                
                for detection in detections:
                    roiData = detection.boundingBoxMapping
                    roi = roiData.roi
                    roi = roi.denormalize(depthFrameColor.shape[1], depthFrameColor.shape[0])
                    topLeft = roi.topLeft()
                    bottomRight = roi.bottomRight()
                    xmin = int(topLeft.x)
                    ymin = int(topLeft.y)
                    xmax = int(bottomRight.x)
                    ymax = int(bottomRight.y)
                    cv2.rectangle(depthFrameColor, (xmin, ymin), (xmax, ymax), color, 1)

                    # Denormalize bounding box
                    x1 = int(detection.xmin * width)
                    x2 = int(detection.xmax * width)
                    y1 = int(detection.ymin * height)
                    y2 = int(detection.ymax * height)
                    try:
                        label = labelMap[detection.label]
                    except:
                        label = detection.label
                    cv2.putText(frame, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, bg_color,3,cv2.LINE_AA)
                    cv2.putText(frame, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color,1,cv2.LINE_AA)
                    cv2.putText(frame, "{:.2f}".format(detection.confidence*100), (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, bg_color,3,cv2.LINE_AA)
                    cv2.putText(frame, "{:.2f}".format(detection.confidence*100), (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color,1,cv2.LINE_AA)
                    cv2.putText(frame, f"X: {int(detection.spatialCoordinates.x)} mm", (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, bg_color,3,cv2.LINE_AA)
                    cv2.putText(frame, f"Y: {int(detection.spatialCoordinates.y)} mm", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_SIMPLEX, 0.5, bg_color,3,cv2.LINE_AA)
                    cv2.putText(frame, f"Z: {int(detection.spatialCoordinates.z)} mm", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, bg_color,3,cv2.LINE_AA)
                    cv2.putText(frame, f"X: {int(detection.spatialCoordinates.x)} mm", (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color,1,cv2.LINE_AA)
                    cv2.putText(frame, f"Y: {int(detection.spatialCoordinates.y)} mm", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color,1,cv2.LINE_AA)
                    cv2.putText(frame, f"Z: {int(detection.spatialCoordinates.z)} mm", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color,1,cv2.LINE_AA)

                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    gray = cv2.medianBlur(gray,5)
                    rows = gray.shape[0]
                    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows / 8, param1=200, param2=30, minRadius=30, maxRadius=100)
                
                    if circles is not None:
                        circles = np.uint16(np.around(circles))
                        for i in circles[0,:]:
                            center = (i[0], i[1])
                            #center
                            cv2.circle(frame,center,1,(0,100,100), 3)
                            #outline
                            radius = i[2]
                            cv2.circle(frame,center,radius,(255,0,255),3)

                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, cv2.FONT_HERSHEY_SIMPLEX)
                    #publish each type of tomato
                    if label == "redripe":
                        x_ = detection.spatialCoordinates.x
                        y_ = detection.spatialCoordinates.y
                        z_ = detection.spatialCoordinates.z
                    elif label == "green":
                        x_ = detection.spatialCoordinates.x
                        y_ = detection.spatialCoordinates.y
                        z_ = detection.spatialCoordinates.z
                    elif label == "turning":
                        x_ = detection.spatialCoordinates.x
                        y_ = detection.spatialCoordinates.y
                        z_ = detection.spatialCoordinates.z
                    elif label == "pink":
                        x_ = detection.spatialCoordinates.x
                        y_ = detection.spatialCoordinates.y
                        z_ = detection.spatialCoordinates.z
                    # self.label_ = label
                    # self.width_ = radius*2
                    label_ = label
                    # x_bot,y_bot,z_bot = self.remap_pose(x_cam,y_cam,z_cam)
                x_bot = x_
                y_bot = y_
                z_bot = z_
                pose_msg = make_pose(x_bot,y_bot,z_bot)
                print(pose_msg)
                pub.publish(pose_msg)
                pub_label.publish(label_)
                pub_rad.publish(radius)
                rate.sleep()


                cv2.putText(frame, "NN fps: {:.2f}".format(fps), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, bg_color,3,cv2.LINE_AA)
                cv2.putText(frame, "NN fps: {:.2f}".format(fps), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color,1,cv2.LINE_AA)
                cv2.imshow("depth", depthFrameColor)
                cv2.imshow("rgb", frame)

                if cv2.waitKey(1) == ord('q'):
                    
                    cv2.destroyAllWindows()
                    break
if __name__ == '__main__':
     detecting()
#     rospy.init_node('cam_info',anonymous=True)
#     rate = rospy.Rate(5) # 10hz
#     camera = DepthCam()
#     camera.detecting()
#     camera.pub_pose.publish(camera.pose_msg)
#     camera.pub_label.publish(camera.label_)
#     camera.pub_width.publish(camera.width_)
#     rate.sleep()
#     # try:
#     #     detecting()
#     # except rospy.ROSInterruptException:
#     #     pass
