/*
 * Copyright (C) 2013 Emmanuel Durand
 *
 * This file is part of blobserver.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * blobserver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with blobserver.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * @mainpage.h
 * Contains the main page for doxygen
 */

#ifndef MAINPAGE_H
#define MAINPAGE_H

/*! \mainpage Blobserver
 * 
 **************
 * \section intro_sec What is Blobserver
 * 
 * Blobserver is an OSC-based server aimed at detecting entities (objects / people / light / ...), in any compatible image flow. Its structure is so that it should be relatively easy to add new actuators as well as new image sources. As of yet, configuration and communication with Blobserver is done entirely through OSC messaging. Some kind of configuration file will be added soon to simplify the setup and usability, especially for permanent installations.
 * 
 * Blobserver is built around the concept of flow. A flow is the association of a actuator and as many sources as needed for it to work correctly. At each frame, all the flows are evaluated, and the various objects detected are sent through OSC to the corresponding clients.
 * 
 **************
 * \section sources_sec List of compatible sources
 * 
 * The following sources are compatible with Blobserver:
 * - any source compatible with OpenCV
 * - shmdata sources
 *
 * Some parameters are available for all kind of 2D sources, none if these transformations are activated by default:
 * - mask (string): file path to the image file to use as a mask
 * - autoExposure (int[7]): parameters for auto exposure, measured in a specified area. Parameters are: [x] [y] [width] [height] [target] [margin] [updateStep%].
 * - exposureLUT (float[2 + i*2]): specify a LUT for the exposure. Parameters are: [number of keys] [interpolation type] [[in key] [out key]]. Interpolation should currently be set to 0
 * - gainLUT (float[2 + i*2]): specify a LUT for the gain. Parameters are: [number of keys] [interpolation type] [[in key] [out key]]. Interpolation should currently be set to 0
 * - scale (float, default 1.0): apply scaling on the image
 * - rotation (float): apply rotation on the image, in degrees
 * - noiseFiltering (int, default 0): set to 1 to activate noise filtering
 * - distortion (int[3]): distortion correction (see http://wiki.panotools.org/Lens_correction_model). Parameters are: [a] [b] [c]
 * - fisheye (float[2]): fisheye correction (see http://wiki.panotools.org/Fisheye_Projection). Parameters are, in pixels: [fisheyeFocal] [rectilinearFocal] 
 * - vignetting (int[3]): correction of the vignetting (see http://lensfun.berlios.de/lens-calibration/lens-vignetting.html). Parameters are: [k1] [k2] [k3]
 * - iccInputProfile (string): file path to an ICC profile (for color correction)
 * - hdri (int[5]): activates the creation of a HDR image. Parameters are: [startExposure] [stepSize] [nbrSteps] [frameSkip] [continuousHDRActive].
 * - save (int[2] string): activates the automatic save of grabs. Parameters are: [activation] [period] [filename] 
 * 
 * \subsection source_2d_opencv_sec OpenCV 2D sources (Source_2D_OpenCV)
 * 
 * Note that OpenCV must have been compiled with the desired camera support.
 * 
 * Available parameters:
 * - cameraNumber (int): camera index as per OpenCV numbering. Setting this to 0 will use the first camera found.
 * - width (int)
 * - height (int)
 * - framerate (int)
 * - exposureTime (int): exposure time, conversion to ms is dependent of the camera model
 * - gain (int): gain applied to the camera sensor output, conversion to dB is dependent of the camera model
 * - gamma (int): gamma applied to the image, multiplied by 1024
 * - whiteBalanceRed (int): coefficient applied to the red channel, multiplied by a value dependent of the camera model
 * - whiteBalanceBlue (int): coefficient applied to the blue channel, multiplied by a value dependent of the camera model
 * - iso (int): link speed to set for firewire cameras
 * 
 * \subsection source_2d_gige_sec Gigabit ethernet cameras (Source_2D_Gige)
 *
 * This source is available only of blobserver was compiled with Aravis support (https://wiki.gnome.org/Aravis).
 * Most of the following parameters have no default but are dependent of the previous state of the camera.
 *
 * Available parameters:
 * - raw (int): if set to 1, the image is grabbed as a RAW image (i.e before Bayer conversion). Bayer conversion is then done by blobserver.
 * - width (int): desired width of the capture
 * - height (int): desired height of the capture
 * - offsetX (int): offset of the capture along X axis
 * - offsetY (int): offset of the capture along Y axis
 * - binning (int): number of pixels, on both axis, which are merged together (useful to get higher sensitivity, but lower resolution)
 * - invertRGB (int): if set to 1, a conversion from BGR to RGB is done
 * - framerate (float): framerate of the capture
 * - exposureTime (float): time of the exposure, in us
 * - gain (float): gain applied to the sensor, in dB
 *
 * \subsection source_2d_shmdata_sec shmdata 2D sources (Source_2D_Shmdata)
 * 
 * Available parameters:
 * - location (string): file path to the shmdata
 * - cameraNumber (int): index of the shmdata, this can be used to access multiple times the same shmdata
 *
 * \subsection source_3d_shmdata_sec shmdata 3D sources (Source_3D_Shmdata)
 *
 * This source is a 3D source (so none of the parameters specific to 2D sources are availables), and it outputs point clouds.
 *
 * Available parameters:
 * - location (string): file path to the shmdata
 * - cameraNumber (int): index of the shmdata, this can be used to access multiple times the same shmdata
 * 
 **************
 * \section actuators_sec List of actuators
 * 
 * \subsection actuator_armpcl_sec Detection of one's arm in his point cloud (Actuator_ArmPcl)
 *
 * This actuator detects the arm (or rather, the farthest part of the body) in a point cloud representing the body of one person. The position of the arm is the outputted.
 *
 * Number of source(s) needed: 1 Source_3D
 *
 * Available parameters:
 * - outputType (int, default 0): selects the type of visual output. 0 is for a 2D representation of the direction pointed by the arm, 1 for the point cloud of the arm
 * - neighboursNbr (int, default 200): maximum number of neighbouring points to keep in the detection
 * - maxDistanceFromMean (float, default 0.7): maximum distance where to search for the arm. Used to get rid of noise (usually farther than the arm in the cloud)
 * - mainAxis (int, default 0): specifies the main axis (meaning the axis of the body). 0, 1, 2 are for X, Y, Z. -1 is for autodetection
 * - maxManhattanDistance (float, default 0.1): maximum distance between the center of the arm's cloud and its neighbours
 * - minCloudSize (int, default 50): if the number of points in the cloud is lower than this value, the position is not outputted (used to get rid of noise)
 *
 * \subsection actuator_bgsubtractor_sec Background subtractor using mixtures of gaussians as models (Actuator_BgSubtractor)
 *
 * This actuator detects objects based on a model of the background which uses mixture of gaussians. It is mostly based on the implementation from OpenCV (http://docs.opencv.org/modules/video/doc/motion_analysis_and_object_tracking.html?#BackgroundSubtractorMOG2%20:%20public%20BackgroundSubtractor).
 * 
 * Number of source(s) needed: 1 Source_2D

 * Available parameters:
 * - filterSize (int, default 3): size of the morphologicial filter used to filter noise.
 * - filterDilateCoeff (int, default 2): coefficient applied to filterSize value for dilation phase of the morphological operation
 * - learningTime (int, default 300): number of frames for a pixel to be considered background
 * - lifetime (int, default 30): time (in frames) during which a blob is kept even if not detected
 * - keepOldBlobs (int[2], default [0]): parameters to not delete blobs which have disappeared. Parameters are: [minAgeToKeep] [maxTimeToKeep]
 * - processNoiseCov (int, default 1e-6): noise of the movement of the tracked object. Used for filtering detection.
 * - measurementNoiseCov (int, default 1e-4): noise of the measurement (capture + detection) of the tracked object. Used for filtering detection.
 * - maxDistanceForColorDiff (float, default 16): maximum distance beyond which the color of blobs is not considered for tracking.
 * - area (int[2], default 0 65535): minimum and maximum areas of the detected objects.
 *
 * OSC output:
 * - name: bgsubtractor
 * - values: Id(int) X(int) Y(int) Size(int) dX(float) dY(float) Age(int) lostDuration(int)
 *
 * \subsection actuator_clusterpcl_sec Clusters of point clouds (Actuator_ClusterPcl)
 *
 * This actuator outputs the number of distinct clusters it can find in the input point cloud
 *
 * Number of source(s) needed: 1 Source_3D
 *
 * Available parameters:
 * - minClusterSize (int, default 50): minimum number of points for a cluster to be kept
 * - maxClusterSize (int, default 25000): maximum number of points for a cluster to be kept
 * - clusterTolerance (float, default 0.03): maximum distance between two points to consider them as neighbours
 *
 * \subsection actuator_depthtouch_sec Adding touch interaction to surfaces using depth map (Actuator_DepthTouch)
 *
 * This actuator uses an input depth map (16 bits single channel image) to create a model of the targeted surface. After the model is created, it detects any object coming close to the surface, depending on the parameters. This means that objects passing in front of the surface are not detected unless their depth is close to the original surface.
 * 
 * Number of source(s) needed: 1 Source_2D

 * Available parameters:
 * - filterSize (int, default 3): size of the morphologicial filter used to filter noise.
 * - lifetime (int, default 30): time (in frames) during which a blob is kept even if not detected
 * - processNoiseCov (int, default 1e-6): noise of the movement of the tracked object. Used for filtering detection.
 * - measurementNoiseCov (int, default 1e-4): noise of the measurement (capture + detection) of the tracked object. Used for filtering detection.
 * - detectionDistance (float, default 100): maximum distance (in mm) for the detection to happen
 * - clickDistance (float, default 20): minimum distance (in mm) for a contact to be detected
 * - stddevCoeff (float, default 20): coefficient applied to the standard deviation map of the original depth map. If the resulting value is greater than detectionDistance, this value is used
 * - learningTime (int, defaut 60): number of frames to use to create the model
 * - learn (no parameter): send this message through OSC to restart the learning process
 *
 * OSC output:
 * - name: depthtouch
 * - values: Id(int) X(int) Y(int) dX(float) dY(float) contact(int)
 *
 * \subsection actuator_hog_sec Histogram of Oriented Gradients (Actuator_Hog)
 *
 * This actuator searches for objects corresponding to the model trained with blobtrainer.
 *
 * Number of source(s) needed: 1 Source_2D
 *
 * Available parameters:
 * - modelFilename (string): path to the file model
 * - maxTimePerFrame (int, default 1e5): maximum time (in us) to allow for detection at each frame
 * - maxThreads (int, default 4): maximum number of threads to use
 * - mergeDistance (int, default 64): distance below which detected blobs are merged
 * - filterSize (int, default 3): size of the morphological filter used to filter noise in the background subtraction pass
 * - roiSize (int[2], default 64 128): size (in pixels) of the detection window
 * - blockSize (int[2], default 2 2): size (in cells) where normalization is applied
 * - cellSize (int[2], default 16 16): size (in pixels) of a histogram of oriented gradients cell
 * - bins (int, default 9): number of orientations to consider
 * - margin (float, default 0.0): margin to the hyperplane to add to the detection (higher = less false positives and less hit rate)
 * - lifetime (int, default 30): time (in frames) during which a blob is kept even if not detected
 * - keepOldBlobs (int[2], default [0]): parameters to not delete blobs which have disappeared. Parameters are: [minAgeToKeep] [maxTimeToKeep]
 * - processNoiseCov (int, default 1e-6): noise of the movement of the tracked object. Used for filtering detection.
 * - measurementNoiseCov (int, default 1e-4): noise of the measurement (capture + detection) of the tracked object. Used for filtering detection.
 * - saveSamples (int, default 0): if set to 1, saves detected samples older than saveSamplesAge
 * - saveSamplesAge (int, default 120): see saveSamples
 *
 * OSC output:
 * - name: hog
 * - values: Id(int) X(int) Y(int) dX(int) dY(int) Age(int) lostDuration(int)
 * 
 * \subsection actuator_lightspot_sec Light spots (Actuator_LightSpots)
 * 
 * Detects the brightest spots in an input image, i.e. light from a torchlight, and outputs the resulting blobs' size, position and ID.
 * 
 * Number of source(s) needed: 1 Source_2D
 * 
 * Available parameters: 
 * - detectionLevel (int, default 2): minimum level (as a multiple of the standard deviation) to consider a point as a light spot.
 * - filterSize (int, default 3): size of the kernel for the morphological operations (to smooth the noise).
 * - processNoiseCov (int, default 1e-5): noise of the movement of the tracked object. Used for filtering detection.
 * - measurementNoiseCov (int, default 1e-5): noise of the measurement (capture + detection) of the tracked object. Used for filtering detection.
 * 
 * OSC output:
 * - name: lightSpots
 * - values: Id(int) X(int) Y(int) Size(int) dX(int) dY(int)
 * 
 * \subsection actuator_mean_outliers_sec Mean outliers (Actuator_MeanOutliers)
 * 
 * This actuator is a generalization of the Lightspots actuator, except that it does not only detect blobs brighter that the mean, but any blob which is far from the mean value of the current frame.
 * 
 * Number of source(s) needed: 1 Source_2D
 * 
 * Available parameters:
 * - detectionLevel (int, default 2): minimum level (as a multiple of the standard deviation) to consider a point as a light spot.
 * - filterSize (int, default 3): size of the kernel for the morphological operations (to smooth the noise).
 * - processNoiseCov (int, default 1e-6): noise of the movement of the tracked object. Used for filtering detection.
 * - measurementNoiseCov (int, default 1e-4): noise of the measurement (capture + detection) of the tracked object. Used for filtering detection.
 * 
 * OSC output:
 * - name: meanOutliers
 * - values: Id(int) X(int) Y(int) Size(int) dX(int) dY(int)
 *
 * \subsection actuator_objonaplane_sec Objects on a plane (Actuator_ObjOnAPlane)
 *
 * This actuator is specificaly designed to detect objects placed on a planar surface, by comparing images from two or more cameras. These cameras should be calibrated geometrically and colorimetrically to get optimal results. Note that this actuator is still in development. Cameras are calibrated by specifying to all of them the coordinates, in the image space, of a set of fixed points in the real space.
 *
 * Number of source(s) needed: 2+ Source_2D
 *
 * Available parameters:
 * - addSpace (float[4+]): adds the coordinates of the plane for the next input source (depending on which spaces have been given yet). All points are given in the input image space.
 * - spaces (float[*], no default): give all the coordinates of the plane for all the input sources. All set of coordinates should have the same number of points, each set being expressed in the corresponding source 2D space.
 * - clearSpace (no args): clears up all registered spaces.
 * - detectionLevel (float, default 10): sets the minimum distance in the color space to consider two colors as being different.
 * - processNoiseCov (int, default 1e-6): noise of the movement of the tracked object. Used for filtering detection.
 * - measurementNoiseCov (int, default 1e-4): noise of the measurement (capture + detection) of the tracked object. Used for filtering detection.
 * - filterSize (int, default 3): size of the kernel for the morphological operations (to smooth the noise).
 * - minBlobArea (int, default 32): minimum size of a blob to not be considered as noise.
 * - maxTrackedBlobs (int, default 16): maximum number of blobs to track
 *
 * \subsection actuator_stitch_sec Stitching (Actuator_Stitch)
 * 
 * This actuator stitches any number of 2D images into one, according to the input parameters.
 *
 * Number of source(s) needed: 1+ Source_2D
 *
 * Available parameters:
 * - cropInput (int[5], no default): crop parameters for the source given by the first value. Parameters are: [sourceIndex] [x] [y] [width] [height]
 * - transform (float[4], no default): translation and rotation parameters for the given source. Parameters are: [sourceIndex] [x] [y] [angle in degree]
 *
 **************
 * \section howto_xml_sec How to use Blobserver - Configuration through a XML file
 * 
 * The easiest way to configure Blobserver is to use a configuration file which is loaded at launch or sent to blobserver with blobcontroller, a small utility dedicated to this purpose. A few examples are shipped with the source code, and here follows the general form of a blobserver configuration file.
 *
 * \code
 * <Blobserver>
 *     <Flow>
 *         <Actuator>
 *             <Type>Actuator_Type</Type>
 *             <Param>
 *                 <Name>Param_1</Name>
 *                 <Value>value</Value>
 *             </Param>
 *             <Param>
 *                 <Name>Param_2</Name>
 *                 <Value>value</Value>
 *             </Param>
 *             <Param>
 *                 ...
 *             </Param>
 *         </Actuator>
 *         <Source>
 *             <Type>Source_Type</Type>
 *             <Subsource>nbr</Subsource>
 *             <Param>
 *                 <Name>Param_1</Name>
 *                 <Value>value</Value>
 *             </Param>
 *             <Param>
 *                 ...
 *             </Param>
 *         </Source>
 *         <Source>
 *             ...
 *         </Source>
 *         <Client>
 *             <Address>ip_address</Address>
 *             <Port>tcp_port</Port>
 *         </Client>
 *         <Server>
 *             <Address>ip_addresss</Address>
 *             <Port>tcp_port</Port>
 *         </Server>
 *     </Flow>
 * 
 *     <Flow>
 *         ...
 *     </Flow>
 * </Blobserver>
 * \endcode
 * 
 **************
 * \section howto_osc_sec How to use Blobserver - Configuration through OSC messages
 * 
 * Blobserver is entirely configurable through OSC messages and listens on port 9002 (by default). Here are the available message.
 * 
 * \subsection howto_osc_signin_sec /blobserver/signIn client port
 * 
 * Signs in to the blobserver this message is sent to. This is mandatory be able to send any other command.
 * 
 * Parameters:
 * - client (string): ip address or network name of the client to which OSC message will be sent.
 * - port (integer): TCP port the messages will be sent to.
 * 
 * \subsection howto_osc_signout_sec /blobserver/signOut client
 * 
 * Signs out from the blobserver this message is sent to. This finishes any flows linked to the specified client.
 * 
 * Parameter:
 * - client (string): ip address or network name of the client we want to sign out.
 * 
 * \subsection howto_osc_changeport_sec /blobserver/changePort client port
 * 
 * Enables to change the port the messages are sent to, for the specified client.
 * 
 * Parameter:
 * - client (string): ip address or network name for the client Blobserver sends messages to.
 * - port (integer): TCP port the messages will be sent to.
 * 
 * \subsection howto_osc_connect_sec /blobserver/connect client actuator source subsource source subsource ...
 * 
 * Ex.:
 * <pre>
 * /blobserver/connect localhost Actuator_MeanOutliers Source_OpenCV 300
 * </pre>
 * 
 * Sets up a new flow, and specifies the source(s) to use as its input.
 * 
 * Parameters:
 * - client (string): ip address or network name of the client to which OSC messages will be sent.
 * - actuator (string): name of the actuator we want to use
 * - source (string): name of one of the sources we want to use
 * - subsource (integer): index of the subsource we want to use (0 for default)
 * 
 * If all went well, returns the following message, with "index" being the index of the newly created flow:
 * <pre>/blobserver/connect "Connected" index</pre>
 * 
 * Else, returns an error message.
 * 
 * \subsection howto_osc_disconnect_sec /blobserver/disconnect client index
 * 
 * Disconnects the flow specified by "index" from the client "client". If the client was the last one to use this flow, it is destroyed in the process.
 * 
 * Parameter:
 * - client (string): ip address or network name for the client Blobserver sends messages to.
 * - index (integer): index of the flow to disconnect
 * 
 * If all went well, returns the following message:
 * <pre>/blobserver/disconnect "Disconnected"</pre>
 * 
 * \subsection howto_osc_disconnect2_sec /blobserver/disconnect client
 * 
 * Disconnects all the flows from the client "client". If the client was the last one to use this flow, it is destroyed in the process.
 * 
 * Parameter:
 * - client (string): ip address or network name for the client Blobserver sends messages to.
 * 
 * If all went well, returns the following message:
 * <pre>/blobserver/disconnect "Disconnected"</pre>
 * 
 * \subsection howto_osc_setparameter_sec /blobserver/setParameter client index "Actuator" parameter values
 * 
 * Sets the given values for the specified parameter of the actuator with the given index.
 * 
 * Parameter:
 * - client (string): ip address or network name for the client Blobserver sends messages to.
 * - index (integer): index of the flow
 * - parameter (string): parameter to modify
 * - values: values for the parameter (number and type depending of the parameter)
 * 
 * Returns an error message if things went wrong.
 * 
 * \subsection howto_osc_setparameter2_sec /blobserver/setParameter client index "Source" srcIndex parameter values
 * 
 * Sets the given values for the specified parameter of the source, given as an index in the specified flow (phew)
 * 
 * Parameters:
 * - client (string): ip address or network name for the client Blobserver sends messages to.
 * - index (integer): index of the flow
 * - srcIndex (integer): index of the source, as from the order sources where given in the /blobserver/connect message
 * - parameter (string): parameter to modifiy
 * - values: values for the source (number and type depending of the parameter)
 * 
 * Returns an error message if things went wrong.
 * 
 * \subsection howto_osc_setparameter3_sec /blobserver/setParameter client index "Start" ( or "Stop")
 * 
 * Starts or stops the given flow.
 * 
 * Parameter:
 * - client (string): ip address or network name for the client Blobserver sends messages to.
 * - index (integer): index of the flow
 * 
 * Returns an error message if things went wrong.
 * 
 * \subsection howto_osc_getparameter_sec /blobserver/getParameter client index "Actuator" parameter
 * 
 * Returns the value(s) for the given parameter of the given actuator:
 * 
 * Parameters:
 * - client (string): ip address or network name for the client Blobserver sends messages to.
 * - index (integer): index of the flow
 * - parameter (string): name of the parameter
 * 
 * \subsection howto_osc_getparameter2_sec /blobserver/getParameter client index "Sources" srcIndex parameter
 * 
 * Return the value(s) for the given parameter of the given source from the given actuator.
 * 
 * Parameters:
 * - client (string): ip address or network name for the client Blobserver sends messages to.
 * - index (integer): index of the flow
 * - srcIndex (integer): index of the source in the flow
 * - parameter (string): name of the parameter
 * 
 * \subsection howto_osc_actuators_sec /blobserver/actuators client
 * 
 * Returns a list of the available actuators, in the form of the following message:
 * <pre>/blobserver/actuators "Actuator_1" "Actuator_2" ...</pre>
 * 
 * Parameters:
 * - client (string): ip address or network name for the client Blobserver sends messages to.
 * 
 * \subsection howto_osc_sources_sec /blobserver/sources client
 * 
 * Returns a list of the available sources, in the form of the following message:
 * <pre>/blobserver/sources "Source_1" "Source_2" ...</pre>
 * 
 * Parameter:
 * - client (string): ip address or network name for the client the Blobserver sends messages to.
 * 
 * \subsection howto_osc_sources2_sec /blobserver/sources client sourceName
 * 
 * Returns a list of the available subsources for the given source, in the form of the following message:
 * <pre>/blobserver/sources subIndex_1 subIndex_2 ...</pre>
 * 
 * Parameters:
 * - client (string): ip address or network name for the client the Blobserver sends messages to.
 * - sourceName: name of the source as given by a call to "/blobserver/sources client"
 * 
 **************
 * \section howto_messages_sec How to use Blobserver - Detection related messages
 * 
 * During each iteration of the main loop, detected objects are sent through OSC to all clients which subscribed to each flow. Messages can vary depending on the actuator used, and you should report to the section dedicated to this actuator for further information. Anyway, these messages have the following general form:
 * <pre>/blobserver/[actuator_name] [values]</pre>
 */

#endif // MAINPAGE_H
