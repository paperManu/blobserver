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
 * Blobserver is an OSC-based server aimed at detecting entities (objects / people / light / ...), in any compatible image flow. Its structure is so that it should be relatively easy to add new detectors as well as new light sources. As of yet, configuration and communication with Blobserver is done entirely through OSC messaging. Some kind of configuration file will be added soon to simplify the setup and usability, especially for permanent installations.
 * 
 * Blobserver is built around the concept of flow. A flow is the association of a detector and as many sources as needed for it to work correctly. At each frame, all the flows are evaluated, and the various objects detected are sent through OSC to the corresponding clients.
 * 
 **************
 * \section sources_sec List of compatible sources
 * 
 * The following sources are compatible with Blobserver:
 * - any source compatible with OpenCV
 * - shmdata sources
 *
 * Some parameters are available for all kind of sources, none if these transformations are activated by default:
 * - mask (string): file path to the image file to use as a mask
 * - autoExposure (int[6]): parameters for auto exposure, measured in a specified area. Parameters are: [x] [y] [width] [height] [target] [margin].
 * - scale (float, default 1.0): apply scaling on the image
 * - rotation (float): apply rotation on the image, in degrees
 * - noiseFiltering (int, default 0): set to 1 to activate noise filtering
 * - distortion (int[3]): distortion correction (see http://wiki.panotools.org/Lens_correction_model). Parameters are: [a] [b] [c]
 * - vignetting (int[3]): correction of the vignetting (see http://lensfun.berlios.de/lens-calibration/lens-vignetting.html). Parameters are: [k1] [k2] [k3]
 * - iccInputProfile (string): file path to an ICC profile (for color correction)
 * - hdri (int[3]): activates the creation of a HDR image. Parameters are: [startExposure] [stepSize] [nbrSteps].
 * - save (int[2] string): activates the automatic save of grabs. Parameters are: [activation] [period] [filename] 
 * 
 * \subsection source_opencv_sec OpenCV sources (Source_OpenCV)
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
 * \subsection source_shmdata_sec shmdata sources (Source_Shmdata)
 * 
 * Available parameters:
 * - location (string): file path to the shmdata
 * - cameraNumber (int): index of the shmdata, this can be used to access multiple times to the same shmdata
 * 
 **************
 * \section detectors_sec List of detectors
 * 
 * \subsection detector_lightspot_sec Lighspots (Detector_LightSpots)
 * 
 * Detects the brightest spots in an input image, i.e. light from a torchlight, and outputs the resulting blobs' size, position and ID.
 * 
 * Number of source(s) needed: 1
 * 
 * Available parameters: 
 * - detectionLevel (int, default 2): minimum level (as a multiple of the standard deviation) to consider a point as a light spot.
 * - filterSize (int, default 3): Size of the kernel for the morphological operations (to smooth the noise).
 * - processNoiseCov (int, default 1e-5): Noise of the movement of the tracked object. Used for filtering detection.
 * - measurementNoiseCov (int, default 1e-5): Noise of the measurement (capture + detection) of the tracked object. Used for filtering detection.
 * 
 * OSC output:
 * - name: lightSpots
 * - values: X(int) Y(int) Size(int) dX(int) dY(int) Id(int)
 * 
 * \subsection detector_mean_outliers_sec Mean Outliers (Detector_MeanOutliers)
 * 
 * This detector is a generalization of the Lightspots detector, except that it does not only detect blobs brighter that the mean, but any blob which is far from the mean value of the current frame.
 * 
 * Number of source(s) needed: 1
 * 
 * Available parameters:
 * - detectionLevel (int, default 2): minimum level (as a multiple of the standard deviation) to consider a point as a light spot.
 * - filterSize (int, default 3): Size of the kernel for the morphological operations (to smooth the noise).
 * - processNoiseCov (int, default 1e-6): Noise of the movement of the tracked object. Used for filtering detection.
 * - measurementNoiseCov (int, default 1e-4): Noise of the measurement (capture + detection) of the tracked object. Used for filtering detection.
 * 
 * OSC output:
 * - name: meanOutliers
 * - values: X(int) Y(int) Size(int) dX(int) dY(int)
 *
 * \subsection detector_hog_sec Histogram of Oriented Gradients (Detector_Hog)
 *
 * This detector searches for objects corresponding to the model trained with blobtrainer.
 *
 * Number of source(s) needed: 1
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
 * - lifetime (int, default 30): time (in frames) during which a blob is kepts even if not detected
 * - processNoiseCov (int, default 1e-6): noise of the movement of the tracked object. Used for filtering detection.
 * - measurementNoiseCov (int, default 1e-4): noise of the measurement (capture + detection) of the tracked object. Used for filtering detection.
 * - saveSamples (int, default 0): if set to 1, saves detected samples older than saveSamplesAge
 * - saveSamplesAge (int, default 120): see saveSamples
 * 
 **************
 * \section howto_xml_sec How to use Blobserver - Configuration through a XML file
 * 
 * The easiest way to configure Blobserver is to use a configuration file which is loaded at launch or sent to blobserver with blobcontroller, a small utility dedicated to this purpose. A few examples are shipped with the source code, and here follows the general form of a blobserver configuration file.
 *
 * \code
 * <Blobserver>
 *     <Flow>
 *         <Detector>
 *             <Type>Detector_Type</Type>
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
 *         </Detector>
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
 * \subsection howto_osc_connect_sec /blobserver/connect client detector source subsource source subsource ...
 * 
 * Ex.:
 * <pre>
 * /blobserver/connect localhost Detector_MeanOutliers Source_OpenCV 300
 * </pre>
 * 
 * Sets up a new flow, and specifies the source(s) to use as its input.
 * 
 * Parameters:
 * - client (string): ip address or network name of the client to which OSC messages will be sent.
 * - detector (string): name of the detector we want to use
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
 * \subsection howto_osc_setparameter_sec /blobserver/setParameter client index "Detector" parameter values
 * 
 * Sets the given values for the specified parameter of the detector with the given index.
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
 * \subsection howto_osc_getparameter_sec /blobserver/getParameter client index "Detector" parameter
 * 
 * Returns the value(s) for the given parameter of the given detector:
 * 
 * Parameters:
 * - client (string): ip address or network name for the client Blobserver sends messages to.
 * - index (integer): index of the flow
 * - parameter (string): name of the parameter
 * 
 * \subsection howto_osc_getparameter2_sec /blobserver/getParameter client index "Sources" srcIndex parameter
 * 
 * Return the value(s) for the given parameter of the given source from the given detector.
 * 
 * Parameters:
 * - client (string): ip address or network name for the client Blobserver sends messages to.
 * - index (integer): index of the flow
 * - srcIndex (integer): index of the source in the flow
 * - parameter (string): name of the parameter
 * 
 * \subsection howto_osc_detectors_sec /blobserver/detectors client
 * 
 * Returns a list of the available detectors, in the form of the following message:
 * <pre>/blobserver/detectors "Detector_1" "Detector_2" ...</pre>
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
 * During each iteration of the main loop, detected objects are sent through OSC to all clients which subscribed to each flow. Messages can vary depending on the detector used, and you should report to the section dedicated to this detector for further information. Anyway, these messages have the following general form:
 * <pre>/blobserver/[detector_name] [values]</pre>
 */

#endif // MAINPAGE_H
