/*
 * Copyright (C) 2012 Emmanuel Durand
 *
 * This file is part of blobserver.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file
 * The main program.
 */

#include <iostream>
#include <limits>
#include <stdio.h>
#include <memory>
#include "glib.h"
#include "opencv2/opencv.hpp"
#include "lo/lo.h"
//#include "gst/gst.h"

#include "blob_2D.h"

static gboolean gVersion = FALSE;
static gboolean gHide = FALSE;
static gboolean gVerbose = FALSE;

static gint gCamNbr = 0;
static gint gWidth = 0;
static gint gHeight = 0;
static gdouble gFramerate = 0.0;

static gint gFilterSize = 3;
static gchar* gDetectionLevel = NULL;

static GString* gIpAddress = NULL;
static GString* gIpPort = NULL;
static gboolean gTcp = FALSE;

static gboolean gOutliers = FALSE;
static gboolean gLight = FALSE;

// TODO: add entries for width, height and framerate
static GOptionEntry gEntries[] =
{
    {"version", 0, 0, G_OPTION_ARG_NONE, &gVersion, "Shows version of this software", NULL},
    {"hide", 0, 0, G_OPTION_ARG_NONE, &gHide, "Hides the camera window", NULL},
    {"verbose", 'v', 0, G_OPTION_ARG_NONE, &gVerbose, "If set, outputs values to the std::out", NULL},
    {"cam", 'c', 0, G_OPTION_ARG_INT, &gCamNbr, "Selects which camera to use, as detected by OpenCV", NULL},
    {"width", 'w', 0, G_OPTION_ARG_INT, &gWidth, "Specifie the desired width for the camera capture", NULL},
    {"height", 'h', 0, G_OPTION_ARG_INT, &gHeight, "Specifie the desired height for the camera capture", NULL},
    {"fps", 0, 0, G_OPTION_ARG_DOUBLE, &gFramerate, "Specifie the desired framerate for the camera capture", NULL},
    {"filter", 'f', 0, G_OPTION_ARG_INT, &gFilterSize, "Specifies the size of the filtering kernel to use", NULL},
    {"level", 'l', 0, G_OPTION_ARG_STRING, &gDetectionLevel, "If applicable, specifies the detection level to use", NULL},
    {"ip", 'i', 0, G_OPTION_ARG_STRING_ARRAY, &gIpAddress, "Specifies the ip address to send messages to", NULL}, 
    {"port", 'p', 0, G_OPTION_ARG_STRING_ARRAY, &gIpPort, "Specifies the port to send messages to", NULL},
    {"tcp", 't', 0, G_OPTION_ARG_NONE, &gTcp, "Use TCP instead of UDP for message transmission", NULL},
    {"outliers", 0, 0, G_OPTION_ARG_NONE, &gOutliers, "Detects the outliers (luminance wise) and outputs their mean position", NULL},
    {"light", 0, 0, G_OPTION_ARG_NONE, &gLight, "Detects light zone, extract them as blobs and outputs their position and size", NULL},
    {NULL}
};

#define BLOB_NO_FILTER          0x0001
#define BLOB_FILTER_OUTLIERS    0x0002
#define BLOB_FILTER_LIGHT       0x0004
#define BLOB_FILTER_COLOR       0x0008

/*****************************/
// Definition of the app class
class App
{
    public:
        struct client
        {
            lo_address address;
            int filters;
        };
        
        ~App();

        static std::shared_ptr<App> getInstance();

        // Initialization, depending on arguments
        int init(int argc, char** argv);

        // Main loop
        int loop();

    private:
        // Attributes
        // Singleton
        static std::shared_ptr<App> mInstance;

        int mMaxTrackedBlobs;

        // liblo related
        std::vector<client> mClients;
        std::vector<lo_address> mOscAddresses;
        lo_server_thread mOscServer;

        // opencv related
        float mDetectionLevel;

        cv::VideoCapture mCamera;
        cv::Mat mCameraBuffer;

        // filter related
        std::map<int, int> mFiltersUsage;

        cv::SimpleBlobDetector* mLightBlobDetector; // OpenCV object which detects the blobs in an image
        std::vector<Blob2D> mLightBlobs; // Vector of detected and tracked blobs
        Blob2D mMeanBlob; // blob for the mean outlier detection

        // Methods
        App();

        // Arguments parser
        int parseArgs(int argc, char **argv);

        // OSC related, server side
        static void oscError(int num, const char* msg, const char* path);
        static int oscGenericHandler(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data);
        static int oscHandlerConnect(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data);
        static int oscHandlerDisconnect(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data);
        static int oscHandlerSetFilter(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data);
        
        // OSC related, client side
        void oscSend(const char* path, int filter, const char* types, lo_arg* argv);

        // Various filter and detection modes availables
        
        // Detects outliers based on the mean and std dev, and
        // outputs their mean position and total size
        cv::Mat detectMeanOutliers();

        // Detects light spots, and outputs each one of their position
        // and size
        cv::Mat detectLightSpots();

        // Detects color blobs, and outputs datas about them
        cv::Mat detectColorBlobs();
       
        // This function tracks the blobs through frames
        template <class T> void trackBlobs(std::vector<Blob::properties> &pProperties, std::vector<T> &pBlobs);

        // This function returns the configuration (element from x linked to element from y)
        // which gives the lowest sum, according the the pDistances. Returns a matrix with the
        // same dimensions as pDistances, filled with 0 and 255
        cv::Mat getLeastSumConfiguration(cv::Mat* pDistances);
        // This function is called by the previous one, and should not be called by itself
        // (it is part of a recursive algorithm)
        cv::Mat getLeastSumForLevel(cv::Mat pConfig, cv::Mat* pDistances, int pLevel, cv::Mat pAttributed, float &pSum, int pShift);
};

std::shared_ptr<App> App::mInstance(nullptr);

/*****************/
App::App()
    :mLightBlobDetector (NULL),
    mDetectionLevel (2.f),
    mMaxTrackedBlobs (8)
{
}


/*****************/
App::~App()
{
}

/*****************/
std::shared_ptr<App> App::getInstance()
{
    if(App::mInstance.get() == nullptr)
        App::mInstance.reset(new App);
    return App::mInstance;
}

/*****************/
int App::init(int argc, char** argv)
{
    // Parse arguments
    int ret = parseArgs(argc, argv);
    if(ret)
        return ret;

    // Initialize GStreamer
    //gst_init(&argc, &argv);

    // Initialize camera
    mCamera.open(gCamNbr);
    if(!mCamera.isOpened())
    {
        std::cout << "Error while opening camera number " << gCamNbr << ". Exiting." << std::endl;
        return 1;
    }

    if(gWidth > 0)
        mCamera.set(CV_CAP_PROP_FRAME_WIDTH, gWidth);
    if(gHeight > 0)
        mCamera.set(CV_CAP_PROP_FRAME_HEIGHT, gHeight);
    if(gFramerate > 0.0)
        mCamera.set(CV_CAP_PROP_FPS, gFramerate);

    // Initialize filters
    mFiltersUsage[BLOB_FILTER_OUTLIERS] = 0;
    mFiltersUsage[BLOB_FILTER_LIGHT] = 0;

    // Get a first frame to initialize the buffer
    mCamera.read(mCameraBuffer);

    // Initialize OSC
    int lNetProto;
    if(gTcp)
        lNetProto = LO_TCP;
    else
        lNetProto = LO_UDP;
    // Client
    if(gIpAddress != NULL)
    {
        std::cout << "IP specified: " << gIpAddress->str << std::endl;

        if(gIpPort != NULL)
        {
            std::cout << "Using port number " << gIpPort->str << std::endl;
        }
        else
        {
            gIpPort = g_string_new("9000");
            std::cout << "No port specified, using 9000" << std::endl;
        }

        client lClient;
        lClient.address = lo_address_new_with_proto(lNetProto, gIpAddress->str, gIpPort->str);
        lClient.filters = BLOB_FILTER_OUTLIERS | BLOB_FILTER_LIGHT;

        mFiltersUsage[BLOB_FILTER_OUTLIERS]++;
        mFiltersUsage[BLOB_FILTER_LIGHT]++;

        mClients.push_back(lClient);

        mOscAddresses.push_back(lClient.address);
    }

    // Server
    mOscServer = lo_server_thread_new_with_proto("9001", lNetProto, App::oscError);
    lo_server_thread_add_method(mOscServer, "/blobserver/connect", "si", App::oscHandlerConnect, NULL);
    lo_server_thread_add_method(mOscServer, "/blobserver/disconnect", "s", App::oscHandlerDisconnect, NULL);
    lo_server_thread_add_method(mOscServer, "/blobserver/filter", "ss", App::oscHandlerSetFilter, NULL);
    lo_server_thread_add_method(mOscServer, NULL, NULL, App::oscGenericHandler, NULL);
    lo_server_thread_start(mOscServer);

    // Initialize a few other things
    // Set mLightDetector to indeed detect light
    cv::SimpleBlobDetector::Params lParams;
    lParams.filterByColor = true;
    lParams.blobColor = 255;
    lParams.minCircularity = 0.1f;
    lParams.maxCircularity = 1.f;
    lParams.minInertiaRatio = 0.f;
    lParams.maxInertiaRatio = 1.f;
    lParams.minArea = 0.f;
    lParams.maxArea = 65535.f;
    mLightBlobDetector = new cv::SimpleBlobDetector(lParams);
    
    return 0;
}

/*****************/
int App::loop()
{
    int frameNbr = 0;

    bool lShowCamera = !gHide;
    int lSourceNumber = 0;

    bool loop = true;
    while(loop)
    {
        int lTotalBuffers = 1; // starting at 1, because there is at least the camera buffer
        std::vector<cv::Mat> lBuffers;
        std::vector<std::string> lBufferNames;

        // Frame capture
        mCamera.read( mCameraBuffer );
        // cv::Mat are not copied when not specified, so the bandwidth usage
        // of the following operation is minimal
        lBuffers.push_back(mCameraBuffer);
        lBufferNames.push_back(std::string("camera"));

        // If the frame seems valid
        if(mCameraBuffer.size[0] > 0 && mCameraBuffer.size[0] > 0)
        {
            // Camera capture
            if(lShowCamera)
                cv::imshow("blobserver", mCameraBuffer);

            // Evaluating a new frame
            lo_arg arg[1];
            arg[0].i = frameNbr;
            oscSend("/blobserver/startFrame", BLOB_NO_FILTER, "i", arg);

            // Informations extraction
            if(mFiltersUsage[BLOB_FILTER_OUTLIERS] > 0)
            {
                lBuffers.push_back(detectMeanOutliers());
                lBufferNames.push_back(std::string("mean outliers"));
                lTotalBuffers += 1;
            }

            if(mFiltersUsage[BLOB_FILTER_LIGHT] > 0)
            {
                lBuffers.push_back(detectLightSpots());
                lBufferNames.push_back(std::string("light blobs"));
                lTotalBuffers += 1;
            }

            if(lShowCamera)
            {
                cv::putText(lBuffers[lSourceNumber], lBufferNames[lSourceNumber].c_str(), cv::Point(10, 30),
                    cv::FONT_HERSHEY_COMPLEX, 1.0, cv::Scalar::all(255.0));
                cv::imshow("blobserver", lBuffers[lSourceNumber]);
            }

            // End of frame
            oscSend("/blobserver/endFrame", BLOB_NO_FILTER, "i", arg);
        }

        char lKey = cv::waitKey(5);
        if(lKey == 'q')
            loop = false;
        if(lKey == 'w')
        {
            lSourceNumber = (lSourceNumber+1)%lTotalBuffers;
            std::cout << "Buffer displayed: " << lBufferNames[lSourceNumber] << std::endl;
        }

        frameNbr++;
    }

    return 0;
}

/*****************/
void App::oscError(int num, const char* msg, const char* path)
{
    std::cout << "liblo server error " << num << " in path " << path << ": " << msg << std::endl;
}

/*****************/
int App::oscGenericHandler(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data)
{
    if(gVerbose)
    {
        std::cout << "Unhandled message received:" << std::endl;

        for(int i = 0; i < argc; ++i)
        {
            lo_arg_pp((lo_type)(types[i]), argv[i]);
        }

        std::cout << std::endl;
    }
}

/*****************/
int App::oscHandlerConnect(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data)
{
    char port[8];
    sprintf(port, "%i", argv[1]->i);

    lo_address address = lo_address_new(&argv[0]->s, port);

    int error = lo_address_errno(address);
    if(error != 0)
    {
        std::cout << "Wrong address received, error " << error << std::endl;
        return 1;
    }
    else
    {
        std::shared_ptr<App> theApp = App::getInstance();

        // Check if we don't have any connection from the same address
        bool isPresent = false;
        for(std::vector<client>::iterator it = theApp->mClients.begin(); it != theApp->mClients.end(); ++it)
        {
            if(strcmp(lo_address_get_url(it->address), lo_address_get_url(address)) == 0)
            {
                isPresent = true;
            }
        }

        if(!isPresent)
        {
            client lClient;
            lClient.address = address;
            lClient.filters = 0;
            theApp->mClients.push_back(lClient);

            lo_send(address, "/blobserver/connect", "s", "Connected");

            std::cout << "Connection accepted from address " << &argv[0]->s << std::endl;
        }
        
        return 0;
    }
}

/*****************/
int App::oscHandlerDisconnect(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data)
{
    std::shared_ptr<App> theApp = App::getInstance();
    lo_address address = lo_address_new(&argv[0]->s, "9000");

    for(std::vector<client>::iterator it = theApp->mClients.begin(); it != theApp->mClients.end(); ++it)
    {
        if(strcmp(lo_address_get_url(it->address), lo_address_get_url(address)) == 0)
        {
            lo_address_free(it->address);
            theApp->mClients.erase(it);
            std::cout << "Connection from address " << &argv[0]->s << " closed." << std::endl;

            lo_send(address, "/blobserver/disconnect", "s", "Disconnected");

            if(it->filters & BLOB_FILTER_OUTLIERS)
                theApp->mFiltersUsage[BLOB_FILTER_OUTLIERS]--;
            if(it->filters & BLOB_FILTER_LIGHT)
                theApp->mFiltersUsage[BLOB_FILTER_LIGHT]--;

            --it;
        }
    }

    lo_address_free(address);

    return 0;
}

/*****************/
int App::oscHandlerSetFilter(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data)
{
    std::shared_ptr<App> theApp = App::getInstance();
    lo_address address = lo_address_new(&argv[0]->s, "9000");

    int filter;
    if(strcmp(&argv[1]->s, "meanOutliers") == 0)
        filter = BLOB_FILTER_OUTLIERS;
    else if(strcmp(&argv[1]->s, "lightSpots") == 0)
        filter = BLOB_FILTER_LIGHT;
    else
        return 1;

    for(std::vector<client>::iterator it = theApp->mClients.begin(); it != theApp->mClients.end(); ++it)
    {
        if(strcmp(lo_address_get_url(it->address), lo_address_get_url(address)) == 0)
        {
            if((it->filters && filter) == true)
            {
                theApp->mFiltersUsage[filter]--;
                it->filters -= filter;

                std::string lPath = "/blobserver/";
                lPath += &argv[1]->s;
                lo_send(address, lPath.c_str(), "s", "Deactivated");
            }
            else
            {
                theApp->mFiltersUsage[filter]++;
                it->filters += filter;

                std::string lPath = "/blobserver/";
                lPath += &argv[1]->s;
                lo_send(address, lPath.c_str(), "s", "Activated");
            }
        }
    }

    lo_address_free(address);

    return 0;
}

/*****************/
void App::oscSend(const char* path, int filter, const char* types = NULL, lo_arg* argv = NULL)
{
    // Message creation
    if(types == NULL || argv == NULL)
        return;

    lo_message message = lo_message_new();

    char c;
    for(int i = 0, c = types[i]; c != '\0'; ++i, c = types[i]) 
    {
        switch(c)
        {
        case LO_INT32:
            lo_message_add_int32(message, argv[i].i);
            break;
        default:
            std::cout << "Unrecognized OSC type: '" << c << "'" << std::endl;
        }
    }

    for(std::vector<client>::iterator it = mClients.begin(); it != mClients.end(); ++it)
    {
        // If it is juste a normal message, not related to filters
        if(filter == BLOB_NO_FILTER)
            lo_send_message(it->address, path, message);
        else if((it->filters && filter) == true)
            lo_send_message(it->address, path, message);
    }
}

/*****************/
int App::parseArgs(int argc, char** argv)
{
    GError *error = NULL;
    GOptionContext* context;

    context = g_option_context_new("- blobserver, sends blobs through OSC");
    g_option_context_add_main_entries(context, gEntries, NULL);
    //g_option_context_add_group(context, gst_init_get_option_group());

    if(!g_option_context_parse(context, &argc, &argv, &error))
    {
        std::cout << "Error while parsing options: " << error->message << std::endl;
        return 1;
    }

    if(gDetectionLevel != NULL)
        mDetectionLevel = (float)g_ascii_strtod(gDetectionLevel, NULL);

    if(gVersion)
    {
        std::cout << PACKAGE_TARNAME << " " << PACKAGE_VERSION << std::endl;
        return 1;
    }

    mFiltersUsage[BLOB_FILTER_OUTLIERS] += gOutliers;
    mFiltersUsage[BLOB_FILTER_LIGHT] += gLight;

    return 0;
}

/*****************/
cv::Mat App::detectMeanOutliers()
{
    cv::Mat lMean, lStdDev;
    cv::Mat lOutlier, lEroded, lFiltered;

    // Eliminate the outliers : calculate the mean and std dev
    lOutlier = cv::Mat::zeros(mCameraBuffer.size[0], mCameraBuffer.size[1], CV_8U);
    lEroded = lOutlier.clone();
    lFiltered = lOutlier.clone();
    cv::cvtColor(mCameraBuffer, lOutlier, CV_RGB2GRAY);

    cv::meanStdDev(mCameraBuffer, lMean, lStdDev);
    cv::absdiff(lOutlier, lMean.at<double>(0), lOutlier);

    // Detect pixels far from the mean (> 2*stddev)
    cv::threshold(lOutlier, lOutlier, mDetectionLevel * lStdDev.at<double>(0), 255, cv::THRESH_BINARY);

    // Erode and dilate to suppress noise
    cv::erode(lOutlier, lEroded, cv::Mat(), cv::Point(-1, -1), gFilterSize);
    cv::dilate(lEroded, lFiltered, cv::Mat(), cv::Point(-1, -1), gFilterSize);

    // Calculate the barycenter of the outliers
    int lNumber = 0;
    int lX = 0, lY = 0;

    for(int x = 0; x < lFiltered.size[1]; ++x)
        for(int y = 0; y < lFiltered.size[0]; ++y)
        {
            if(lFiltered.at<uchar>(y, x) == 255)
            {
                lX += x;
                lY += y;
                lNumber++;
            }
        }

    bool isDetected = false;
    if(lNumber > 0)
    {
        isDetected = true;
        lX /= lNumber;
        lY /= lNumber;
    }
    else
    {
        lX = lFiltered.size[1] / 2;
        lY = lFiltered.size[0] / 2;
    }

    // Filtering
    static bool isInitialized = false;

    Blob::properties props;
    props.position.x = lX;
    props.position.y = lY;
    props.size = lNumber;

    if(!isInitialized)
    {
        mMeanBlob.init(props);
        isInitialized = true;
    }
    else
    {
        mMeanBlob.predict();
        mMeanBlob.setNewMeasures(props);
        props = mMeanBlob.getBlob();
    }

    lX = (int)(props.position.x);
    lY = (int)(props.position.y);
    lNumber = (int)(props.size);
    int lSpeedX = (int)(props.speed.x);
    int lSpeedY = (int)(props.speed.y);

    if(gVerbose)
    {
        std::cout << "--- Outliers detection:" << std::endl;
        std::cout << "x: " << lX << " - y: " << lY << " - size: " << lNumber << " - speedX: " << lSpeedX << " - speedY: " << lSpeedY << std::endl;
    }

    // Send the result
    lo_arg args[5];
    args[0].i = lX;
    args[1].i = lY;
    args[2].i = lNumber;
    args[3].i = lSpeedX;
    args[4].i = lSpeedY;
    oscSend("/blobserver/meanOutliers", BLOB_FILTER_OUTLIERS, "iiiii", args);

    return lFiltered;
}

/*****************/
cv::Mat App::detectLightSpots()
{
    cv::Mat lMean, lStdDev;
    cv::Mat lOutlier, lLight;
    cv::Mat lEroded;
    std::vector<cv::KeyPoint> lKeyPoints;

    // Eliminate the outliers : calculate the mean and std dev
    lOutlier = cv::Mat::zeros(mCameraBuffer.size[0], mCameraBuffer.size[1], CV_8U);
    lLight = lOutlier.clone();
    cv::cvtColor(mCameraBuffer, lOutlier, CV_RGB2GRAY);

    cv::meanStdDev(mCameraBuffer, lMean, lStdDev);
    cv::absdiff(lOutlier, lMean.at<double>(0), lOutlier);

    // Detect pixels which values are superior to the mean
    cv::threshold(lOutlier, lLight, lMean.at<double>(0), 255, cv::THRESH_BINARY);
    
    // Detect pixels far from the mean (> 2*stddev by default)
    cv::threshold(lOutlier, lOutlier, mDetectionLevel * lStdDev.at<double>(0), 255, cv::THRESH_BINARY);
    
    // Combinaison of both previous conditions
    cv::bitwise_and(lOutlier, lLight, lLight);

    // Erode and dilate to suppress noise
    cv::erode(lLight, lEroded, cv::Mat(), cv::Point(-1, -1), gFilterSize);
    cv::dilate(lEroded, lLight, cv::Mat(), cv::Point(-1, -1), gFilterSize);

    // Now we have to detect blobs
    mLightBlobDetector->detect(lLight, lKeyPoints);
    
    // We use Blob::properties, not cv::KeyPoints
    std::vector<Blob::properties> lProperties;
    for(int i = 0; i < std::min((int)(lKeyPoints.size()), mMaxTrackedBlobs); ++i)
    {
        Blob::properties properties;
        properties.position.x = lKeyPoints[i].pt.x;
        properties.position.y = lKeyPoints[i].pt.y;
        properties.size = lKeyPoints[i].size;
        properties.speed.x = 0.f;
        properties.speed.y = 0.f;

        lProperties.push_back(properties);
    }

    // We want to track them
    trackBlobs<Blob2D>(lProperties, mLightBlobs);

    // Make sure their covariances are correctly set
    std::vector<Blob2D>::iterator lIt = mLightBlobs.begin();
    for(; lIt != mLightBlobs.end(); ++lIt)
    {
        lIt->setParameter("processNoiseCov", 1e-5);
        lIt->setParameter("measurementNoiseCov", 1e-5);
    }

    if(gVerbose)
        std::cout << "--- Light blobs detection:" << std::endl;

    // And we send and print them
    for(int i = 0; i < mLightBlobs.size(); ++i)
    {
        int lX, lY, lSize, ldX, ldY;
        Blob::properties properties = mLightBlobs[i].getBlob();
        lX = (int)(properties.position.x);
        lY = (int)(properties.position.y);
        lSize = (int)(properties.size);
        ldX = (int)(properties.speed.x);
        ldY = (int)(properties.speed.y);

        if(gVerbose)
        {
            std::cout << &(mLightBlobs[i]) << " - Blob #" << mLightBlobs[i].getId() << " - x=" << lX << " - y=" << lY << " - size=" << lSize << " - dX = " << ldX << " - dY = " << ldY<< std::endl;
            // Print the blob number on the blob
            char lNbrStr[8];
            sprintf(lNbrStr, "%i", mLightBlobs[i].getId());
            cv::putText(lLight, lNbrStr, cv::Point(lX, lY), cv::FONT_HERSHEY_COMPLEX, 0.66, cv::Scalar(128.0, 128.0, 128.0, 128.0));
        }

        // Send the result through OSC
        lo_arg args[6];
        args[0].i = lX;
        args[1].i = lY;
        args[2].i = lSize;
        args[3].i = ldX;
        args[4].i = ldY;
        args[5].i = mLightBlobs[i].getId();
        oscSend("/blobserver/lightSpots", BLOB_FILTER_LIGHT, "iiiiii", args);
    }

    return lLight;
}

/*****************/
cv::Mat App::detectColorBlobs()
{
    cv::Mat lMatBlobs;

    return lMatBlobs;
}

/*****************/
template<class T> void App::trackBlobs(std::vector<Blob::properties> &pProperties, std::vector<T> &pBlobs)
{
    // First we update all the previous blobs we detected,
    // and keep their predicted new position
    for(int i = 0; i < pBlobs.size(); ++i)
        pBlobs[i].predict();
    
    // Then we compare all these prediction with real measures and
    // associate them together
    cv::Mat lConfiguration;
    if(pBlobs.size() != 0)
    {
        cv::Mat lTrackMat = cv::Mat::zeros(pProperties.size(), pBlobs.size(), CV_32F);

        // Compute the squared distance between all new blobs, and all tracked ones
        for(int i = 0; i < pProperties.size(); ++i)
        {
            for(int j = 0; j < pBlobs.size(); ++j)
            {
                Blob::properties properties = pProperties[i];
                lTrackMat.at<float>(i, j) = pBlobs[j].getDistanceFromPrediction(properties);
            }
        }

        // We associate each tracked blobs with the fittest blob, using a least square approach
        lConfiguration = getLeastSumConfiguration(&lTrackMat);
    }

    cv::Mat lAttributedKeypoints = cv::Mat::zeros(pProperties.size(), 1, CV_8U);
    typename std::vector<T>::iterator lBlob = pBlobs.begin();
    // We update the blobs which we were able to track
    for(int i = 0; i < lConfiguration.rows; ++i)
    {
        int lIndex = lConfiguration.at<uchar>(i);
        if(lIndex < 255)
        {
            lBlob->setNewMeasures(pProperties[lIndex]);
            lBlob++;
            lAttributedKeypoints.at<uchar>(lIndex) = 255;
        }
    }
    // We delete the blobs we couldn't track
    //for(lBlob = mLightBlobs.begin(); lBlob != mLightBlobs.end(); lBlob++)
    for(int i = 0; i < lConfiguration.rows; ++i)
    {
        int lIndex = lConfiguration.at<uchar>(i);
        if(lIndex == 255)
        {
            pBlobs.erase(pBlobs.begin()+i);
        }
    }
    // And we create new blobs for the new objects detected
    for(int i = 0; i < lAttributedKeypoints.rows; ++i)
    {
        int lIndex = lAttributedKeypoints.at<uchar>(i);
        if(lIndex == 0)
        {
            T lNewBlob;
            lNewBlob.init(pProperties[i]);
            pBlobs.push_back(lNewBlob);
        }
    }
}

/*****************/
cv::Mat App::getLeastSumConfiguration(cv::Mat* pDistances)
{
    float lMinSum = 0.f;
    cv::Mat lConfiguration = cv::Mat::ones(mLightBlobs.size(), 1, CV_8U)*255;
    cv::Mat lAttributed = cv::Mat::zeros(pDistances->rows, 1, CV_8U);

    lConfiguration = getLeastSumForLevel(lConfiguration, pDistances, 0, lAttributed, lMinSum, 0);

    return lConfiguration;
}

/*****************/
cv::Mat App::getLeastSumForLevel(cv::Mat pConfig, cv::Mat* pDistances, int pLevel, cv::Mat pAttributed, float &pSum, int pShift)
{
    // If we lost one or more blobs, we will need to shift the remaining blobs to test all
    // the possible combinations
    int lLevelRemaining = pConfig.rows - (pLevel + 1);
    int lMaxShift = std::max(0, std::min(pConfig.rows - pDistances->rows - pShift, lLevelRemaining));

    float lMinSum = std::numeric_limits<float>::max();
    float lCurrentSum;
    cv::Mat lAttributed;
    cv::Mat lConfig, lCurrentConfig;

    // We try without shifting anything
    for(int i = 0; i < pAttributed.rows + lMaxShift; ++i)
    {

        // If we do not shift
        if(i < pAttributed.rows)
        {
            if(pAttributed.at<uchar>(i) == 0)
            {    
                lAttributed = pAttributed.clone();
                lCurrentConfig = pConfig.clone();

                lAttributed.at<uchar>(i) = 255;
                lCurrentSum = pSum + pDistances->at<float>(i, pLevel);
                lCurrentConfig.at<uchar>(pLevel) = i;

                if(lLevelRemaining > 0)
                    lCurrentConfig = getLeastSumForLevel(lCurrentConfig, pDistances, pLevel + 1, lAttributed, lCurrentSum, pShift);

                if(lCurrentSum < lMinSum)
                {
                    lMinSum = lCurrentSum;
                    lConfig = lCurrentConfig;
                }
            }
        }
        // if we shift, don't attribute this keypoint to any blob
        else if(i >= pAttributed.rows)
        {
            lAttributed = pAttributed.clone();
            lCurrentConfig = pConfig.clone();
            lCurrentSum = pSum;

            if(lLevelRemaining > 0)
                lCurrentConfig = getLeastSumForLevel(lCurrentConfig, pDistances, pLevel + 1, lAttributed, lCurrentSum, pShift + 1);
            
            if(lCurrentSum < lMinSum)
            {
                lMinSum = lCurrentSum;
                lConfig = lCurrentConfig;
            }
        }
    }

    if(lMinSum < std::numeric_limits<float>::max())
    {
        pSum = lMinSum;
        return lConfig;
    }
    else
    {
        return pConfig;
    }
}

/*****************/
int main(int argc, char** argv)
{
    std::shared_ptr<App> theApp = App::getInstance();
    int ret;

    ret = theApp->init(argc, argv);
    if(ret != 0)
        return ret;

    ret = theApp->loop();
    return ret;
}
