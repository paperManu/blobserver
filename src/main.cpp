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
#include "source_opencv.h"
#include "detector_meanOutliers.h"
#include "detector_lightSpots.h"
#include "atom/osc.h"
#include "abstract-factory.h" // TODO: check license issues

static gboolean gVersion = FALSE;
static gboolean gHide = FALSE;
static gboolean gVerbose = FALSE;

static gint gCamNbr = 0;
static gint gWidth = 0;
static gint gHeight = 0;
static gdouble gFramerate = 0.0;

static gint gFilterSize = 3;
static gchar* gDetectionLevel = NULL;
static gchar* gMaskFilename = NULL;

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
    {"mask", 'm', 0, G_OPTION_ARG_STRING, &gMaskFilename, "Specifies a mask which will be applied to all detectors", NULL},
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
        // Struct to contain information about the OSC client
        struct Client
        {
            lo_address address;
            int filters;
        };

        // Struct to contain a complete flow, from capture to client
        struct Flow
        {
            std::vector<std::shared_ptr<Source>> sources;
            std::shared_ptr<Detector> detector;
            lo_address client;
            unsigned int id;
        };
        
        ~App();

        static std::shared_ptr<App> getInstance();

        // Initialization, depending on arguments
        int init(int argc, char** argv);

        // Main loop
        int loop();

    private:
        /***********/
        // Attributes
        // Singleton
        static std::shared_ptr<App> mInstance;
        int mMaxTrackedBlobs;

        // Factories
        factory::AbstractFactory<Detector, std::string, std::string, int> mDetectorFactory;
        factory::AbstractFactory<Source, std::string, std::string, int> mSourceFactory;

        // liblo related
        std::vector<Client> mClients;
        std::vector<lo_address> mOscAddresses;
        lo_server_thread mOscServer;

        // detection related
        float mDetectionLevel;
        std::vector<std::shared_ptr<Source>> mSources;
        std::vector<Flow> mFlows;

        std::shared_ptr<Source> mSource;
        cv::Mat mCameraBuffer;
        cv::Mat mMask;

        static unsigned int mCurrentId;

        // filter related
        std::map<int, int> mFiltersUsage;

        Detector_MeanOutliers mMeanOutliersDetector;
        Detector_LightSpots mLightSpotsDetector;

        /********/
        // Methods
        App();

        // Arguments parser
        int parseArgs(int argc, char **argv);

        // Factory registering
        void registerClasses();

        unsigned int getValidId() {return ++mCurrentId;}

        // OSC related, server side
        static void oscError(int num, const char* msg, const char* path);
        static int oscGenericHandler(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data);
        static int oscHandlerConnect(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data);
        static int oscHandlerDisconnect(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data);
        static int oscHandlerSetFilter(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data);
        static int oscHandlerSetParameter(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data);
        
        // OSC related, client side
        void oscSend(const char* path, int filter, const char* types, lo_arg* argv);
};

std::shared_ptr<App> App::mInstance(nullptr);
unsigned int App::mCurrentId = 0;

/*****************/
App::App()
    :mDetectionLevel (2.f),
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
    // Register source and detector classes
    registerClasses();

    // Parse arguments
    int ret = parseArgs(argc, argv);
    if(ret)
        return ret;

    // Initialize GStreamer
    //gst_init(&argc, &argv);

    // Initialize camera
    mSource.reset(new Source_OpenCV);
    if(!mSource->connect())
    {
        std::cout << "Error while opening camera number " << gCamNbr << ". Exiting." << std::endl;
        return 1;
    }

    mSources.push_back(mSource);
   
    if(gWidth > 0)
        mSource->setParameter("width", gWidth);
    if(gHeight > 0)
        mSource->setParameter("height", gHeight);
    if(gFramerate > 0.0)
        mSource->setParameter("framerate", gFramerate);

    // Initialize filters
    mFiltersUsage[BLOB_FILTER_OUTLIERS] = 0;
    mFiltersUsage[BLOB_FILTER_LIGHT] = 0;

    // Get a first frame to initialize the buffer
    mSource->grabFrame();
    mCameraBuffer = mSource->retrieveFrame();

    // Initialize OSC
    int lNetProto;
    if(gTcp)
        lNetProto = LO_TCP;
    else
        lNetProto = LO_UDP;

    // Client
    if (gIpAddress != NULL)
    {
        // If an IP is specified, we create flows using all the specified detectors
        std::cout << "IP specified: " << gIpAddress->str << std::endl;

        if (gIpPort != NULL)
        {
            std::cout << "Using port number " << gIpPort->str << std::endl;
        }
        else
        {
            gIpPort = g_string_new("9000");
            std::cout << "No port specified, using 9000" << std::endl;
        }

        Flow lFlow;
        lFlow.client = lo_address_new_with_proto(lNetProto, gIpAddress->str, gIpPort->str);
        lFlow.sources.push_back(mSource);

        if (gOutliers)
        {
            lFlow.detector.reset(new Detector_MeanOutliers);
            if (mMask.total() > 0)
                lFlow.detector->setMask(mMask);
            lFlow.id = getValidId();
            mFlows.push_back(lFlow);
        }
        if (gLight)
        {
            lFlow.detector.reset(new Detector_LightSpots);
            if (mMask.total() > 0)
                lFlow.detector->setMask(mMask);
            lFlow.id = getValidId();
            mFlows.push_back(lFlow);
        }

        Client lClient;
        lClient.address = lo_address_new_with_proto(lNetProto, gIpAddress->str, gIpPort->str);
        lClient.filters = BLOB_FILTER_OUTLIERS | BLOB_FILTER_LIGHT;

        mFiltersUsage[BLOB_FILTER_OUTLIERS]++;
        mFiltersUsage[BLOB_FILTER_LIGHT]++;

        if(mMask.total() > 0)
            mMeanOutliersDetector.setMask(mMask);

        mClients.push_back(lClient);

        mOscAddresses.push_back(lClient.address);
    }

    // Server
    mOscServer = lo_server_thread_new_with_proto("9001", lNetProto, App::oscError);
    lo_server_thread_add_method(mOscServer, "/blobserver/connect", "si", App::oscHandlerConnect, NULL);
    lo_server_thread_add_method(mOscServer, "/blobserver/disconnect", "s", App::oscHandlerDisconnect, NULL);
    lo_server_thread_add_method(mOscServer, "/blobserver/filter", "ss", App::oscHandlerSetFilter, NULL);
    lo_server_thread_add_method(mOscServer, "/blobserver/parameter", NULL, App::oscHandlerSetParameter, NULL);
    lo_server_thread_add_method(mOscServer, NULL, NULL, App::oscGenericHandler, NULL);
    lo_server_thread_start(mOscServer);

    return 0;
}

/*****************/
int App::parseArgs(int argc, char** argv)
{
    GError *error = NULL;
    GOptionContext* context;

    context = g_option_context_new("- blobserver, sends blobs through OSC");
    g_option_context_add_main_entries(context, gEntries, NULL);
    //g_option_context_add_group(context, gst_init_get_option_group());

    if (!g_option_context_parse(context, &argc, &argv, &error))
    {
        std::cout << "Error while parsing options: " << error->message << std::endl;
        return 1;
    }

    if (gDetectionLevel != NULL)
        mDetectionLevel = (float)g_ascii_strtod(gDetectionLevel, NULL);

    if (gMaskFilename != NULL)
    {
        mMask = cv::imread(gMaskFilename, CV_LOAD_IMAGE_GRAYSCALE);
    }

    if (gVersion)
    {
        std::cout << PACKAGE_TARNAME << " " << PACKAGE_VERSION << std::endl;
        return 1;
    }

    mFiltersUsage[BLOB_FILTER_OUTLIERS] += gOutliers;
    mFiltersUsage[BLOB_FILTER_LIGHT] += gLight;

    return 0;
}

/*****************/
void App::registerClasses()
{
    // Register detectors
    mDetectorFactory.register_class<Detector_LightSpots>(Detector_LightSpots::getClassName(),
        Detector_LightSpots::getDocumentation());
    mDetectorFactory.register_class<Detector_MeanOutliers>(Detector_MeanOutliers::getClassName(),
        Detector_MeanOutliers::getDocumentation());

    // Register sources
    mSourceFactory.register_class<Source_OpenCV>(Source_OpenCV::getClassName(),
        Source_OpenCV::getDocumentation());
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
        mSource->grabFrame();
        mCameraBuffer = mSource->retrieveFrame();
        // cv::Mat are not copied when not specified, so the bandwidth usage
        // of the following operation is minimal
        lBuffers.push_back(mCameraBuffer); // TODO: remove all mention of mCameraBuffer
        lBufferNames.push_back(std::string("camera"));

        // If the frame seems valid
        if (mCameraBuffer.total() > 0)
        {
            // Camera capture
            if (lShowCamera)
                cv::imshow("blobserver", mCameraBuffer);

            // Evaluating a new frame
            lo_arg arg[1];
            arg[0].i = frameNbr;
            oscSend("/blobserver/startFrame", BLOB_NO_FILTER, "i", arg);

            // Grab from all the sources
            {
                std::vector<std::shared_ptr<Source>>::const_iterator iter;
                for (iter = mSources.begin(); iter != mSources.end(); ++iter)
                {
                    std::shared_ptr<Source> source = (*iter);
                    source->grabFrame(); // TODO: add the grabbed frame to the lBuffers vector
                }
            }

            // Go through the flows
            {
                std::vector<Flow>::iterator iter;
                for (iter = mFlows.begin(); iter != mFlows.end(); ++iter)
                {
                    Flow flow = (*iter);
                    // Retrieve the frames from all sources in this flow
                    std::vector<cv::Mat> frames;
                    for (int i = 0; i < flow.sources.size(); ++i)
                        frames.push_back(flow.sources[i]->retrieveFrame());

                    // Apply the detector on these (TODO: this, currently) frames
                    atom::Message message = flow.detector->detect(frames[0]);
                    lBuffers.push_back(flow.detector->getOutput());
                    lBufferNames.push_back(flow.detector->getName());
                    lTotalBuffers += 1;

                    // Send messages
                    int nbr = atom::toInt(message[0]);
                    int size = atom::toInt(message[1]);
                    for (int i = 0; i < nbr; ++i)
                    {
                        atom::Message msg;
                        for (int j = 0; j < size; ++j)
                            msg.push_back(message[i * size + 2 + j]);
                        
                        lo_message oscMsg = lo_message_new();
                        atom::message_build_to_lo_message(msg, oscMsg);
                        lo_send_message(flow.client, flow.detector->getOscPath().c_str(), oscMsg);
                    }
                }
            }

            if (lShowCamera)
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

        usleep(1000);
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
    std::shared_ptr<App> theApp = App::getInstance();

    char port[8];
    sprintf(port, "%i", argv[1]->i);

    atom::Message message;
    atom::message_build_from_lo_args(message, types, argv, argc);

    if(message.size() < 2)
        return 1; // TODO: add error messages for all the following returns
    
    lo_address address = lo_address_new(atom::toString(message[0]).c_str(), port);

    int error = lo_address_errno(address);
    if (error != 0)
    {
        std::cout << "Wrong address received, error " << error << std::endl;
        return 1;
    }
    else
    {
        std::shared_ptr<App> theApp = App::getInstance();

        // Check arguments
        // First argument is the chosen detector, next ones are sources
        std::string detectorName;
        try
        {
            detectorName = atom::toString(message[1]);
        }
        catch (atom::BadTypeTagError typeError)
        {
            return 1;
        }

        // Create the specified detector
        std::shared_ptr<Detector> detector;
        if (theApp->mDetectorFactory.key_exists(detectorName))
            detector = theApp->mDetectorFactory.create(detectorName);
        else
            return 1;

        // Check how many cameras we need for it
        unsigned int sourceNbr = detector->getSourceNbr();
        
        // Allocate all the sources
        std::vector<std::shared_ptr<Source>> sources;
        atom::Message::const_iterator iter;
        for (iter = message.begin()+3; iter != message.end(); ++iter)
        {
            if (iter+1 == message.end())
                return 1;

            std::string sourceName;
            int sourceIndex;
            try
            {
                sourceName = atom::toString(*iter);
                sourceIndex = atom::toInt(*(iter+1));
            }
            catch (atom::BadTypeTagError typeError)
            {
                return 1;
            }

            // Check if this source is not already connected
            bool alreadyConnected = false;
            std::vector<std::shared_ptr<Source>>::const_iterator iterSource;
            for (iterSource = theApp->mSources.begin(); iterSource != theApp->mSources.end(); ++iterSource)
            {
                if (iterSource->get()->getName() == sourceName && iterSource->get()->getSubsourceNbr() == (unsigned int)sourceIndex)
                {
                    sources.push_back(*iterSource);
                    alreadyConnected = true;
                }
            }

            if (!alreadyConnected)
            {
                std::shared_ptr<Source> source;
                if (theApp->mSourceFactory.key_exists(sourceName))
                    source = theApp->mSourceFactory.create(sourceName, sourceIndex);
                else
                    return 1;

                sources.push_back(source);
            }
        }

        // If enough sources have been specified
        if (sources.size() >= sourceNbr)
        {
            // We can create the flow!
            Flow flow;
            
            flow.detector = detector;
            flow.client = address;
            flow.id = theApp->getValidId();

            std::vector<std::shared_ptr<Source>>::const_iterator source;
            for (source = sources.begin(); source != sources.end(); ++source)
            {
                flow.sources.push_back(*source);

                // Add the sources to the mSources vector
                // (if they are not already there
                bool isInSources = false;
                std::vector<std::shared_ptr<Source>>::const_iterator iter;
                for (iter = theApp->mSources.begin(); iter != theApp->mSources.end(); ++iter)
                {
                    if (iter->get()->getName() == source->get()->getName() && iter->get()->getSubsourceNbr() == source->get()->getSubsourceNbr())
                        isInSources = true;
                }
                if (!isInSources)
                    theApp->mSources.push_back(*source);
            }
            theApp->mFlows.push_back(flow);

            // Tell the client that he is connected, and give him the flow id
            lo_send(address, "/blobserver/connect", "si", "Connected", (int)flow.id);
        }

        // TODO: delete what comes after this
        // Check if we don't have any connection from the same address
        bool isPresent = false;
        for(std::vector<Client>::iterator it = theApp->mClients.begin(); it != theApp->mClients.end(); ++it)
        {
            if(strcmp(lo_address_get_url(it->address), lo_address_get_url(address)) == 0)
            {
                isPresent = true;
            }
        }

        if(!isPresent)
        {
            Client lClient;
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

    for(std::vector<Client>::iterator it = theApp->mClients.begin(); it != theApp->mClients.end(); ++it)
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

    for(std::vector<Client>::iterator it = theApp->mClients.begin(); it != theApp->mClients.end(); ++it)
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
int App::oscHandlerSetParameter(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data)
{
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

    for(std::vector<Client>::iterator it = mClients.begin(); it != mClients.end(); ++it)
    {
        // If it is juste a normal message, not related to filters
        if(filter == BLOB_NO_FILTER)
            lo_send_message(it->address, path, message);
        else if((it->filters && filter) == true)
            lo_send_message(it->address, path, message);
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
