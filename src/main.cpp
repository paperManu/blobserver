/*
 * Copyright (C) 2012 Emmanuel Durand
 *
 * This file is part of blobserver.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * switcher is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with switcher.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file
 * The main program.
 */

#include <iostream>
#include <limits>
#include <stdio.h>
#include <memory>
#include <mutex>
#include <thread>
#include <glib.h>
#include <opencv2/opencv.hpp>
#include <lo/lo.h>
#include <atom/osc.h>
//#include "gst/gst.h"

#include "base_objects.h"
#include "blob_2D.h"
#include "configurator.h"
#include "source_opencv.h"
#include "detector_meanOutliers.h"
#include "detector_lightSpots.h"
#include "detector_objOnAPlane.h"
#include "abstract-factory.h"

static gboolean gVersion = FALSE;
static gboolean gHide = FALSE;
static gboolean gVerbose = FALSE;

static gchar* gConfigFile = NULL;

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

// TODO: check wether all these options still work
// TODO: or better, delete options that are availables in config file
static GOptionEntry gEntries[] =
{
    {"version", 0, 0, G_OPTION_ARG_NONE, &gVersion, "Shows version of this software", NULL},
    {"config", 'C', 0, G_OPTION_ARG_STRING, &gConfigFile, "Specify a configuration file to load at startup", NULL},
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

/*****************************/
// Definition of the app class
class App
{
    public:
        // Struct to contain a complete flow, from capture to client
        struct Flow
        {
            std::vector<std::shared_ptr<Source>> sources;
            std::shared_ptr<Detector> detector;
            std::shared_ptr<OscClient> client;
            unsigned int id;
            bool run;
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

        // Factories
        factory::AbstractFactory<Detector, std::string, std::string, int> mDetectorFactory;
        factory::AbstractFactory<Source, std::string, std::string, int> mSourceFactory;

        // liblo related
        lo_server_thread mOscServer;

        // detection related
        std::vector<std::shared_ptr<Source>> mSources;
        std::vector<Flow> mFlows;
        // A mutex to prevent unexpected changes in flows
        std::mutex mFlowMutex;
        std::mutex mSourceMutex;

        // Threads
        std::shared_ptr<std::thread> mSourcesThread;

        std::shared_ptr<Source> mSource;
        cv::Mat mMask; // TODO: set mask through a parameter
        // TODO: send mask through gstreamer! or from any source!

        static unsigned int mCurrentId;

        /********/
        // Methods
        App();

        // Arguments parser
        int parseArgs(int argc, char **argv);

        // Factory registering
        void registerClasses();

        // Creates a new and unique ID for a flow
        unsigned int getValidId() {return ++mCurrentId;}

        // Sources update function, use in a thread
        static void updateSources();

        // OSC related, server side
        static void oscError(int num, const char* msg, const char* path);
        static int oscGenericHandler(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data);
        static int oscHandlerConnect(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data);
        static int oscHandlerDisconnect(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data);
        static int oscHandlerSetParameter(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data);
        static int oscHandlerGetParameter(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data);
        static int oscHandlerGetDetectors(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data);
        static int oscHandlerGetSources(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data);
};

std::shared_ptr<App> App::mInstance(nullptr);
unsigned int App::mCurrentId = 0;

/*****************/
App::App()
{
    mCurrentId = 0;
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

    // Initialize OSC
    int lNetProto;
    if (gTcp)
        lNetProto = LO_TCP;
    else
        lNetProto = LO_UDP;

    // Create the thread which will grab from all sources
    mSourcesThread.reset(new std::thread(updateSources));

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

        // Initialize camera
        mSource.reset(new Source_OpenCV);
        if (!mSource->connect())
        {
            std::cout << "Error while opening camera number " << gCamNbr << ". Exiting." << std::endl;
            return 1;
        }
    
        mSources.push_back(mSource);
       
        if (gWidth > 0)
        {
            atom::Message msg;
            msg.push_back(atom::StringValue::create("width"));
            msg.push_back(atom::FloatValue::create((double)gWidth));
            mSource->setParameter(msg);
        }
        if (gHeight > 0)
        {
            atom::Message msg;
            msg.push_back(atom::StringValue::create("width"));
            msg.push_back(atom::FloatValue::create((double)gHeight));
            mSource->setParameter(msg);
        }
        if (gFramerate > 0.0)
        {
            atom::Message msg;
            msg.push_back(atom::StringValue::create("framerate"));
            msg.push_back(atom::FloatValue::create((double)gFramerate));
            mSource->setParameter(msg);
        }
    
        // Create the flows
        Flow lFlow;
        lFlow.client.reset(new OscClient(lo_address_new_with_proto(lNetProto, gIpAddress->str, gIpPort->str)));
        lFlow.sources.push_back(mSource);
        lFlow.run = true;

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
    }

    // Server
    mOscServer = lo_server_thread_new_with_proto("9002", lNetProto, App::oscError);
    lo_server_thread_add_method(mOscServer, "/blobserver/connect", NULL, App::oscHandlerConnect, NULL);
    lo_server_thread_add_method(mOscServer, "/blobserver/disconnect", NULL, App::oscHandlerDisconnect, NULL);
    lo_server_thread_add_method(mOscServer, "/blobserver/setParameter", NULL, App::oscHandlerSetParameter, NULL);
    lo_server_thread_add_method(mOscServer, "/blobserver/getParameter", NULL, App::oscHandlerGetParameter, NULL);
    lo_server_thread_add_method(mOscServer, "/blobserver/detectors", NULL, App::oscHandlerGetDetectors, NULL);
    lo_server_thread_add_method(mOscServer, "/blobserver/sources", NULL, App::oscHandlerGetSources, NULL);
    lo_server_thread_add_method(mOscServer, NULL, NULL, App::oscGenericHandler, NULL);
    lo_server_thread_start(mOscServer);

    if (gConfigFile != NULL)
    {
        Configurator configurator;
        configurator.loadXML((char*)gConfigFile);
    }

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

    if (gMaskFilename != NULL)
    {
        mMask = cv::imread(gMaskFilename, CV_LOAD_IMAGE_GRAYSCALE);
    }

    if (gVersion)
    {
        std::cout << PACKAGE_TARNAME << " " << PACKAGE_VERSION << std::endl;
        return 1;
    }

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
    mDetectorFactory.register_class<Detector_ObjOnAPlane>(Detector_ObjOnAPlane::getClassName(),
        Detector_ObjOnAPlane::getDocumentation());

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
        std::vector<cv::Mat> lBuffers;
        std::vector<std::string> lBufferNames;

        // First buffer is a black screen. No special reason, except we need
        // a first buffer
        lBuffers.push_back(cv::Mat::zeros(480, 640, CV_8UC3));
        lBufferNames.push_back(std::string("This is Blobserver"));

        // Retrive the capture from all the sources
        {
            std::lock_guard<std::mutex> lock(mSourceMutex);

            // First we grab, then we retrieve all frames
            // This way, sync between frames is better
            std::for_each (mSources.begin(), mSources.end(), [&] (std::shared_ptr<Source> source)
            {
                cv::Mat frame = source->retrieveCorrectedFrame();

                lBuffers.push_back(frame);

                atom::Message msg;
                msg.push_back(atom::StringValue::create("id"));
                msg = source->getParameter(msg);
                int id = atom::toInt(msg[1]);
                char name[16];
                sprintf(name, "%i", id);
                lBufferNames.push_back(source->getName() + std::string(" ") + std::string(name));
            } );
        }

        // Go through the flows
        {
            std::lock_guard<std::mutex> lock(mFlowMutex);

            std::for_each (mFlows.begin(), mFlows.end(), [&] (Flow flow)
            {
                if (flow.run == false)
                    return;

                // Retrieve the frames from all sources in this flow
                // Their is no risk for sources to disappear here, so no
                // need for a mutex (they are freed earlier)
                std::vector<cv::Mat> frames;
                for (int i = 0; i < flow.sources.size(); ++i)
                    frames.push_back(flow.sources[i]->retrieveCorrectedFrame());

                // Apply the detector on these frames
                atom::Message message = flow.detector->detect(frames);

                // Lock everything to add result to buffers, and send OSC messages
                lBuffers.push_back(flow.detector->getOutput());
                lBufferNames.push_back(flow.detector->getName());

                // Send messages
                // Beginning of the frame
                lo_send(flow.client->get(), "/blobserver/startFrame", "ii", frameNbr, flow.id);

                int nbr = atom::toInt(message[0]);
                int size = atom::toInt(message[1]);
                for (int i = 0; i < nbr; ++i)
                {
                    atom::Message msg;
                    for (int j = 0; j < size; ++j)
                        msg.push_back(message[i * size + 2 + j]);
                    
                    lo_message oscMsg = lo_message_new();
                    atom::message_build_to_lo_message(msg, oscMsg);
                    lo_send_message(flow.client->get(), flow.detector->getOscPath().c_str(), oscMsg);
                }

                // End of the frame
                lo_send(flow.client->get(), "/blobserver/endFrame", "ii", frameNbr, flow.id);
            } );
        }

        if (lShowCamera)
        {
            // Check if the current source number is still available
            if (lSourceNumber >= lBuffers.size())
                lSourceNumber = 0;

            cv::putText(lBuffers[lSourceNumber], lBufferNames[lSourceNumber].c_str(), cv::Point(10, 30),
                cv::FONT_HERSHEY_COMPLEX, 1.0, cv::Scalar::all(255.0));
            cv::imshow("blobserver", lBuffers[lSourceNumber]);
        }

        char lKey = cv::waitKey(5);
        if(lKey == 27) // Escape
            loop = false;
        if(lKey == 'w')
        {
            lSourceNumber = (lSourceNumber+1)%lBuffers.size();
            std::cout << "Buffer displayed: " << lBufferNames[lSourceNumber] << std::endl;
        }

        frameNbr++;

        timespec req, rem;
        req.tv_sec = 0;
        req.tv_nsec = req.tv_sec*1000000L;
        nanosleep(&req, &rem);
    }

    return 0;
}

/*****************/
void App::updateSources()
{
    std::shared_ptr<App> theApp = App::getInstance();

    bool run = true;

    while(run)
    {
        {
            std::lock_guard<std::mutex> lock(theApp->mSourceMutex);
            
            std::vector<std::shared_ptr<Source>>::iterator iter;
            // First we grab, then we retrieve all frames
            // This way, sync between frames is better
            for (iter = theApp->mSources.begin(); iter != theApp->mSources.end(); ++iter)
            {
                std::shared_ptr<Source> source = (*iter);
                source->grabFrame();
            
                // We also check if this source is still used
                if (source.use_count() == 2) // 2, because this ptr and the one in the vector
                {
                    std::cout << "Source " << source->getName() << " is no longer used. Disconnecting." << std::endl;
                    theApp->mSources.erase(iter);
                    --iter;
                }
            }
        }

        usleep(1000);
    }
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

    return 1;
}

/*****************/
int App::oscHandlerConnect(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data)
{
    std::shared_ptr<App> theApp = App::getInstance();

    // Messge must be : ip / port / detector / source0 / subsource0 / source1 / ...
    atom::Message message;
    atom::message_build_from_lo_args(message, types, argv, argc);


    char port[8];
    try
    {
        int portNbr = atom::toInt(message[1]);
        sprintf(port, "%i", portNbr);
    }
    catch (atom::BadTypeTagError exception)
    {
        return 1;
    }
    
    //lo_address address = lo_address_new(atom::toString(message[0]).c_str(), port);
    std::shared_ptr<OscClient> address(new OscClient(lo_address_new(atom::toString(message[0]).c_str(), port)));

    int error = lo_address_errno(address->get());
    if (error != 0)
    {
        std::cout << "Wrong address received, error " << error << std::endl;
        return 1;
    }
    else
    {
        if (message.size() < 5)
        {
            lo_send(address->get(), "/blobserver/connect", "s", "Too few arguments");
            return 1; 
        }

        // Check arguments
        // First argument is the chosen detector, next ones are sources
        std::string detectorName;
        try
        {
            detectorName = atom::toString(message[2]);
        }
        catch (atom::BadTypeTagError typeError)
        {
            lo_send(address->get(), "/blobserver/connect", "s", "Expected a detector type at position 2");
            return 1;
        }

        // Create the specified detector
        std::shared_ptr<Detector> detector;
        if (theApp->mDetectorFactory.key_exists(detectorName))
            detector = theApp->mDetectorFactory.create(detectorName);
        else
        {
            lo_send(address->get(), "/blobserver/connect", "s", "Detector type not recognized");
            return 1;
        }

        // Check how many cameras we need for it
        unsigned int sourceNbr = detector->getSourceNbr();
        
        // Allocate all the sources
        std::vector<std::shared_ptr<Source>> sources;
        atom::Message::const_iterator iter;
        for (iter = message.begin()+3; iter != message.end(); iter+=2)
        {
            if (iter+1 == message.end())
            {
                lo_send(address->get(), "/blobserver/connect", "s", "Missing sub-source number");
                return 1;
            }

            std::string sourceName;
            int sourceIndex;
            try
            {
                sourceName = atom::toString(*iter);
                sourceIndex = atom::toInt(*(iter+1));
            }
            catch (atom::BadTypeTagError typeError)
            {
                lo_send(address->get(), "/blobserver/connect", "s", "Expected integer as a sub-source number");
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
                {
                    std::string error = "Unable to create source ";
                    error += sourceName;
                    lo_send(address->get(), "/blobserver/connect", "s", error.c_str());
                    return 1;
                }
                
                if (!source->connect())
                {
                    std::string error = "Unable to connect to source ";
                    error += sourceName;
                    lo_send(address->get(), "/blobserver/connect", "s", error.c_str());
                    return 1;
                }

                sources.push_back(source);
            }
        }

        // If enough sources have been specified
        if (sources.size() >= sourceNbr)
        {
            std::lock_guard<std::mutex> lock(theApp->mFlowMutex);
            std::lock_guard<std::mutex> lockToo(theApp->mSourceMutex);

            // We can create the flow!
            Flow flow;
            
            flow.detector = detector;
            flow.client = address;
            flow.id = theApp->getValidId();
            flow.run = false;

            std::vector<std::shared_ptr<Source>>::const_iterator source;
            for (source = sources.begin(); source != sources.end(); ++source)
            {
                flow.sources.push_back(*source);

                // Add the sources to the mSources vector
                // (if they are not already there)
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
            lo_send(address->get(), "/blobserver/connect", "si", "Connected", (int)flow.id);
        }
        else
        {
            lo_send(address->get(), "/blobserver/connect", "s", "The specified detector needs more sources");
            return 1;
        }
        
        return 0;
    }
}

/*****************/
int App::oscHandlerDisconnect(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data)
{
    std::shared_ptr<App> theApp = App::getInstance();
    
    atom::Message message;
    atom::message_build_from_lo_args(message, types, argv, argc);

    std::string addressStr = atom::toString(message[0]);
    std::shared_ptr<OscClient> address(new OscClient(lo_address_new(addressStr.c_str(), "9000")));
    int error = lo_address_errno(address->get());
    if (error != 0)
    {
        std::cout << "Wrong address received, error " << error << std::endl;
        return 1;
    }
    
    if (message.size() != 1 && message.size() != 2)
    {
        lo_send(address->get(), "/blobserver/disconnect", "s", "Wrong number of arguments");
        return 1;
    }
    
    bool all = false;
    int detectorId;
    if (message.size() == 1)
        all = true;
    else
        detectorId = atom::toInt(message[1]);

    // Delete flows related to this address, according to the parameter
    std::lock_guard<std::mutex> lock(theApp->mFlowMutex);
    std::vector<Flow>::iterator flow;
    for (flow = theApp->mFlows.begin(); flow != theApp->mFlows.end();)
    {
        if (std::string(lo_address_get_url(flow->client->get())) == std::string(lo_address_get_url(address->get())))
        {
            if (all == true || detectorId == flow->id)
            {
                theApp->mFlows.erase(flow);
                std::cout << "Connection from address " << addressStr << " closed." << std::endl;
                lo_send(address->get(), "/blobserver/disconnect", "s", "Disconnected");
            }
            else
            {
                flow++;
            }
        }
        else
        {
            flow++;
        }
    }

    return 0;
}

/*****************/
int App::oscHandlerSetParameter(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data)
{
    std::shared_ptr<App> theApp = App::getInstance();    

    atom::Message message;
    atom::message_build_from_lo_args(message, types, argv, argc);
        
    std::string addressStr = atom::toString(message[0]);
    std::shared_ptr<OscClient> address(new OscClient(lo_address_new(addressStr.c_str(), "9000")));

    // Message must contain ip address, flow id, target (detector or src), src number if applicable, parameter and value
    // or just ip address, flow id, and start/stop
    if (message.size() < 3)
    {
        lo_send(address->get(), "/blobserver/setParameter", "s", "Wrong number of arguments");
        return 1;
    }
    
    int error = lo_address_errno(address->get());
    if (error != 0)
    {
        std::cout << "Wrong address received, error " << error << std::endl;
        return 1;
    }

    // Find the flow
    int result = 0;

    unsigned int flowId = (unsigned int)(atom::toInt(message[1]));
    std::vector<Flow>::iterator flow;
    for (flow = theApp->mFlows.begin(); flow != theApp->mFlows.end(); ++flow)
    {
        if (flow->id == flowId)
        {
            std::lock_guard<std::mutex> lock(theApp->mFlowMutex);

            // If the parameter is for the detector
            if (atom::toString(message[2]) == "Detector")
            {
                if (message.size() < 5)
                {
                    lo_send(address->get(), "/blobserver/setParameter", "s", "Wrong number of arguments");
                    result = 1;
                }
                else
                {
                    atom::Message msg;
                    for (int i = 3; i < message.size(); ++i)
                        msg.push_back(message[i]);
                    flow->detector->setParameter(msg);
                }
            }
            // If the parameter is for one of the sources
            else if (atom::toString(message[2]) == "Source")
            {
                if (message.size() < 6)
                {
                    lo_send(address->get(), "/blobserver/setParameter", "s", "Wrong number of arguments");
                    result = 1;
                }
                else
                {
                    int srcNbr = atom::toInt(message[3]);
                    if (srcNbr >= flow->sources.size())
                    {
                        lo_send(address->get(), "/blobserver/setParameter", "s", "Wrong source index");
                        result = 1;
                    }
                    else
                    {
                        atom::Message msg;
                        for (int i = 4; i < message.size(); ++i)
                            msg.push_back(message[i]);
                        flow->sources[srcNbr]->setParameter(msg);
                    }
                }
            }
            else if (atom::toString(message[2]) == "Start")
            {
                flow->run = true;
            }
            else if (atom::toString(message[2]) == "Stop")
            {
                flow->run = false;
            }
        }
    }

    return result;
}

/*****************/
int App::oscHandlerGetParameter(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data)
{
    std::shared_ptr<App> theApp = App::getInstance();

    atom::Message message;
    atom::message_build_from_lo_args(message, types, argv, argc);

    std::shared_ptr<OscClient> address;
    try
    {
        std::string addressStr = atom::toString(message[0]);
        address.reset(new OscClient(lo_address_new(addressStr.c_str(), "9000")));
    }
    catch (...)
    {
        return 1;
    }

    if (message.size() < 4)
    {
        lo_send(address->get(), "/blobserver/getParameter", "s", "Wrong number of arguments");
        return 1;
    }

    unsigned int flowId;
    std::string entity;

    try
    {
        flowId = (unsigned int)(atom::toInt(message[1]));
        entity = atom::toString(message[2]);
    }
    catch (atom::BadTypeTagError exception)
    {
        return 1;
    }

    // Go through the flows
    int result = 0;
    std::for_each (theApp->mFlows.begin(), theApp->mFlows.end(), [&] (Flow flow)
    {
        if (flow.id == flowId)
        {
            std::lock_guard<std::mutex> lock(theApp->mFlowMutex);

            // If the parameter is for the detector
            if (entity == "Detector")
            {
                atom::Message msg;
                msg.push_back(message[3]);
                msg = flow.detector->getParameter(msg);

                lo_message oscMsg = lo_message_new();
                atom::message_build_to_lo_message(msg, oscMsg);
                lo_send_message(address->get(), "/blobserver/getParameter", oscMsg);
            }
            // If the parameter is for the sources
            else if (entity == "Sources")
            {
                if (message.size() < 5)
                {
                    lo_send(address->get(), "/blobserver/getParameter", "s", "Wrong number of arguments");
                    result = 1;
                }
                else
                {
                    int srcNbr;
                    try
                    {
                        srcNbr = atom::toInt(message[3]);
                    }
                    catch (...)
                    {
                        return 1;
                    }

                    if (srcNbr >= flow.sources.size())
                    {
                        result = 1;
                    }
                    else
                    {
                        atom::Message msg;
                        msg.push_back(message[4]);
                        msg = flow.sources[srcNbr]->getParameter(msg);

                        lo_message oscMsg = lo_message_new();
                        atom::message_build_to_lo_message(msg, oscMsg);
                        lo_send_message(address->get(), "/blobserver/getParameter", oscMsg);
                    }
                }
            }
        }
    } );

    return result;
}

/*****************/
int App::oscHandlerGetDetectors(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data)
{
    std::shared_ptr<App> theApp = App::getInstance();

    atom::Message message;
    atom::message_build_from_lo_args(message, types, argv, argc);

    if (message.size() < 1)
        return 1;

    std::shared_ptr<OscClient> address;
    try
    {
        std::string addressStr = atom::toString(message[0]);
        address.reset(new OscClient(lo_address_new(addressStr.c_str(), "9000")));
    }
    catch (atom::BadTypeTagError exception)
    {
        return 1;
    }

    // Get all the available detectors
    std::vector<std::string> keys = theApp->mDetectorFactory.get_keys();

    atom::Message outMessage;
    std::for_each (keys.begin(), keys.end(), [&] (std::string key)
    {
        std::cout << key << std::endl;
        outMessage.push_back(atom::StringValue::create(key.c_str()));
    } );

    lo_message oscMsg = lo_message_new();
    atom::message_build_to_lo_message(outMessage, oscMsg);

    lo_send_message(address->get(), "/blobserver/detectors", oscMsg);
}

/*****************/
int App::oscHandlerGetSources(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data)
{
    std::shared_ptr<App> theApp = App::getInstance();

    atom::Message message;
    atom::message_build_from_lo_args(message, types, argv, argc);

    std::string addressStr;
    try
    {
        addressStr = atom::toString(message[0]);
    }
    catch (...)
    {
        return 1;
    }
    std::shared_ptr<OscClient> address(new OscClient(lo_address_new(addressStr.c_str(), "9000")));
    
    // If we have another parameter, it means we want to get availables subsources
    atom::Message outMessage;
    if (message.size() > 1)
    {
        std::string sourceName;
        try
        {
            sourceName = atom::toString(message[1]);
        }
        catch (...)
        {
            return 1;
        }

        // We try to create the named source
        std::shared_ptr<Source> source;
        if (theApp->mSourceFactory.key_exists(sourceName))
            source = theApp->mSourceFactory.create(sourceName, -1);
        else
            return 1;

        // Ask the source for all the available subsources
        outMessage = source->getSubsources();
    }
    else
    {
        // Get all the available sources
        std::vector<std::string> keys = theApp->mSourceFactory.get_keys();

        std::for_each (keys.begin(), keys.end(), [&] (std::string key)
        {
            outMessage.push_back(atom::StringValue::create(key.c_str()));
        } );
    }

    lo_message oscMsg = lo_message_new();
    atom::message_build_to_lo_message(outMessage, oscMsg);
    lo_send_message(address->get(), "/blobserver/sources", oscMsg);
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
