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
 * @blobserver.cpp
 * The main program from the blobserver suite.
 */

#include <iostream>
#include <limits>
#include <stdio.h>
#include <memory>
#include <mutex>
#include <thread>
#include <chrono>

#include <glib.h>
#include <glib/gstdio.h>
#include <opencv2/opencv.hpp>
#include <lo/lo.h>
#include <atom/osc.h>

#include "config.h"
#include "abstract-factory.h"
#include "base_objects.h"
#include "blob_2D.h"
#include "configurator.h"
#include "threadPool.h"

#include "source_opencv.h"
#ifdef HAVE_SHMDATA
#include "source_shmdata.h"
#endif
#include "detector_lightSpots.h"
#include "detector_meanOutliers.h"
#include "detector_nop.h"
#include "detector_objOnAPlane.h"

using namespace std;

static gboolean gVersion = FALSE;
static gboolean gHide = FALSE;
static gboolean gVerbose = FALSE;

static gchar* gConfigFile = NULL;
static gchar* gMaskFilename = NULL;
static gboolean gTcp = FALSE;

static gboolean gBench = FALSE;

static GOptionEntry gEntries[] =
{
    {"version", 'v', 0, G_OPTION_ARG_NONE, &gVersion, "Shows version of this software", NULL},
    {"config", 'C', 0, G_OPTION_ARG_STRING, &gConfigFile, "Specify a configuration file to load at startup", NULL},
    {"hide", 'H', 0, G_OPTION_ARG_NONE, &gHide, "Hides the camera window", NULL},
    {"verbose", 'V', 0, G_OPTION_ARG_NONE, &gVerbose, "If set, outputs values to the std::out", NULL},
    {"mask", 'm', 0, G_OPTION_ARG_STRING, &gMaskFilename, "Specifies a mask which will be applied to all detectors", NULL},
    {"tcp", 't', 0, G_OPTION_ARG_NONE, &gTcp, "Use TCP instead of UDP for message transmission", NULL},
    {"bench", 'B', 0, G_OPTION_ARG_NONE, &gBench, "Enables printing timings of main loop, for debug purpose", NULL},
    {NULL}
};

/*****************************/
// Definition of the app class
class App
{
    public:
        ~App();

        static shared_ptr<App> getInstance();

        // Initialization, depending on arguments
        int init(int argc, char** argv);

        // Main loop
        int loop();

    private:
        /***********/
        // Attributes
        // Singleton
        static shared_ptr<App> mInstance;

        bool mRun;

        // Factories
        factory::AbstractFactory<Detector, string, string, int> mDetectorFactory;
        factory::AbstractFactory<Source, string, string, int> mSourceFactory;

        // liblo related
        lo_server_thread mOscServer;

        // detection related
        vector<shared_ptr<Source>> mSources;
        vector<Flow> mFlows;
        // A mutex to prevent unexpected changes in flows
        mutex mFlowMutex;
        mutex mSourceMutex;
        // A thread pool for detectors
        shared_ptr<ThreadPool> mThreadPool;

        // Threads
        shared_ptr<thread> mSourcesThread;

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

shared_ptr<App> App::mInstance(nullptr);
unsigned int App::mCurrentId = 0;

/*****************/
App::App()
{
    mCurrentId = 0;
    mThreadPool.reset(new ThreadPool(5));
}


/*****************/
App::~App()
{
}

/*****************/
shared_ptr<App> App::getInstance()
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


    cout << "Cleaning up shared memory in /tmp..." << endl;
    GDir* directory;
    GError* error;
    directory = g_dir_open((const gchar*)"/tmp", 0, &error);
    const gchar* filename;
    while ((filename = g_dir_read_name(directory)) != NULL)
    {
        if (strstr((const char*)filename, (const char*)"blobserver") != NULL)
        {
            char buffer[128];
            sprintf(buffer, "/tmp/%s", filename);
            cout << "Removing file " << buffer << endl;
            g_remove((const gchar*)buffer);
        }
    }

    // Server
    mOscServer = lo_server_thread_new_with_proto("9002", lNetProto, App::oscError);
    if (mOscServer != NULL)
    {
        lo_server_thread_add_method(mOscServer, "/blobserver/connect", NULL, App::oscHandlerConnect, NULL);
        lo_server_thread_add_method(mOscServer, "/blobserver/disconnect", NULL, App::oscHandlerDisconnect, NULL);
        lo_server_thread_add_method(mOscServer, "/blobserver/setParameter", NULL, App::oscHandlerSetParameter, NULL);
        lo_server_thread_add_method(mOscServer, "/blobserver/getParameter", NULL, App::oscHandlerGetParameter, NULL);
        lo_server_thread_add_method(mOscServer, "/blobserver/detectors", NULL, App::oscHandlerGetDetectors, NULL);
        lo_server_thread_add_method(mOscServer, "/blobserver/sources", NULL, App::oscHandlerGetSources, NULL);
        lo_server_thread_add_method(mOscServer, NULL, NULL, App::oscGenericHandler, NULL);
        lo_server_thread_start(mOscServer);
    }
    else
    {
        cout << "TCP port not available for the Osc server to launch - Exiting" << endl;
        exit(1);
    }

    // Configuration file needs to be loaded in a thread
    {
        Configurator configurator;
        configurator.loadXML((char*)gConfigFile);
    }

    // Create the thread which will grab from all sources
    // This must be run AFTER loading the configuration, as some params
    // can't be changed after the first grab for some sources
    mRun = true;
    mSourcesThread.reset(new thread(updateSources));

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
        cout << "Error while parsing options: " << error->message << endl;
        return 1;
    }

    if (gMaskFilename != NULL)
    {
        mMask = cv::imread(gMaskFilename, CV_LOAD_IMAGE_GRAYSCALE);
    }

    if (gVersion)
    {
        cout << PACKAGE_TARNAME << " " << PACKAGE_VERSION << endl;
        return 1;
    }

    return 0;
}

/*****************/
void App::registerClasses()
{
    // Register detectors
    mDetectorFactory.register_class<Detector_Nop>(Detector_Nop::getClassName(),
        Detector_Nop::getDocumentation());
    mDetectorFactory.register_class<Detector_LightSpots>(Detector_LightSpots::getClassName(),
        Detector_LightSpots::getDocumentation());
    mDetectorFactory.register_class<Detector_MeanOutliers>(Detector_MeanOutliers::getClassName(),
        Detector_MeanOutliers::getDocumentation());
    mDetectorFactory.register_class<Detector_ObjOnAPlane>(Detector_ObjOnAPlane::getClassName(),
        Detector_ObjOnAPlane::getDocumentation());

    // Register sources
    mSourceFactory.register_class<Source_OpenCV>(Source_OpenCV::getClassName(),
        Source_OpenCV::getDocumentation());
    mSourceFactory.register_class<Source_Shmdata>(Source_Shmdata::getClassName(),
        Source_Shmdata::getDocumentation());
}

/*************/
void timeSince(unsigned long long timestamp, std::string stage)
{
    auto now = chrono::high_resolution_clock::now();
    unsigned long long currentTime = chrono::duration_cast<chrono::microseconds>(now.time_since_epoch()).count();
    std::cout << stage << " - " << ((long long)currentTime - (long long)timestamp)/1000 << "ms" << std::endl;
}

/*****************/
int App::loop()
{
    int frameNbr = 0;

    bool lShowCamera = !gHide;
    int lSourceNumber = 0;

    unsigned long long usecPeriod = 33333;

    mutex lMutex;

    while(mRun)
    {
        unsigned long long chronoStart;
        chronoStart = chrono::duration_cast<chrono::microseconds>(chrono::high_resolution_clock::now().time_since_epoch()).count();

        vector<cv::Mat> lBuffers;
        vector<string> lBufferNames;

        // First buffer is a black screen. No special reason, except we need
        // a first buffer
        lBuffers.push_back(cv::Mat::zeros(480, 640, CV_8UC3));
        lBufferNames.push_back(string("This is Blobserver"));

        // Retrieve the capture from all the sources
        {
            //lock_guard<mutex> lock(mSourceMutex);

            // First we grab, then we retrieve all frames
            // This way, sync between frames is better
            for_each (mSources.begin(), mSources.end(), [&] (shared_ptr<Source> source)
            {
                cv::Mat frame = source->retrieveCorrectedFrame();

                lBuffers.push_back(frame);

                atom::Message msg;
                msg.push_back(atom::StringValue::create("id"));
                msg = source->getParameter(msg);
                int id = atom::toInt(msg[1]);
                char name[16];
                sprintf(name, "%i", id);
                lBufferNames.push_back(source->getName() + string(" ") + string(name));
            } );
        }

        if (gBench)
            timeSince(chronoStart, string("1 - Retrieve corrected frames"));

        // Go through the flows
        {
            lock_guard<mutex> lock(mFlowMutex);
            vector<shared_ptr<thread> > threads;
            threads.resize(mFlows.size());

            // Update all sources for all flows
            //for_each (mFlows.begin(), mFlows.end(), [&] (Flow flow)
            for (int index = 0; index < mFlows.size(); ++index)
            {
                Flow* flow = &mFlows[index];
                if (flow->run == false)
                    continue;

                // Apply the detector on these frames
                mThreadPool->enqueue([=, &lMutex] ()
                {
                    // Retrieve the frames from all sources in this flow
                    // There is no risk for sources to disappear here, so no
                    // need for a mutex (they are freed earlier)
                    vector<cv::Mat> frames;
                    {
                        for (int i = 0; i < flow->sources.size(); ++i)
                        {
                            lock_guard<mutex> lock(lMutex);
                            frames.push_back(flow->sources[i]->retrieveCorrectedFrame());
                        }
                    }

                    flow->detector->detect(frames);
                } );
            }
            // Wait for all detectors to finish
            mThreadPool->waitAllThreads(); 

            if (gBench)
                timeSince(chronoStart, string("2.1 - Update detectors"));

            for_each (mFlows.begin(), mFlows.end(), [&] (Flow flow)
            {
                if (flow.run == false)
                    return;

                // Get the message resulting from the detection
                atom::Message message;
                message = flow.detector->getLastMessage();

                cv::Mat output = flow.detector->getOutput();
                lBuffers.push_back(output);
                flow.shm->setImage(output);

                lBufferNames.push_back(flow.detector->getName());

                // Send messages
                // Beginning of the frame
                lo_send(flow.client->get(), "/blobserver/startFrame", "ii", frameNbr, flow.id);

                int nbr, size;
                if (message.size() < 2)
                {
                    nbr = 0;
                    size = 0;
                }
                else
                {
                    nbr = atom::toInt(message[0]);
                    size = atom::toInt(message[1]);
                }

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

            if (gBench)
                timeSince(chronoStart, string("2.2 - Update buffers"));
        }

        if (lShowCamera)
        {
            // Check if the current source number is still available
            if (lSourceNumber >= lBuffers.size())
                lSourceNumber = 0;

            cv::Mat displayMat = lBuffers[lSourceNumber].clone();
            cv::putText(displayMat, lBufferNames[lSourceNumber].c_str(), cv::Point(10, 30),
                cv::FONT_HERSHEY_COMPLEX, 1.0, cv::Scalar::all(255.0));
            cv::imshow("blobserver", displayMat);
        }

        char lKey = cv::waitKey(1);
        if(lKey == 27) // Escape
            mRun = false;
        if(lKey == 'w')
        {
            lSourceNumber = (lSourceNumber+1)%lBuffers.size();
            cout << "Buffer displayed: " << lBufferNames[lSourceNumber] << endl;
        }

        unsigned long long chronoEnd = chrono::duration_cast<chrono::microseconds>(chrono::high_resolution_clock::now().time_since_epoch()).count();
        unsigned long long chronoElapsed = chronoEnd - chronoStart;
        
        timespec nap;
        nap.tv_sec = 0;
        if (chronoElapsed < usecPeriod)
            nap.tv_nsec = (usecPeriod - chronoElapsed) * 1e3;
        else
            nap.tv_nsec = 0;

        nanosleep(&nap, NULL);

        if (gBench)
            timeSince(chronoStart, string("3 - Total frame time"));

        frameNbr++;
    }

    mSourcesThread->join();

    return 0;
}

/*****************/
void App::updateSources()
{
    shared_ptr<App> theApp = App::getInstance();

    unsigned long long msecPeriod = 16;

    while(theApp->mRun)
    {
        unsigned long long chronoStart = chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now().time_since_epoch()).count();

        {
            lock_guard<mutex> lock(theApp->mSourceMutex);
            
            vector<shared_ptr<Source>>::iterator iter;
            // First we grab, then we retrieve all frames
            // This way, sync between frames is better
            for (iter = theApp->mSources.begin(); iter != theApp->mSources.end(); ++iter)
            {
                shared_ptr<Source> source = (*iter);
                source->grabFrame();
            
                // We also check if this source is still used
                if (source.use_count() == 2) // 2, because this ptr and the one in the vector
                {
                    cout << "Source " << source->getName() << " is no longer used. Disconnecting." << endl;
                    theApp->mSources.erase(iter);
                    --iter;
                }
            }
        }

        unsigned long long chronoEnd = chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now().time_since_epoch()).count();
        unsigned long long chronoElapsed = chronoEnd - chronoStart;

        timespec nap;
        nap.tv_sec = 0;
        if (chronoElapsed < msecPeriod)
            nap.tv_nsec = msecPeriod - chronoElapsed * 1e6;
        else
            nap.tv_nsec = 0;

        nanosleep(&nap, NULL);
    }
}

/*****************/
void App::oscError(int num, const char* msg, const char* path)
{
    cout << "liblo server error " << num << endl;
}

/*****************/
int App::oscGenericHandler(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data)
{
    if(gVerbose)
    {
        cout << "Unhandled message received:" << endl;

        for(int i = 0; i < argc; ++i)
        {
            lo_arg_pp((lo_type)(types[i]), argv[i]);
        }

        cout << endl;
    }

    return 1;
}

/*****************/
int App::oscHandlerConnect(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data)
{
    shared_ptr<App> theApp = App::getInstance();

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
    shared_ptr<OscClient> address(new OscClient(lo_address_new(atom::toString(message[0]).c_str(), port)));

    int error = lo_address_errno(address->get());
    if (error != 0)
    {
        cout << "Wrong address received, error " << error << endl;
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
        string detectorName;
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
        shared_ptr<Detector> detector;
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
        vector<shared_ptr<Source>> sources;
        atom::Message::const_iterator iter;
        for (iter = message.begin()+3; iter != message.end(); iter+=2)
        {
            if (iter+1 == message.end())
            {
                lo_send(address->get(), "/blobserver/connect", "s", "Missing sub-source number");
                return 1;
            }

            string sourceName;
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
            vector<shared_ptr<Source>>::const_iterator iterSource;
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
                shared_ptr<Source> source;
                if (theApp->mSourceFactory.key_exists(sourceName))
                    source = theApp->mSourceFactory.create(sourceName, sourceIndex);
                else
                {
                    string error = "Unable to create source ";
                    error += sourceName;
                    lo_send(address->get(), "/blobserver/connect", "s", error.c_str());
                    return 1;
                }
                
                if (!source->connect())
                {
                    string error = "Unable to connect to source ";
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
            lock_guard<mutex> lock(theApp->mFlowMutex);
            lock_guard<mutex> lockToo(theApp->mSourceMutex);

            // We can create the flow!
            Flow flow;
            
            flow.detector = detector;
            flow.client = address;
            flow.id = theApp->getValidId();
            flow.run = false;

            char shmFile[128];
            sprintf(shmFile, "/tmp/blobserver_output_%i", flow.id);
            flow.shm.reset(new ShmImage(shmFile));

            vector<shared_ptr<Source>>::const_iterator source;
            for (source = sources.begin(); source != sources.end(); ++source)
            {
                flow.sources.push_back(*source);

                // Add the sources to the mSources vector
                // (if they are not already there)
                bool isInSources = false;
                vector<shared_ptr<Source>>::const_iterator iter;
                for (iter = theApp->mSources.begin(); iter != theApp->mSources.end(); ++iter)
                {
                    if (iter->get()->getName() == source->get()->getName() && iter->get()->getSubsourceNbr() == source->get()->getSubsourceNbr())
                        isInSources = true;
                }
                if (!isInSources)
                    theApp->mSources.push_back(*source);

                // Adds a weak ptr to sources to the detector, for it to control them
                detector->addSource(*source);
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
    shared_ptr<App> theApp = App::getInstance();
    
    atom::Message message;
    atom::message_build_from_lo_args(message, types, argv, argc);

    string addressStr = atom::toString(message[0]);
    shared_ptr<OscClient> address(new OscClient(lo_address_new(addressStr.c_str(), "9000")));
    int error = lo_address_errno(address->get());
    if (error != 0)
    {
        cout << "Wrong address received, error " << error << endl;
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
    lock_guard<mutex> lock(theApp->mFlowMutex);
    vector<Flow>::iterator flow;
    for (flow = theApp->mFlows.begin(); flow != theApp->mFlows.end();)
    {
        if (string(lo_address_get_url(flow->client->get())) == string(lo_address_get_url(address->get())))
        {
            if (all == true || detectorId == flow->id)
            {
                lo_send(flow->client->get(), "/blobserver/disconnect", "s", "Disconnected");
                theApp->mFlows.erase(flow);
                cout << "Connection from address " << addressStr << " closed." << endl;
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
    shared_ptr<App> theApp = App::getInstance();    

    atom::Message message;
    atom::message_build_from_lo_args(message, types, argv, argc);
        
    string addressStr = atom::toString(message[0]);
    shared_ptr<OscClient> address(new OscClient(lo_address_new(addressStr.c_str(), "9000")));

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
        cout << "Wrong address received, error " << error << endl;
        return 1;
    }

    // Find the flow
    int result = 0;

    unsigned int flowId = (unsigned int)(atom::toInt(message[1]));
    vector<Flow>::iterator flow;
    for (flow = theApp->mFlows.begin(); flow != theApp->mFlows.end(); ++flow)
    {
        if (flow->id == flowId)
        {
            lock_guard<mutex> lock(theApp->mFlowMutex);

            // If the parameter is for the detector
            if (atom::toString(message[2]) == "Detector")
            {
                if (message.size() < 5)
                {
                    lo_send(flow->client->get(), "/blobserver/setParameter", "s", "Wrong number of arguments");
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
                    lo_send(flow->client->get(), "/blobserver/setParameter", "s", "Wrong number of arguments");
                    result = 1;
                }
                else
                {
                    int srcNbr = atom::toInt(message[3]);
                    if (srcNbr >= flow->sources.size())
                    {
                        lo_send(flow->client->get(), "/blobserver/setParameter", "s", "Wrong source index");
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
    shared_ptr<App> theApp = App::getInstance();

    atom::Message message;
    atom::message_build_from_lo_args(message, types, argv, argc);

    shared_ptr<OscClient> address;
    try
    {
        string addressStr = atom::toString(message[0]);
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
    string entity;

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
    for_each (theApp->mFlows.begin(), theApp->mFlows.end(), [&] (Flow flow)
    {
        if (flow.id == flowId)
        {
            lock_guard<mutex> lock(theApp->mFlowMutex);

            // If the parameter is for the detector
            if (entity == "Detector")
            {
                atom::Message msg;
                msg.push_back(message[3]);
                msg = flow.detector->getParameter(msg);

                lo_message oscMsg = lo_message_new();
                atom::message_build_to_lo_message(msg, oscMsg);
                lo_send_message(flow.client->get(), "/blobserver/getParameter", oscMsg);
            }
            // If the parameter is for the sources
            else if (entity == "Sources")
            {
                if (message.size() < 5)
                {
                    lo_send(flow.client->get(), "/blobserver/getParameter", "s", "Wrong number of arguments");
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
                        lo_send_message(flow.client->get(), "/blobserver/getParameter", oscMsg);
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
    shared_ptr<App> theApp = App::getInstance();

    atom::Message message;
    atom::message_build_from_lo_args(message, types, argv, argc);

    if (message.size() < 1)
        return 1;

    shared_ptr<OscClient> address;
    try
    {
        string addressStr = atom::toString(message[0]);
        address.reset(new OscClient(lo_address_new(addressStr.c_str(), "9000")));
    }
    catch (atom::BadTypeTagError exception)
    {
        return 1;
    }

    // Get all the available detectors
    vector<string> keys = theApp->mDetectorFactory.get_keys();

    atom::Message outMessage;
    for_each (keys.begin(), keys.end(), [&] (string key)
    {
        cout << key << endl;
        outMessage.push_back(atom::StringValue::create(key.c_str()));
    } );

    lo_message oscMsg = lo_message_new();
    atom::message_build_to_lo_message(outMessage, oscMsg);

    lo_send_message(address->get(), "/blobserver/detectors", oscMsg);
}

/*****************/
int App::oscHandlerGetSources(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data)
{
    shared_ptr<App> theApp = App::getInstance();

    atom::Message message;
    atom::message_build_from_lo_args(message, types, argv, argc);

    string addressStr;
    try
    {
        addressStr = atom::toString(message[0]);
    }
    catch (...)
    {
        return 1;
    }
    shared_ptr<OscClient> address(new OscClient(lo_address_new(addressStr.c_str(), "9000")));
    
    // If we have another parameter, it means we want to get availables subsources
    atom::Message outMessage;
    if (message.size() > 1)
    {
        string sourceName;
        try
        {
            sourceName = atom::toString(message[1]);
        }
        catch (...)
        {
            return 1;
        }

        // We try to create the named source
        shared_ptr<Source> source;
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
        vector<string> keys = theApp->mSourceFactory.get_keys();

        for_each (keys.begin(), keys.end(), [&] (string key)
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
    shared_ptr<App> theApp = App::getInstance();
    int ret;

    ret = theApp->init(argc, argv);
    if(ret != 0)
        return ret;

    ret = theApp->loop();
    return ret;
}