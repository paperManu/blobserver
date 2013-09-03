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
 * @blobserver.h
 * The blobserver App main class.
 */

#ifndef BLOBSERVER_H
#define BLOBSERVER_H

#include <map>
#include <memory>
#include <mutex>
#include <thread>

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
#include "constants.h"
#include "actuator.h"
#include "source.h"
#include "threadPool.h"

#if HAVE_MAPPER
#include "mapper/mapper.h"
#endif

/*************/
// Struct to contain a complete flow, from capture to client
struct Flow
{
    std::vector<std::shared_ptr<Source>> sources;
    std::shared_ptr<Actuator> actuator;
    std::shared_ptr<OscClient> client;
#if HAVE_SHMDATA
    std::shared_ptr<Shm> sink;
#endif
#if HAVE_MAPPER
    std::vector<mapper_signal> mapperSignal;
#endif
    unsigned int id;
    bool run;
};

/*****************************/
// Definition of the app class
class App
{
    public:
        ~App();

        static std::shared_ptr<App> getInstance();

        // Initialization, depending on arguments
        int init(int argc, char** argv);

        // Main loop
        int loop();

        void stop();

    private:
        /***********/
        // Attributes
        // Singleton
        static std::shared_ptr<App> mInstance;

        bool mRun;

        // Factories
        factory::AbstractFactory<Actuator, std::string, std::string, int> mActuatorFactory;
        factory::AbstractFactory<Source, std::string, std::string, int> mSourceFactory;

        // liblo related
        lo_server_thread mOscServer;

        // libmapper related
#if HAVE_MAPPER
        mapper_device mMapperDevice;
#endif

        // detection related
        std::vector<std::shared_ptr<Source>> mSources;
        std::map<std::string, std::shared_ptr<OscClient>> mClients;
        std::vector<Flow> mFlows;
        // A mutex to prevent unexpected changes in flows
        std::mutex mFlowMutex;
        std::mutex mSourceMutex;
        // A thread pool for actuators
        std::shared_ptr<ThreadPool> mThreadPool;

        // Threads
        std::shared_ptr<std::thread> mSourcesThread;

        static unsigned int mCurrentId;

        /********/
        // Methods
        App();

        // Arguments parser
        int parseArgs(int argc, char **argv);

        // Factory registering
        void registerClasses();

        // Log handler
        static void logHandler(const gchar* log_domain, GLogLevelFlags log_level, const gchar* message, gpointer user_data);

        // Creates a new and unique ID for a flow
        unsigned int getValidId() {return ++mCurrentId;}

        // Sources update function, use in a thread
        static void updateSources();

        // OSC related, server side
        static void oscError(int num, const char* msg, const char* path);
        static int oscGenericHandler(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data);
        static int oscHandlerSignIn(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data);
        static int oscHandlerSignOut(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data);
        static int oscHandlerChangePort(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data);
        static int oscHandlerChangeIp(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data);
        static int oscHandlerConnect(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data);
        static int oscHandlerDisconnect(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data);
        static int oscHandlerSetParameter(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data);
        static int oscHandlerGetParameter(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data);
        static int oscHandlerGetActuators(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data);
        static int oscHandlerGetSources(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data);

        // OSC related, client side
        void sendToAllClients(const char* path, atom::Message& message);
};

#endif // BLOBSERVER_H
