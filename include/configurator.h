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
 * blobserver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with blobserver.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * @config_file.h
 * The Configurator base class.
 */

#ifndef CONFIG_FILE_H
#define CONFIG_FILE_H

#include <stdio.h>
#include <atomic>

#include <libxml/xmlreader.h>
#include <libxml/xmlwriter.h>
#include <atom/message.h>
#include <lo/lo.h>

#include "config.h"
#include "base_objects.h"

#define LOAD_MAX_WAIT_TIME_MS 5000

class Configurator
{
    public:
        Configurator(int proto = LO_UDP);
        ~Configurator();

        void loadXML(const char* filename, bool distant = false);

    private:
        /*** Attributes ***/
        bool mReady;
        int mProto;

        lo_server_thread mOscServer;

        std::atomic_int mLastIndexReceived;
        bool mVerbose;

        /*** Methods ***/
        bool loadFlow(const xmlDocPtr doc, xmlNodePtr cur, bool distant = false);

        std::string getStringValueFrom(const xmlDocPtr doc, const xmlNodePtr cur, const xmlChar* attr);
        int getIntValueFrom(const xmlDocPtr doc, const xmlNodePtr cur, const xmlChar* attr);
        bool getParamValuesFrom(const xmlDocPtr doc, xmlNodePtr cur, std::string& paramName, atom::Message& values);

        void checkString(std::string& str, const std::string defaultStr);
        void checkInt(int& value, const int defaultValue);

        static void oscError(int num, const char* msg, const char* path);
        static int oscGenericHandler(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data);
        static int oscHandlerConnect(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data);
};

#endif // BASE_OBJECTS_H
