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
 * switcher is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with switcher.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @blobtrainer.cpp
 * This small utility is used to create models needed by the SVM classifier
 * used in some detectors.
 */

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <memory>
#include <vector>

#include <glib.h>
#include <opencv2/opencv.hpp>

#include "config.h"
#include "descriptor_hog.h"

using namespace std;

static gboolean gVersion = FALSE;
static gchar* gExtension = NULL;

static GOptionEntry gEntries[] =
{
    {"version", 'v', 0, G_OPTION_ARG_NONE, &gVersion, "Shows the version of this software", NULL},
    {"extension", 'e', 0, G_OPTION_ARG_STRING, &gExtension, "Specifies the extension for the files to load (default: png)", NULL},
    {NULL}
};

/*************/
int parseArgs(int argc, char** argv)
{
    GError *error = NULL;
    GOptionContext* context;

    context = g_option_context_new(" - blobtrainer, a utility to train SVM classifier for blobserver");
    g_option_context_add_main_entries(context, gEntries, NULL);

    if (!g_option_context_parse(context, &argc, &argv, &error))
    {
        cout << "Error while parsing options: " << error->message << endl;
        return 1;
    }

    if (gVersion)
    {
        cout << "blobtrainer " << PACKAGE_VERSION << ", from the blobserver suite." << endl;
        return 1;
    }

    if (gExtension == NULL)
    {
        gExtension = (gchar*)"png";
    }

    return 0;
}

/*************/
vector<string> loadFileList()
{
    vector<string> fileList;
    GError* error;
    GDir* directory;

    directory = g_dir_open((const gchar*)"./", 0, &error);
    const gchar* filename;
    while ((filename = g_dir_read_name(directory)) != NULL)
    {
        char* substr = strstr((char*)filename, (char*)gExtension);
        if (substr == filename + strlen((const char*)filename) - strlen((const char*)gExtension))
        {
            string strName = string((const char*)filename);
            fileList.push_back(strName);
        }
    }

    return fileList;
}

/*************/
int main(int argc, char** argv)
{
    int result = parseArgs(argc, argv);
    if (result)
        return result;

    return 0;
}
