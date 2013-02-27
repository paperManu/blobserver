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
 * @blobcontroller.cpp
 * Small standalone application to control blobserver.
 */

#include <iostream>
#include <stdio.h>
#include <glib.h>

#include "configurator.h"

using namespace std;

static gboolean gVersion = FALSE;
static gchar* gConfigFile = NULL;

static GOptionEntry gEntries[] =
{
    {"version", 'v', 0, G_OPTION_ARG_NONE, &gVersion, "Shows version of this software", NULL},
    {"config", 'C', 0, G_OPTION_ARG_STRING, &gConfigFile, "Specify a configuration file to load at startup", NULL},
    {NULL}
};

int main(int argc, char** argv)
{
    GError *error = NULL;
    GOptionContext* context;

    context = g_option_context_new("- blobcontroller, controls blobserver through OSC");
    g_option_context_add_main_entries(context, gEntries, NULL);

    if (!g_option_context_parse(context, &argc, &argv, &error))
    {
        cout << "Error while parsing options: " << error->message << endl;
        return 1;
    }

    if (gVersion)
    {
        std::cout << PACKAGE_TARNAME << " " << PACKAGE_VERSION << std::endl;
        return 1;
    }

    if (gConfigFile == NULL)
    {
        cout << "You need to specify a configuration file, otherwise this software is of no use..." << endl;
    }
    else
    {
        Configurator configurator;
        configurator.loadXML((char*)gConfigFile);
    }

    return 0;
}
