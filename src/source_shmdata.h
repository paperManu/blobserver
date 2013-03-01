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

/*
 * @source_shmdata.h
 * The Source_Shmdata class.
 */

#ifndef SOURCE_SHMDATA_H
#define SOURCE_SHMDATA_H

#include <mutex>
#include <shmdata/any-data-reader.h>

#include "source.h"

using namespace std;

class Source_Shmdata : public Source
{
    public:
        Source_Shmdata();
        Source_Shmdata(int pParam);
        ~Source_Shmdata();

        static string getClassName() {return mClassName;}
        static string getDocumentation() {return mDocumentation;}

        atom::Message getSubsources(); 

        bool connect();
        bool disconnect();
        bool grabFrame();
        cv::Mat retrieveFrame();

        void setParameter(atom::Message pParam);
        atom::Message getParameter(atom::Message pParam);

    private:
        static string mClassName;
        static string mDocumentation;

        shmdata_any_reader_t* mReader;

        void make(int pParam);
        static void onData(shmdata_any_reader_t* reader, void* shmbuf, void* data, int data_size, unsigned long long timestamp,
            const char* type_description, void* user_data);
};

#endif // SOURCE_SHMDATA_H
