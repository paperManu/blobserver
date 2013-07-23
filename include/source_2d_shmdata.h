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
 * @source_2d_shmdata.h
 * The Source_2D_Shmdata class.
 */

#ifndef SOURCE_SHMDATA_H
#define SOURCE_SHMDATA_H

#include "config.h"
#if HAVE_SHMDATA
#include <mutex>
#include <shmdata/any-data-reader.h>

#include "source_2d.h"

class Source_2D_Shmdata : public Source_2D
{
    public:
        Source_2D_Shmdata();
        Source_2D_Shmdata(int pParam);
        ~Source_2D_Shmdata();

        static std::string getClassName() {return mClassName;}
        static std::string getDocumentation() {return mDocumentation;}

        atom::Message getSubsources() const; 

        bool connect();
        bool disconnect();
        bool grabFrame();
        cv::Mat retrieveFrame();

        void setParameter(atom::Message pParam);
        atom::Message getParameter(atom::Message pParam) const;

    private:
        static std::string mClassName;
        static std::string mDocumentation;

        shmdata_any_reader_t* mReader;

        void make(int pParam);
        static void onData(shmdata_any_reader_t* reader, void* shmbuf, void* data, int data_size, unsigned long long timestamp,
            const char* type_description, void* user_data);
};
#endif // HAVE_SHMDATA

#endif // SOURCE_SHMDATA_H
