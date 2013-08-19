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
 * @base_objects.h
 * A few classes and structs used in multiple files
 */

#ifndef BASE_OBJECTS_H
#define BASE_OBJECTS_H

#include <opencv2/opencv.hpp>
#include <lo/lo.h>

#include "capture.h"
#include "config.h"
#if HAVE_SHMDATA
#include <shmdata/any-data-writer.h>
#endif
#if HAVE_PCL
#include "shmpointcloud.h"
#endif

/*************/
// lo_address in an object
class OscClient
{
    public:
        OscClient(lo_address pAddress) {mAddress = pAddress;}
        ~OscClient() {lo_address_free(mAddress);}

        lo_address get() const {return mAddress;}
        void replace(lo_address newAddress)
        {
            lo_address_free(mAddress);
            mAddress = newAddress;
        }

    private:
        lo_address mAddress;
};

/*************/
// LookupTable objects
class LookupTable
{
    public:
        enum interpolation
        {
            linear = 0,
            bezier = 1
        };
        LookupTable(interpolation inter, std::vector< std::vector<float> > keys);

        float operator[](const float& value);

    private:
        interpolation mInterpolation;
        float mStart[2], mEnd[2];
        std::vector< std::vector<float> > mKeys;
};

#if HAVE_SHMDATA
/*************/
// Generic Shmdata writer class
class Shm
{
    public:
        virtual void setCapture(Capture_Ptr& capture, const unsigned long long timestamp = 0) {};

    private:
        shmdata_any_writer_t* _writer;
        std::string _filename;
};

/*************/
// Class which auto-selects the shm type
class ShmAuto : public Shm
{
    public:
        ShmAuto(const char* filename);
        void setCapture(Capture_Ptr& capture, const unsigned long long timestamp = 0);

    private:
        std::shared_ptr<Shm> mShm;
        std::string mFilename;
};

/*************/
// Simple class to send image through shm
class ShmImage : public Shm
{
    public:
        ShmImage(const char* filename);
        ~ShmImage();
        void setCapture(Capture_Ptr& capture, const unsigned long long timestamp = 0);

    private:
        shmdata_any_writer_t* _writer;
        std::string _filename;
        int _type;
        unsigned int _width, _height, _bpp;
        unsigned long long _startTime;

        bool init(const unsigned int width, const unsigned int height, int type);
};
#endif // HAVE_SHMDATA

#if HAVE_PCL
/*************/
// Class to write a PCL through shm
class ShmPcl : public Shm
{
    public:
        ShmPcl(const char* filename);
        void setCapture(Capture_Ptr& capture, const unsigned long long timestamp = 0);

    private:
        std::shared_ptr< ShmPointCloud<pcl::PointXYZRGBA> > _writer;
};
#endif // HAVE_PCL

#endif // BASE_OBJECTS_H
