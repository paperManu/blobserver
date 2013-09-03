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
 * @capture.h
 * The Capture base class.
 */

#ifndef CAPTURE_H
#define CAPTURE_H

#include <memory>

#include "config.h"

#include <atom/stringvalue.h>
#include <glib.h>
#include <opencv2/opencv.hpp>
#if HAVE_PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#endif // HAVE_PCL

/*************/
class Capture
{
    public:
        Capture() {};
        ~Capture() {};

        virtual std::string type() {};
};

/*************/
class Capture_2D_Mat : public Capture
{
    public:
        Capture_2D_Mat() {};
        Capture_2D_Mat(cv::Mat input) {mBuffer = input;}

        cv::Mat& get() {return mBuffer;}

        std::string type() {return std::string("Capture_2D_Mat");}

    private:
        cv::Mat mBuffer;
};

#if HAVE_PCL
/*************/
class Capture_3D_PclRgba : public Capture
{
    public:
        Capture_3D_PclRgba() {};
        Capture_3D_PclRgba(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input) {mPcl = input;}

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr get() {return mPcl;}

        std::string type() {return std::string("Capture_3D_Pcl");}

    private:
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr mPcl;
};
#endif //HAVE_PCL

typedef std::shared_ptr<Capture> Capture_Ptr;
typedef std::shared_ptr<Capture_2D_Mat> Capture_2D_Mat_Ptr;
#if HAVE_PCL
typedef std::shared_ptr<Capture_3D_PclRgba> Capture_3D_PclRgba_Ptr;
#endif

#endif // CAPTURE_H
