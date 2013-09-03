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
 * @capture_pcl.h
 * The pcl related Capture classes.
 */

#ifndef CAPTURE_PCL
#define CAPTURE_PCL

#include "capture.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

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

typedef std::shared_ptr<Capture_3D_PclRgba> Capture_3D_PclRgba_Ptr;

#endif // CAPTURE_PCL
