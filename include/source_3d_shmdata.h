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
 * @source_3d_shm.h
 * The Source_3D_Shmdata class.
 */

#ifndef SOURCE_3D_SHM
#define SOURCE_3D_SHM

#include <memory>

#include <glib.h>
#include <atom/message.h>

#include "config.h"

#if HAVE_PCL && HAVE_SHMDATA
#include "capture_pcl.h"
#include "constants.h"
#include "helpers.h"
#include "shmpointcloud.h"
#include "source.h"

/*************/
//! Class Source_3D_Shmdata, which reads point clouds (XYZRGBA) from a shmdata
class Source_3D_Shmdata : public Source
{
    public:
        Source_3D_Shmdata();
        Source_3D_Shmdata(std::string pParam);
        ~Source_3D_Shmdata();

        static std::string getClassName() {return mClassName;}
        static std::string getDocumentation() {return mDocumentation;}

        atom::Message getSubsources() const {return atom::Message();}

        bool connect();
        bool disconnect();
        bool grabFrame();
        Capture_Ptr retrieveFrame();

        void setParameter(atom::Message pParam);
        atom::Message getParameter(atom::Message pParam) const;

    private:
        static std::string mClassName;
        static std::string mDocumentation;

        std::shared_ptr< ShmPointCloud<pcl::PointXYZRGBA> > mShm;

        void make(std::string pParam);
};

#endif // HAVE_PCL && HAVE_SHMDATA
#endif // SOURCE_3D_SHM
