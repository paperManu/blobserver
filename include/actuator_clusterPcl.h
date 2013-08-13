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
 * @actuator_clusterPcl.h
 * The Actuator_ClusterPcl class.
 */

#ifndef ACTUATOR_CLUSTERPCL_H
#define ACTUATOR_CLUSTERPCL_H

#if HAVE_PCL

/*************/
// Class Actuator_ClusterPcl
class Actuator_ClusterPcl : public Actuator
{
    public:
        Actuator_ClusterPcl();
        Actuator_ClusterPcl(int pParam);

        static std::string getClassName() {return mClassName;}
        static std::string getDocumentation() {return mDocumentation;}

        atom::Message detect(std::vector<Capture_Ptr> pCaptures);
        void setParameter(atom::Message pMessage);

        Capture_Ptr getOutput() const {return mCapture;}

        std::shared_ptr<Shm> getShmObject(const char* filename) const {return std::shared_ptr<ShmPcl>(new ShmPcl(filename));}

    private:
        static std::string mClassName;
        static std::string mDocumentation;

        static unsigned int mSourceNbr;
        unsigned int mFrameNumber;

        Capture_Ptr mCapture;

        void make();
};

#endif //HAVE_PCL

#endif // ACTUATOR_CLUSTERPCL_H
