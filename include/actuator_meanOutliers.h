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
 * @actuator_mainOutliers.h
 * The Actuator_MainOutliers class.
 */

#ifndef ACTUATOR_MEANOUTLIERS_H
#define ACTUATOR_MEANOUTLIERS_H

#include "actuator.h"
#include "blob_2D.h"

/*************/
// Class Actuator_MeanOutliers
class Actuator_MeanOutliers : public Actuator
{
    public:
        Actuator_MeanOutliers();
        Actuator_MeanOutliers(int pParam);

        static std::string getClassName() {return mClassName;}
        static std::string getDocumentation() {return mDocumentation;}

        atom::Message detect(const std::vector< Capture_Ptr > pCaptures);
        void setParameter(atom::Message pMessage);

        std::shared_ptr<Shm> getShmObject(const char* filename) const {return std::shared_ptr<Shm>(new ShmImage(filename));}

    private:
        static std::string mClassName;
        static std::string mDocumentation;
        static unsigned int mSourceNbr;

        float mDetectionLevel; // Above std dev * mDetectionLevel, an object is detected
        int mFilterSize;
        Blob2D mMeanBlob;
        bool isInitialized;

        void make();
};

#endif // ACTUATOR_MEANOUTLIERS_H
