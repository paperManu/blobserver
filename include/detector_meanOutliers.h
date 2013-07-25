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
 * @detector_mainOutliers.h
 * The Detector_MainOutliers class.
 */

#ifndef DETECTOR_MEANOUTLIERS_H
#define DETECTOR_MEANOUTLIERS_H

#include "detector.h"
#include "blob_2D.h"

/*************/
// Class Detector_MeanOutliers
class Detector_MeanOutliers : public Detector
{
    public:
        Detector_MeanOutliers();
        Detector_MeanOutliers(int pParam);

        static std::string getClassName() {return mClassName;}
        static std::string getDocumentation() {return mDocumentation;}

        atom::Message detect(const std::vector< Capture_Ptr > pCaptures);
        void setParameter(atom::Message pMessage);

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

#endif // DETECTOR_MEANOUTLIERS_H
