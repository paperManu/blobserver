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
 * @actuator_bgsubtractor.h
 * The Actuator_BgSubtractor class.
 */

#ifndef BGSUBTRACTOR_H
#define BGSUBTRACTOR_H

#include <vector>

#include "config.h"
#include "actuator.h"
#include "blob_2D_color.h"

/*************/
// Class Actuator_BgSubtractor
class Actuator_BgSubtractor : public Actuator
{
    public:
        Actuator_BgSubtractor();
        Actuator_BgSubtractor(int pParam);

        static std::string getClassName() {return mClassName;}
        static std::string getDocumentation() {return mDocumentation;}

        atom::Message detect(const std::vector< Capture_Ptr > pCaptures);
        void setParameter(atom::Message pMessage);

    private:
        static std::string mClassName;
        static std::string mDocumentation;
        static unsigned int mSourceNbr;

        std::vector<Blob2DColor> mBlobs; // Vector of detected and tracked blobs

        // Some filtering parameters
        int mFilterSize;
        int mFilterDilateCoeff;
        
        // Maximum portion of the image considered as FG, above which detection
        // is not continued (to handle global lighting change for example)
        float mMaxFGPortion;

        // Tracking and movement filtering parameters
        int mBlobLifetime;
        int mKeepOldBlobs, mKeepMaxTime; // Parameters to set when we need blobs to be kept even when not detected anymore
        float mProcessNoiseCov, mMeasurementNoiseCov;
        float mMaxDistanceForColorDiff;

        // Background subtractor, used to select window of interest
        // to feed to the SVM
        cv::BackgroundSubtractorMOG2 mBgSubtractor;

        // Various variables
        cv::Mat mBgSubtractorBuffer;
        float mLearningRate, mLearningTime;
        float mMinArea, mMaxArea;

        // Methods
        void make();
};

REGISTER_ACTUATOR(Actuator_BgSubtractor)

#endif // BGSUBTRACTOR_H
