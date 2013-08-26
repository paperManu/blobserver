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
 * @actuator_depthtouch.h
 * The Actuator_DepthTouch class.
 */

#ifndef ACTUATOR_DEPTHTOUCH_H
#define ACTUATOR_DEPTHTOUCH_H

#include <vector>

#include "config.h"
#include "actuator.h"
#include "blob_2D.h"

/*************/
// Class Actuator_DepthTouch
class Actuator_DepthTouch : public Actuator
{
    public:
        Actuator_DepthTouch();
        Actuator_DepthTouch(int pParam);

        static std::string getClassName() {return mClassName;}
        static std::string getDocumentation() {return mDocumentation;}

        atom::Message detect(const std::vector< Capture_Ptr > pCaptures);
        void setParameter(atom::Message pMessage);

        std::shared_ptr<Shm> getShmObject(const char* filename) const {return std::shared_ptr<Shm>(new ShmImage(filename));}

    private:
        static std::string mClassName;
        static std::string mDocumentation;
        static unsigned int mSourceNbr;

        // Detection parameters
        int mFilterSize;
        float mDetectionDistance;
        float mSigmaCoeff;
        int mLearningTime;

        // Internal variables
        bool mIsLearning, mJustLearnt;
        int mLearningLeft;

        // Tracking and movement filtering parameters
        std::vector<Blob2D> mBlobs; // Vector of detected and tracked blobs
        int mBlobLifetime;
        float mProcessNoiseCov, mMeasurementNoiseCov;

        cv::Mat mBackgroundMean;
        cv::Mat mBackgroundStddev;
        std::vector<cv::Mat> mLearningData;

        // Methods
        void make();
        void learn(cv::Mat input);
};

#endif // ACTUATOR_DEPTHTOUCH_H
