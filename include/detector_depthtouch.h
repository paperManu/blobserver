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
 * @detector_depthtouch.h
 * The Detector_DepthTouch class.
 */

#ifndef DETECTOR_DEPTHTOUCH_H
#define DETECTOR_DEPTHTOUCH_H

#include <vector>

#include "config.h"
#include "detector.h"
#include "blob_2D.h"

/*************/
// Class Detector_DepthTouch
class Detector_DepthTouch : public Detector
{
    public:
        Detector_DepthTouch();
        Detector_DepthTouch(int pParam);

        static std::string getClassName() {return mClassName;}
        static std::string getDocumentation() {return mDocumentation;}

        atom::Message detect(const std::vector<cv::Mat> pCaptures);
        void setParameter(atom::Message pMessage);

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
        bool mIsLearning;
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

#endif // DETECTOR_DEPTHTOUCH_H
