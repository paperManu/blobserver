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
 * @detector_bgsubtractor.h
 * The Detector_BgSubtractor class.
 */

#ifndef DETECTOR_BGSUBTRACTOR_H
#define DETECTOR_BGSUBTRACTOR_H

#include <vector>

#include "config.h"
#include "detector.h"
#include "blob_2D.h"

/*************/
// Class Detector_BgSubtractor
class Detector_BgSubtractor : public Detector
{
    public:
        Detector_BgSubtractor();
        Detector_BgSubtractor(int pParam);

        static std::string getClassName() {return mClassName;}
        static std::string getDocumentation() {return mDocumentation;}

        atom::Message detect(const std::vector<cv::Mat> pCaptures);
        void setParameter(atom::Message pMessage);

    private:
        static std::string mClassName;
        static std::string mDocumentation;
        static unsigned int mSourceNbr;

        std::vector<Blob2D> mBlobs; // Vector of detected and tracked blobs

        // Some filtering parameters
        int mFilterSize;
        int mFilterDilateCoeff;

        // Tracking and movement filtering parameters
        int mBlobLifetime;
        float mProcessNoiseCov, mMeasurementNoiseCov;

        // Background subtractor, used to select window of interest
        // to feed to the SVM
        cv::BackgroundSubtractorMOG2 mBgSubtractor;

        // Various variables
        cv::Mat mBgSubtractorBuffer;
        float mMinArea, mMaxArea;

        // Methods
        void make();
};

#endif // DETECTOR_BGSUBTRACTOR_H
