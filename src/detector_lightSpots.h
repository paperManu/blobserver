/*
 * Copyright (C) 2012 Emmanuel Durand
 *
 * This file is part of blobserver.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * @detector_lightSpots.h
 * The Detector_LightSpots class.
 */

#ifndef DETECTOR_LIGHTSPOTS_H
#define DETECTOR_LIGHTSPOTS_H

#include "detector.h"
#include "blob_2D.h"

class Detector_LightSpots : public Detector
{
    public:
        Detector_LightSpots();

        atom::Message detect(cv::Mat pCapture);
        void setParameter(atom::Message pMessage);

    private:
        cv::SimpleBlobDetector* mLightBlobDetector; // OpenCV object which detects the blobs in an image
        std::vector<Blob2D> mLightBlobs; // Vector of detected and tracked blobs

        int mMaxTrackedBlobs;
        float mDetectionLevel;
        int mFilterSize;
        float mProcessNoiseCov, mMeasurementNoiseCov;
};

#endif // DETECTOR_LIGHTSPOTS_H
