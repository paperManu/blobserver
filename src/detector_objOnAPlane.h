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
 * @detector_objOnAPlane.h
 * The Detector_ObjOnAPlane class.
 */

 #ifndef DETECTOR_OBJONAPLANE_H
 #define DETECTOR_OBJONAPLANE_H

#include <memory>
#include "detector.h"
#include "blob_2D.h"

class Detector_ObjOnAPlane : public Detector
{
    public:
        Detector_ObjOnAPlane();
        Detector_ObjOnAPlane(int pParam);

        atom::Message detect(std::vector<cv::Mat> pCaptures);
        void setParameter(atom::Message pMessage);

    private:
        static std::string mClassName;
        static std::string mDocumentation;

        int mMaxTrackedBlobs;
        float mDetectionLevel;
        float mProcessNoiseCov, mMeasurementNoiseCov;
        
        std::shared_ptr<cv::SimpleBlobDetector> mBlobDetector; // OpenCV object which detects the blobs in an image
        std::vector<Blob2D> mBlobs; // Vector of detected and tracked blobs

        std::vector<std::vector<cv::Vec2f>> mSpaces; // First space is the real plane
        std::vector<cv::Mat> mMaps;
        bool mMapsUpdated;

        void make(); // Called by the constructor
        void updateMaps(std::vector<cv::Mat> pCaptures); // Updates the space conversion maps
};

 #endif // DETECTOR_OBJONAPLANE_H
