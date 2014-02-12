/*
 * Copyright (C) 2014 Emmanuel Durand
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
 * @face.h
 * The Actuator_Face class.
 */

#ifndef FACE_H
#define FACE_H

#include <vector>
#include <opencv2/objdetect/objdetect.hpp>

#include "config.h"
#include "actuator.h"
#include "blob_2D_color.h"

/*************/
// Class Actuator_Face
class Actuator_Face : public Actuator
{
    public:
        Actuator_Face();
        Actuator_Face(int pParam);

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

        // Tracking and movement filtering parameters
        int mBlobLifetime;
        int mKeepOldBlobs, mKeepMaxTime; // Parameters to set when we need blobs to be kept even when not detected anymore
        float mProcessNoiseCov, mMeasurementNoiseCov;
        float mMaxDistanceForColorDiff;

        // Eye and face detection objects
        cv::CascadeClassifier mFaceCascade;
        cv::CascadeClassifier mEyeCascade;

        struct Person
        {
            cv::Rect face;
            std::vector<cv::Rect> eyes;
        };
        std::vector<Person> mPersons;

        // Methods
        void make();
};

REGISTER_ACTUATOR(Actuator_Face)

#endif // BGSUBTRACTOR_H
