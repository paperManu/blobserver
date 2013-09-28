/*
 * Copyright (C) 2013 Emmanuel Durand
 *
 * This file is part of blobserver.
 *
 * This program is free software: you can redistribute it and/or modify
 *
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
 * @mirrorball.h
 * The Actuator_MirrorBall class.
 */

#ifndef MIRRORBALL_H
#define MIRRORBALL_H

#include "actuator.h"

 /*************/
// Class Actuator_MirrorBall
class Actuator_MirrorBall : public Actuator
{
    public:
        Actuator_MirrorBall();
        Actuator_MirrorBall(int pParam);

        static std::string getClassName() {return mClassName;}
        static std::string getDocumentation() {return mDocumentation;}

        atom::Message detect(std::vector< Capture_Ptr > pCaptures);
        void setParameter(atom::Message pMessage);

    private:
        // Attributes
        static std::string mClassName;
        static std::string mDocumentation;

        static unsigned int mSourceNbr;
        unsigned int mFrameNumber;

        cv::Mat mImage;
        cv::Mat mSphereImage; // image cropped to contain only the chromed sphere
        cv::Mat mEquiImage;

        float mFOV, mCroppedFOV; // FOV of the whole image, FOV of the cropped one
        cv::Vec3f mSphere; // position and radius of the sphere projection in the input image
        std::vector<cv::Vec3f> mSpherePositions; // Positions of the sphere in the mTrackingLenght previous images
        bool mFixedSphere; // If mSphere is set by the config file, this is true

        float mSphereDiameter; // Real sphere radius in mm
        float mSphereReflectance;
        float mCameraDistance;

        cv::Mat mProjectionMap;

        unsigned int mTrackingLength; // Averager length for the sphere detection
        float mThreshold; // Threshold to test if the sphere has moved

        // Methods
        void make();
        cv::Vec3f detectSphere();
        cv::Vec3f filterSphere(cv::Vec3f newSphere);
        void getDistanceFromCamera();
        // Creates the transformation map
        void createTransformationMap();
        // Method to calculate illumination direction from the position of pixels on the chromed sphere
        cv::Vec2f directionFromViewAngle(cv::Vec2f angle);
        // Converts the projection map from angular values to equirectangular
        void projectionMapFromDirections();
};

REGISTER_ACTUATOR(Actuator_MirrorBall)

#endif // MIRRORBALL_H
