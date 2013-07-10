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
 * @detector_stitch.h
 * The Detector_Stitch class.
 */

#ifndef DETECTOR_STITCH_H
#define DETECTOR_STITCH_H

#include "detector.h"
 #include "base_objects.h"

 /*************/
// Class Detector_Stitch
class Detector_Stitch : public Detector
{
    public:
        Detector_Stitch();
        Detector_Stitch(int pParam);

        static std::string getClassName() {return mClassName;}
        static std::string getDocumentation() {return mDocumentation;}

        atom::Message detect(std::vector<cv::Mat> pCaptures);
        void setParameter(atom::Message pMessage);

    private:
        static std::string mClassName;
        static std::string mDocumentation;
        static unsigned int mSourceNbr;

        unsigned int mFrameNumber;

        bool defineOutputResolution;
        bool source_crop[2];
        unsigned int source_crop_parameters[2][4];
        unsigned int source_pos[2][2];

        void make();
};

#endif // DETECTOR_STITCH_H
