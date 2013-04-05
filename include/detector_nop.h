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
 * switcher is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with switcher.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * @detector_nop.h
 * The Detector_Nop class.
 */

#ifndef DETECTOR_NOP_H
#define DETECTOR_NOP_H

#include "detector.h"

using namespace std;

 /*************/
// Class Detector_Nop
class Detector_Nop : public Detector
{
    public:
        Detector_Nop();
        Detector_Nop(int pParam);

        static std::string getClassName() {return mClassName;}
        static std::string getDocumentation() {return mDocumentation;}

        atom::Message detect(vector<cv::Mat> pCaptures);
        void setParameter(atom::Message pMessage);

    private:
        static string mClassName;
        static string mDocumentation;
        static unsigned int mSourceNbr;

        unsigned int mFrameNumber;

        void make();
};

#endif // DETECTOR_NOP_H
