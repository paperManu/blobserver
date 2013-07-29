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
 * @detector_nop.h
 * The Detector_Nop class.
 */

#ifndef DETECTOR_NOP_H
#define DETECTOR_NOP_H

#include "detector.h"

 /*************/
// Class Detector_Nop
class Detector_Nop : public Detector
{
    public:
        Detector_Nop();
        Detector_Nop(int pParam);

        static std::string getClassName() {return mClassName;}
        static std::string getDocumentation() {return mDocumentation;}

        atom::Message detect(std::vector< Capture_Ptr > pCaptures);
        void setParameter(atom::Message pMessage);

        std::shared_ptr<Shm> getShmObject(const char* filename) const {return std::shared_ptr<Shm>(new ShmImage(filename));}

    private:
        static std::string mClassName;
        static std::string mDocumentation;
        static unsigned int mSourceNbr;

        unsigned int mFrameNumber;

        void make();
};

#endif // DETECTOR_NOP_H
