/*
 * Copyright (C) 2012 Emmanuel Durand
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
 * @nop.h
 * The Actuator_Nop class.
 */

#ifndef NOP_H
#define NOP_H

#include "actuator.h"

 /*************/
// Class Actuator_Nop
class Actuator_Nop : public Actuator
{
    public:
        Actuator_Nop();
        Actuator_Nop(int pParam);

        static std::string getClassName() {return mClassName;}
        static std::string getDocumentation() {return mDocumentation;}

        atom::Message detect(std::vector< Capture_Ptr > pCaptures);
        void setParameter(atom::Message pMessage);

        std::vector<Capture_Ptr> getOutput() const;

    private:
        static std::string mClassName;
        static std::string mDocumentation;

        static unsigned int mSourceNbr;
        unsigned int mFrameNumber;

        Capture_Ptr mCapture;

        void make();
};

REGISTER_ACTUATOR(Actuator_Nop)

#endif // NOP_H
