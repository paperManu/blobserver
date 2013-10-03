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
 * @glsl.h
 * The Actuator_GLSL class.
 */

#ifndef NOP_H
#define NOP_H

#include "actuator.h"
#include <GLFW/glfw3.h>

 /*************/
// Class Actuator_GLSL
class Actuator_GLSL : public Actuator
{
    public:
        Actuator_GLSL();
        Actuator_GLSL(int pParam);
        ~Actuator_GLSL();

        static std::string getClassName() {return mClassName;}
        static std::string getDocumentation() {return mDocumentation;}

        atom::Message detect(std::vector< Capture_Ptr > pCaptures);
        void setParameter(atom::Message pMessage);

        Capture_Ptr getOutput() const {return mCapture;}

    private:
        // Attributes
        static std::string mClassName;
        static std::string mDocumentation;

        static unsigned int mSourceNbr;
        unsigned int mFrameNumber;

        Capture_Ptr mCapture;

        bool isGlfw;
        GLFWwindow* mWindow;

        // Methods
        void make();
};

REGISTER_ACTUATOR(Actuator_GLSL)

#endif // NOP_H
