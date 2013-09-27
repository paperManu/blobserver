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
 * @python.h
 * The Actuator_Python class.
 */

#ifndef PYTHON_H
#define PYTHON_H

#include "actuator.h"

#include <Python.h>

 /*************/
// Class Actuator_Python
class Actuator_Python : public Actuator
{
    public:
        Actuator_Python();
        Actuator_Python(int pParam);
        ~Actuator_Python();

        static std::string getClassName() {return mClassName;}
        static std::string getDocumentation() {return mDocumentation;}

        atom::Message detect(std::vector< Capture_Ptr > pCaptures);
        void setParameter(atom::Message pMessage);

    private:
        static std::string mClassName;
        static std::string mDocumentation;

        static unsigned int mSourceNbr;
        unsigned int mFrameNumber;

        bool mIsFileLoaded;
        PyObject* mPythonMain;
        PyObject* mPythonGlobal;
        PyObject* mPythonModule;

        PyObject* mPythonOutput;
        PyObject* mRawCapture;
        int mRawRows, mRawCols, mRawChannels;

        void make();
};

REGISTER_ACTUATOR(Actuator_Python)

#endif // PYTHON_H
