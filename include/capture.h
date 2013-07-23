/*
 * Copyright (C) 2013 Emmanuel Durand
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
 * @capture.h
 * The Capture base class.
 */

#ifndef CAPTURE_H
#define CAPTURE_H

#include <glib.h>

/*************/
//! Class containing a capture from a source
template <typename T>
class Capture
{
    public:
        Capture();
        Capture(const T& input);
        ~Capture();

        Capture& operator=(const T& input);
        T& operator()();

        type_info type() {return typeid(mBuffer);}

    private:
        T mBuffer;
};


/*************/
template <typename T>
Capture::Capture()
{
}

/*************/
template <typename T>
Capture::Capture(const T& input)
{
    mBuffer = input;
}

/*************/
template <typename T>
Capture::~Capture()
{
}

/*************/
template <typename T>
Capture& Capture::operator=(const T& input)
{
    mBuffer = input;
    return *this;
}

/*************/
template <typename T>
T& Capture::operator=()()
{
    return mBuffer;
}

#endif // CAPTURE_H
