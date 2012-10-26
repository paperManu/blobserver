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
 * @source_opencv.h
 * The Source_OpenCV class.
 */

#ifndef SOURCE_OPENCV_H
#define SOURCE_OPENCV_H

#include "source.h"

class Source_OpenCV : public Source
{
    public:
        Source_OpenCV();
        Source_OpenCV(int pParam);
        ~Source_OpenCV();

        static std::string getClassName() {return mClassName;}
        static std::string getDocumentation() {return mDocumentation;}

        bool connect();
        bool disconnect();
        bool grabFrame();
        cv::Mat retrieveFrame();
        void setParameter(const char* pParam, float pValue);

    private:
        static std::string mClassName;
        static std::string mDocumentation;

        cv::VideoCapture mCamera;
        cv::Mat mBuffer;
};

#endif // SOURCE_OPENCV_H
