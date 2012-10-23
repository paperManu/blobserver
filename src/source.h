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
 * @source.h
 * The Source base class.
 */

 #ifndef SOURCE_H
 #define SOURCE_H

#include "opencv2/opencv.hpp"

class Source
{
    public:
        Source();

        virtual bool connect() {return true;};
        virtual bool disconnect() {return true;};
        virtual bool grabFrame() {};
        virtual cv::Mat retrieveFrame() {return cv::Mat::zeros(1, 1, CV_8U);};
        virtual void setParameter(const char* pParam, float pValue) {};

        unsigned int getWidth() {return mWidth;};
        unsigned int getHeight() {return mHeight;};
        unsigned int getChannels() {return mChannels;};
        unsigned int getFramerate() {return mFramerate;};

    protected:
        // Base caracteristics of the grabber
        unsigned int mWidth, mHeight;
        unsigned int mChannels;
        unsigned int mFramerate;
};

 #endif // SOURCE_H
