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
 * @source_2d_gige.h
 * The Source_2D_Gige class.
 */

#ifndef SOURCE_2D_GIGE_H
#define SOURCE_2D_GIGE_H

#include "config.h"
#include "source_2d.h"

#if HAVE_ARAVIS

#include <arv.h>
#include <mutex>

class Source_2D_Gige : public Source_2D
{
    public:
        Source_2D_Gige();
        Source_2D_Gige(std::string pParam);
        ~Source_2D_Gige();

        static std::string getClassName() {return mClassName;}
        static std::string getDocumentation() {return mDocumentation;}

        atom::Message getSubsources() const; 
        virtual bool grabFrame();

        bool connect();
        bool disconnect();
        cv::Mat retrieveRawFrame();

        void setParameter(atom::Message pParam);
        atom::Message getParameter(atom::Message pParam) const;

    private:
        static std::string mClassName;
        static std::string mDocumentation;

        cv::Mat mConvertedFrame;
        std::mutex mFrameMutex;

        ArvCamera* mCamera;
        ArvStream* mStream;

        bool mInvertRGB, mBayer;

        // Methods
        void make(std::string pParam);
        void allocateStream();
        static void streamCb(void* user_data, ArvStreamCallbackType type, ArvBuffer* buffer);
};

#endif // HAVE_ARAVIS

#endif // SOURCE_2D_OPENCV_H
