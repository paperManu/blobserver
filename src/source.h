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
 * @source.h
 * The Source base class.
 */

#ifndef SOURCE_H
#define SOURCE_H

#include <atomic>
#include <mutex>
#include <vector>

#include <opencv2/opencv.hpp>
#include <lcms2.h>
#include <atom/message.h>

using namespace std;

/*************/
// Buffering of cv::Mat
class MatBuffer
{
    public:
        MatBuffer(unsigned int size = 2);
        ~MatBuffer();

        MatBuffer& operator=(cv::Mat& mat);
        cv::Mat get();

    private:
        vector<cv::Mat> _mats;
        atomic_uint _head;
};

/*************/
// Source class
class Source
{
    public:
        Source();
        Source(int pParam);
        ~Source();

        static std::string getClassName() {return mClassName;}
        static std::string getDocumentation() {return mDocumentation;}

        // Returns all the subsources availables through this source
        // Needs to be redefined in subclasses
        virtual atom::Message getSubsources() {return atom::Message();}

        // Base methods
        virtual bool connect() {return true;}
        virtual bool disconnect() {return true;}
        virtual bool grabFrame() {}
        virtual cv::Mat retrieveFrame() {return mBuffer.get();}
        cv::Mat retrieveCorrectedFrame();

        virtual void setParameter(atom::Message pParam) {}
        virtual atom::Message getParameter(atom::Message pParam) {}

        // Useful parameters to get without a message
        std::string getName() {return mName;}
        unsigned int getWidth() {return mWidth;}
        unsigned int getHeight() {return mHeight;}
        unsigned int getChannels() {return mChannels;}
        unsigned int getFramerate() {return mFramerate;}
        unsigned int getSubsourceNbr() {return mSubsourceNbr;}


    protected:
        MatBuffer mBuffer; // Image buffer
        cv::Mat mCorrectedBuffer; // Corrected image buffer
        bool mUpdated;
        mutable mutex mMutex;

        // Base caracteristics of the source
        std::string mName;
        unsigned int mWidth, mHeight;
        unsigned int mChannels;
        unsigned int mFramerate;
        unsigned int mSubsourceNbr;
        unsigned int mId;
        
        // Base methods for any type of source
        void setBaseParameter(atom::Message pParam);
        atom::Message getBaseParameter(atom::Message pParam);

    private:
        static std::string mClassName;
        static std::string mDocumentation;
        
        // Distorsion parameters
        bool mCorrectDistortion;
        bool mCorrectVignetting;
        struct OpticalDesc
        {
            cv::Vec3f distortion;
            cv::Vec3f vignetting;
        } mOpticalDesc;
        
        cv::Mat mVignettingMat;
        cv::Mat mDistortionMat;
        bool mRecomputeVignettingMat;
        bool mRecomputeDistortionMat;

        // Color correction
        cmsHTRANSFORM mICCTransform;

        // Methods to correct the optical distortion
        cv::Mat correctVignetting(cv::Mat pImg);
        cv::Mat correctDistortion(cv::Mat pImg);

        // Methods related to colorimetry. Default output profile is sRGB
        cmsHTRANSFORM loadICCTransform(std::string pFile);
};

#endif // SOURCE_H
