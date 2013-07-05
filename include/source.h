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

#include "helpers.h"
#include "hdribuilder.h"

/*************/
//! A simple buffer of cv::Mat
class MatBuffer
{
    public:
        /**
         * \brief Constructor
         * \param size The size of the buffer
         */
        MatBuffer(unsigned int size = 3);

        /**
         * \brief Assigns a new value to the buffer, automatically marked as the current head value
         * \param mat The new cv::Mat which will be used as head of the buffer
         */
        MatBuffer& operator=(cv::Mat& mat);

        /**
         * \brief Gets the current head of the buffer
         * \return Returns the cv::Mat which is the latest value of the buffer
         */
        cv::Mat get() const;

    private:
        std::vector<cv::Mat> _mats; //!< All the cv::Mat currently held by the buffer
        std::atomic_uint _head; //!< Index of the head of the buffer
};

/*************/
//! Base Source class, from which all Source classes derive
class Source
{
    public:
        /**
         * \brief Default constructor
         */
        Source();

        /**
         * \brief Constructor allowing to specify a subsource
         * \param pParam Index of the subsource to allocate
         */
        Source(int pParam);

        /**
         * \brief Destructor
         */
        ~Source();

        /**
         * \return Returns the class name of the source
         */
        static std::string getClassName() {return mClassName;}

        /**
         * \return Returns the documentation of the source
         */
        static std::string getDocumentation() {return mDocumentation;}

        /**
         * \brief Allows to get all the availables subsources for a given source
         * \return Returns a message containing all subsources name
         */
        virtual atom::Message getSubsources() const {return atom::Message();}

        // Base methods
        /**
         * \brief Connects to the source, depending on the specified subsource number
         */
        virtual bool connect() {return true;}

        /**
         * \brief Disconnects the source
         */
        virtual bool disconnect() {return true;}

        /**
         * \brief Tells the source to grab a frame, without returning it yet
         */
        virtual bool grabFrame() {}

        /**
         * \brief Retrieves the last frame grabbed by the source with grabFrame()
         */
        virtual cv::Mat retrieveFrame() {return mBuffer.get();}

        /**
         * \brief Retrieves the last frame grabbed by the source, corrected with the various available corrections if specified so
         */
        cv::Mat retrieveModifiedFrame();

        /**
         * \brief Sets a parameter
         * \param pParam A message containing the name of the parameter, and its desired value
         */
        virtual void setParameter(atom::Message pParam) {}

        /**
         * \brief Gets the current value for a given parameter
         * \param pParam A message containing the name of the parameter
         * \return Returns a message containing the name of the parameter and its current value
         */
        virtual atom::Message getParameter(atom::Message pParam) const {}

        /**
         * \brief Gets the name of the source
         */
        std::string getName() const {return mName;}
        /**
         * \brief Gets the width of the source
         */
        unsigned int getWidth() const {return mWidth;}
        /**
         * \brief Gets the height of the source
         */
        unsigned int getHeight() const {return mHeight;}
        /**
         * \brief Gets the channel number of the source
         */
        unsigned int getChannels() const {return mChannels;}
        /**
         * \brief Gets the framerate of the source
         */
        unsigned int getFramerate() const {return mFramerate;}
        /**
         * \brief Gets the subsource number
         */
        unsigned int getSubsourceNbr() const {return mSubsourceNbr;}


    protected:
        MatBuffer mBuffer; //!< Image buffer
        cv::Mat mCorrectedBuffer; //!< Corrected image buffer
        bool mUpdated; //!< Flag set to true if a new grab is available
        mutable std::mutex mMutex; //!< Mutex to prevent concurrent read/write of mBuffer

        // Base caracteristics of the source
        std::string mName;
        unsigned int mWidth, mHeight;
        unsigned int mChannels;
        unsigned int mFramerate;

        float mExposureTime, mExposureParam; // Both are not necessarily identical
        float mAperture;
        float mGain;
        float mISO;
        float mGamma;

        unsigned int mSubsourceNbr;
        unsigned int mId;
        
        bool mHdriActive;

        // Base methods for any type of source
        void setBaseParameter(atom::Message pParam);
        atom::Message getBaseParameter(atom::Message pParam) const;

    private:
        static std::string mClassName; //!< Class name, to be set in child class
        static std::string mDocumentation; //!< Class documentation, to be set in child class

        // Mask
        cv::Mat mMask;
        
        // Noise reduction parameter
        bool mFilterNoise;

        // Basic geometric correction parameters
        float mScale;
        float mRotation;

        // Distorsion parameters
        bool mCorrectDistortion; //!< Flag set if distortion correction is activated
        bool mCorrectFisheye; //!< Flag set if fisheye correction is activated
        bool mCorrectVignetting; //!< Flag set if vignetting correction is activated
        struct OpticalDesc 
        {
            cv::Vec3f distortion;
            cv::Vec2f fisheye;
            cv::Vec3f vignetting;
        } mOpticalDesc; //!< Struct which contains correction data

        cv::Mat mVignettingMat;
        cv::Mat mDistortionMat;
        cv::Mat mFisheyeMat;
        bool mRecomputeVignettingMat;
        bool mRecomputeDistortionMat;
        bool mRecomputeFisheyeMat;

        // HDRi builder
        HdriBuilder mHdriBuilder;
        float mHdriStartExposure, mHdriStepSize;
        int mHdriSteps;

        // Color correction
        cmsHTRANSFORM mICCTransform;

        // Auto exposure
        cv::Rect mAutoExposureRoi;
        float mAutoExposureTarget, mAutoExposureThreshold, mAutoExposureStep;

        // File saving
        bool mSaveToFile;
        std::string mBaseFilename;
        int mSavePeriod;

        /************/
        // Methods
        /************/
        float getEV();

        // Mask
        void applyMask(cv::Mat& pImg);

        // Noise correction
        void filterNoise(cv::Mat& pImg);

        // Basic geometric corrections
        void scale(cv::Mat& pImg);
        void rotate(cv::Mat& pImg);

        // Methods to correct the optical distortion
        void correctVignetting(cv::Mat& pImg);
        void correctDistortion(cv::Mat& pImg);
        void correctFisheye(cv::Mat& pImg);

        // Method related to colorimetry. Default output profile is sRGB
        cmsHTRANSFORM loadICCTransform(std::string pFile);

        // Method to apply a ROI auto exposure (to override in-camera exposure)
        void applyAutoExposure(cv::Mat& pImg);

        // Method to create a HDRI from LDRIs
        void createHdri(cv::Mat& pImg);

        // Method to save the result to an image
        void saveToFile(cv::Mat& pImg);
};

#endif // SOURCE_H
