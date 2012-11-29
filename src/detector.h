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
 * @detector.h
 * The detector base class.
 */

#ifndef DETECTOR_H
#define DETECTOR_H

#include "atom/message.h"
#include "opencv2/opencv.hpp"

#include "blob.h"

class Detector
{
    public:
        Detector();
        Detector(int pParam);

        static std::string getClassName() {return mClassName;}
        static std::string getDocumentation() {return mDocumentation;}

        /* Detects objects in the capture given as a parameter, and returns
         * a message with informations about each blob
         * The two first values in the message are the number of blob,
         * and the size of each blob in the message
         */
        virtual atom::Message detect(std::vector<cv::Mat> pCaptures) {}
        void setMask(cv::Mat pMask);

        virtual void setParameter(atom::Message pParam) {}
        atom::Message getParameter(atom::Message pParam);
        
        std::string getName() {return mName;}
        std::string getOscPath() {return mOscPath;}
        unsigned int getSourceNbr() {return mSourceNbr;}
        cv::Mat getOutput() {return mOutputBuffer.clone();}

    protected:
        cv::Mat mOutputBuffer;
        std::string mOscPath;
        std::string mName;
        unsigned int mSourceNbr;

        cv::Mat getMask(cv::Mat pCapture, int pInterpolation = CV_INTER_NN);

    private:
        static std::string mClassName;
        static std::string mDocumentation;

        cv::Mat mSourceMask, mMask;
};

// Useful functions
// trackBlobs is used to keep track of blobs through frames
cv::Mat getLeastSumConfiguration(cv::Mat* pDistances);
cv::Mat getLeastSumForLevel(cv::Mat pConfig, cv::Mat* pDistances, int pLevel, cv::Mat pAttributed, float &pSum, int pShift);

template<class T> void trackBlobs(std::vector<Blob::properties> &pProperties, std::vector<T> &pBlobs)
{
    // First we update all the previous blobs we detected,
    // and keep their predicted new position
    for(int i = 0; i < pBlobs.size(); ++i)
        pBlobs[i].predict();
    
    // Then we compare all these prediction with real measures and
    // associate them together
    cv::Mat lConfiguration;
    if(pBlobs.size() != 0)
    {
        cv::Mat lTrackMat = cv::Mat::zeros(pProperties.size(), pBlobs.size(), CV_32F);

        // Compute the squared distance between all new blobs, and all tracked ones
        for(int i = 0; i < pProperties.size(); ++i)
        {
            for(int j = 0; j < pBlobs.size(); ++j)
            {
                Blob::properties properties = pProperties[i];
                lTrackMat.at<float>(i, j) = pBlobs[j].getDistanceFromPrediction(properties);
            }
        }

        // We associate each tracked blobs with the fittest blob, using a least square approach
        lConfiguration = getLeastSumConfiguration(&lTrackMat);
    }

    cv::Mat lAttributedKeypoints = cv::Mat::zeros(pProperties.size(), 1, CV_8U);
    typename std::vector<T>::iterator lBlob = pBlobs.begin();
    // We update the blobs which we were able to track
    for(int i = 0; i < lConfiguration.rows; ++i)
    {
        int lIndex = lConfiguration.at<uchar>(i);
        if(lIndex < 255)
        {
            lBlob->setNewMeasures(pProperties[lIndex]);
            lBlob++;
            lAttributedKeypoints.at<uchar>(lIndex) = 255;
        }
    }
    // We delete the blobs we couldn't track
    //for(lBlob = mLightBlobs.begin(); lBlob != mLightBlobs.end(); lBlob++)
    for(int i = 0; i < lConfiguration.rows; ++i)
    {
        int lIndex = lConfiguration.at<uchar>(i);
        if(lIndex == 255)
        {
            pBlobs.erase(pBlobs.begin()+i);
        }
    }
    // And we create new blobs for the new objects detected
    for(int i = 0; i < lAttributedKeypoints.rows; ++i)
    {
        int lIndex = lAttributedKeypoints.at<uchar>(i);
        if(lIndex == 0)
        {
            T lNewBlob;
            lNewBlob.init(pProperties[i]);
            pBlobs.push_back(lNewBlob);
        }
    }
}

#endif // DETECTOR_H
