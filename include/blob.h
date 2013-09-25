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

/**
 * @blob.h
 * The blob base class.
 */

#ifndef BLOB_H
#define BLOB_H

#include "opencv2/opencv.hpp"

class Blob
{
    public:
    // A struct to describe blobs
    struct properties
    {
        cv::Point2f position;
        cv::Point2f speed;
        cv::Mat colorHist;
        float orientation;
        float size;
    };

    public:
        Blob();
        ~Blob();

        int getId() {return mId;};
        virtual void setParameter(std::string pParam, float pValue) {};

        virtual void init(properties pNewBlob) {};
        virtual properties predict() = 0;
        virtual void setNewMeasures(properties pNewBlob) {};
        virtual float getDistanceFromPrediction(properties pBlob) = 0;
        
        // Lifetime is linked to the time left for the blob to live if not detected again
        void setLifetime(int time) {mTotalLifetime = mLifetime = time;}
        void renewLifetime()
        {
            if (mLifetime < 0)
                mLostDuration = 0;
            mLifetime = mTotalLifetime;
        }
        void reduceLifetime()
        {
            if (mLifetime < 0)
                mLostDuration++;
            mLifetime--;
        }
        int getLifetime() const {return mLifetime;}
        // Age is the total time the blob has been there
        void getOlder() {mAge++;}
        unsigned long getAge() const {return mAge;}
        unsigned long getLostDuration() const {return mLostDuration;}

        properties getBlob();
        bool isUpdated();

    protected:
        bool updated;
        
        int mTotalLifetime;
        int mLifetime;
        unsigned long mAge; // The age of the blob, not counting time it was lost
        unsigned long mLostDuration;
        properties mProperties;
        properties mPrediction;

        cv::KalmanFilter mFilter;

        int mId;
};

#endif // BLOB_H
