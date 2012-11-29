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
        cv::Point position;
        cv::Point speed;
        cv::Vec3b color;
        float orientation;
        float size;
    };

    public:
        Blob();
        ~Blob();

        int getId() {return mId;};
        virtual void setParameter(std::string pParam, float pValue) {};

        virtual void init(properties pNewBlob) {};
        virtual properties predict() {};
        virtual void setNewMeasures(properties pNewBlob) {};
        virtual float getDistanceFromPrediction(properties pBlob) {};
        
        properties getBlob();
        bool isUpdated();

    protected:
        bool updated;
        
        properties mProperties;
        properties mPrediction;

        cv::KalmanFilter mFilter;

        int mId;
};

#endif // BLOB_H
