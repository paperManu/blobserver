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
 * @helpers.h
 * Function to help handling of various things
 */

#ifndef HELPERS_H
#define HELPERS_H

#include "opencv2/opencv.hpp"
#include "atom/message.h"

#include "blob.h"

/*************/
// Class for parallel masking
template <typename PixType>
class Parallel_Mask : public cv::ParallelLoopBody
{
    public:
        Parallel_Mask(cv::Mat* buffer, cv::Mat* mask):
            _buffer(buffer), _mask(mask) {}

        void operator()(const cv::Range& r) const
        {
            PixType* buffer = &(_buffer->at<PixType>(r.start, 0));
            PixType* mask = &(_mask->at<uchar>(r.start, 0));
            for (int y = r.start; y != r.end; ++y, buffer += _buffer->cols*sizeof(PixType), mask += _mask->cols*sizeof(uchar))
            {
                for (int x = 0; x < _buffer->cols; ++x)
                {
                    if (*(mask + x*sizeof(uchar)) == 0)
                        *(buffer + x*sizeof(PixType)) = PixType(0);
                }
            }
        }

    private:
        cv::Mat* _buffer;
        cv::Mat* _mask;
};

/*************/
// Function to read a value from a message
template<class T>
bool readParam(const atom::Message pParam, T& pValue, int pIndex = 1)
{
    if (pParam.size() < pIndex+1)
        return false;

    auto tag = pParam[pIndex].get()->getTypeTag();
    if (tag == atom::FloatValue::TYPE_TAG)
    {
        if (typeid(pValue) != typeid(float))
            return false;
        float value;
        value = atom::FloatValue::convert(pParam[pIndex])->getFloat();
        reinterpret_cast<float&>(pValue) = value;
    }
    else if (tag == atom::IntValue::TYPE_TAG)
    {
        if (typeid(pValue) != typeid(int))
            return false;
        int value;
        value = atom::IntValue::convert(pParam[pIndex])->getInt();
        reinterpret_cast<int&>(pValue) = value;
    }
    else if (tag == atom::StringValue::TYPE_TAG)
    {
        if (typeid(pValue) != typeid(std::string))
            return false;
        std::string value;
        value = atom::StringValue::convert(pParam[pIndex])->getString();
        reinterpret_cast<std::string&>(pValue) = value;
    }
    else
        return false;

    return true;
}

/*************/
// Useful functions
// trackBlobs is used to keep track of blobs through frames
/*************/
template<class T>
class BlobPair
{
    public:
        BlobPair(T* current, Blob::properties* measure):
            _current(current), _measure(measure)
        {
            _dist = _current->getDistanceFromPrediction(*_measure);
        }

        T* getCurrent() const {return _current;}
        Blob::properties* getMeasure() const {return _measure;}

        bool operator< (const BlobPair<T>& second) const
        {
            if (this->_dist > second.getDist())
                return true;
            else
                return false;
        }

        float getDist() const {return _dist;}

    private:
        T* _current;
        Blob::properties* _measure;
        float _dist;
};

/*************/
template<class T>
void trackBlobs(std::vector<Blob::properties> &pProperties, std::vector<T> &pBlobs, int pLifetime = 30,
                int pKeepOldBlobs = 0, int pKeepMaxTime = 0, float pMaxDistanceToLink = 0.f, float pOcclusionDistance = 0.f)
{
    // First we update all the previous blobs we detected,
    // and keep their predicted new position
    for(int i = 0; i < pBlobs.size(); ++i)
        if (pBlobs[i].getLifetime() > 0)
            pBlobs[i].predict();

    // Then we compare all these prediction with real measures and
    // associate them together
    std::vector< BlobPair<T> > lPairs;
    if(pBlobs.size() != 0)
    {
        std::vector<BlobPair<T> > lSearchPairs;

        // Compute the squared distance between all new blobs, and all tracked ones
        for (int i = 0; i < pProperties.size(); ++i)
        {
            for (int j = 0; j < pBlobs.size(); ++j)
            {
                BlobPair<T> lPair(&pBlobs[j], &pProperties[i]);
                lSearchPairs.push_back(lPair);
            }
        }

        // We loop through the pairs to find the closest ones
        while (lSearchPairs.size())
        {
            std::make_heap(lSearchPairs.begin(), lSearchPairs.end());
            // Get the nearest new blob
            std::pop_heap(lSearchPairs.begin(), lSearchPairs.end());
            BlobPair<T> nearest = lSearchPairs.back();
            lSearchPairs.pop_back();

            // If the distance is higher than maxDistanceToLink, we don't consider that these
            // blobs match. Indeed, no one matches the existing blob
            if (pMaxDistanceToLink == 0.f || nearest.getDist() <= pMaxDistanceToLink)
                lPairs.push_back(nearest);

            // Delete pairs with the same current blob
            // as well as pairs with the same new blob
            for (int j = 0; j < lSearchPairs.size();)
            {
                if (lSearchPairs[j].getCurrent() == nearest.getCurrent())
                    lSearchPairs.erase(lSearchPairs.begin() + j);
                else if (lSearchPairs[j].getMeasure() == nearest.getMeasure())
                    lSearchPairs.erase(lSearchPairs.begin() + j);
                else
                    j++;
            }
        }
    }

    // We update the blobs which we were able to track
    for (int i = 0; i < lPairs.size(); ++i)
    {
        lPairs[i].getCurrent()->setNewMeasures(*(lPairs[i].getMeasure()));
        lPairs[i].getCurrent()->renewLifetime();
    }

    // We delete the blobs we were not able to track
    // (unless we want to keep all blobs)
    for (int i = 0; i < pBlobs.size();)
    {
        bool isIn = false;
        for (int j = 0; j < lPairs.size(); ++j)
        {
            if (lPairs[j].getCurrent() == &pBlobs[i])
                isIn = true;
        }

        if (pBlobs[i].getLifetime() > 0)
            pBlobs[i].getOlder();

        if (!isIn)
        {
            pBlobs[i].reduceLifetime();
            if (pBlobs[i].getLifetime() < 0 && (pBlobs[i].getAge() < pKeepOldBlobs || pKeepOldBlobs == 0)
                || pBlobs[i].getLostDuration() > pKeepMaxTime && pKeepMaxTime > 0)
            {
                pBlobs.erase(pBlobs.begin() + i);
            }
            else
                i++;
        }
        else
            i++;
    }

    // Also, we extend lifetime of blobs which may become occluded
    if (pOcclusionDistance > 0.f)
    {
        std::vector<BlobPair<T>> lOcclusionPairs;

        for (int i = 0; i < pBlobs.size(); ++i)
        {
            pBlobs[i].setUnoccluded();
            for (int j = 0; j < pBlobs.size(); ++j)
            {
                if (i == j)
                    continue;

                Blob::properties props = pBlobs[j].getBlob();
                BlobPair<T> lPair(&pBlobs[i], &props);
                lOcclusionPairs.push_back(lPair);
            }
        }

        while (lOcclusionPairs.size())
        {
            std::make_heap(lOcclusionPairs.begin(), lOcclusionPairs.end());
            std::pop_heap(lOcclusionPairs.begin(), lOcclusionPairs.end());
            BlobPair<T> nearest = lOcclusionPairs.back();
            lOcclusionPairs.pop_back();

            if (nearest.getDist() <= pOcclusionDistance && nearest.getCurrent()->getAge() > pLifetime * 2)
            {
                nearest.getCurrent()->renewLifetime();
                nearest.getCurrent()->setOccluded();
            }
            else
                break;
        }
    }

    // And we create new blobs for the new objects detected
    for (int i = 0; i < pProperties.size(); ++i)
    {
        bool isIn = false;
        for (int j = 0; j < lPairs.size(); ++j)
        {
            if (lPairs[j].getMeasure() == &pProperties[i])
                isIn = true;
        }

        if (!isIn)
        {
            T newBlob;
            newBlob.init(pProperties[i]);
            newBlob.setLifetime(pLifetime);
            pBlobs.push_back(newBlob);
        }
    }
}


#endif // HELPERS_H
