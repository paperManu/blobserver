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
 * @hdribuilder.h
 * HdriBuilder, a class to build HDR images.
 */

#ifndef HDRIBUILDER_H
#define HDRIBUILDER_H

#include <opencv2/opencv.hpp>

struct LDRi
{
    cv::Mat image;
    float EV;
};

class HdriBuilder
{
    public:
        HdriBuilder();
        ~HdriBuilder();
    
        void setContinuous(bool val) {mContinuous = val;}

        // Adds an LDR image to the list
        // LDRi must be of type RGB8u
        bool addLDR(const cv::Mat& pImage, float pEV);

        // Retrieves the HDRI
        // To call after the HDRI generation
        cv::Mat getHDRI() const;
    
        // Generate the HDRI
        // Empties the LDRi list
        bool computeHDRI();
    
    private:
        /*****************/
        // Attributes
        // If mContinous is set to true, LDRi list is never cleared
        bool mContinuous;

        // LDR images list
        std::vector<LDRi> mLDRi;
    
        // Computed HDRi
        cv::Mat mHDRi;
    
        // Minimum sum used in the HDRi computation
        float mMinSum;
    
        // Images with highest and lowest exposures
        unsigned int mMinExposureIndex;
        unsigned int mMaxExposureIndex;

        // LUT to convert between LDR value to a gaussian coeff
        std::map<uchar, float> mGaussianLUT;
    
        /****************/
        // Methods
        // Returns the coefficient to apply to a 8u value
        // according to a gaussian curve centered on 127
        float getGaussian(unsigned char pValue) const;
};

#endif // HDRIBUILDER_H
