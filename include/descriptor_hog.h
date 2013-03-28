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
 * switcher is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with switcher.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * @descriptor_hog.h
 * A class to create a history of oriented gradients (HOG) descriptor
 * for use with a classifier. This is an implementation of the HOG
 * descriptor from Dalal et Triggs article [Dalal et al. 2005]:
 * "Histograms of Oriented Gradients for Human Detection".
 */

#include <opencv2/opencv.hpp>

#include "config.h"

using namespace std;

/*************/
//! Class designed to output a HOG descriptor for a given input image
class Descriptor_Hog
{
    public:
        /**
         * Constructor
         */
        Descriptor_Hog();

        /**
         * Gets the descriptor of the specified image, with the given cropping parameters
         * if they are set. Otherwise, default parameters set with setCrop() are used.
         * \param pImage Image for which to create a descriptor
         * \param pCropRect Rectangle defining the region to crop
         * \return A vector of floats containing all components of the descriptor
         */
        vector<float> getDescriptor(const cv::Mat& pImage, const cv::Rect_<int> pCropRect = cv::Rect_<int>()) const;

        /**
         * Sets the default crop and margin to use
         * \param pCropH Horizontal size of the crop
         * \param pCropV Vertical size of the crop
         * \param pTransH Horizontal translation of the crop
         * \param pTransV Horizontal translation of the crop
         * \param pMargin Margin to consider inside the cropped selection
         */
        void setRoi(cv::Rect_<int> pCropRect, uint pMargin = 0);

        /**
         * Specifies various parameters for the histogram of oriented gradients creation.
         * See [Dalal et al. 2005] for more information about each one of them.
         * \param pBlockSize Number of cells per block, in both dimensions (default: 2x2)
         * \param pCellSize Number of pixels per cell, in both dimensions (default: 8x8)
         * \param pBinsPerCell Number of bin (corresponding to orientations) stored in each cell (default: 9)
         * \param pSigned Set to true if signed gradients are desired, false otherwise (default: false)
         */
        void setHogParams(const cv::Size_<uint> pBlockSize, const cv::Size_<uint> pCellSize, const unsigned int pBinsPerCell, const bool pSigned = false);

    private:
        // Crop parameters
        bool _doCrop;
        cv::Rect_<int> _cropRect;
        uint _margin;

        // R-HOG parameters (see [Dalal et al. 2005])
        cv::Size_<int> _blockSize;
        cv::Size_<int> _cellSize;
        uint _binsPerCell;
        bool _signed;
};

/*************/
Descriptor_Hog::Descriptor_Hog():
    _doCrop(false),
    _margin(0)
{
    _blockSize.width = 2;
    _blockSize.height = 2;
    _cellSize.width = 8;
    _cellSize.height = 8;
    _binsPerCell = 9;
    _signed = false;
}

/*************/
vector<float> Descriptor_Hog::getDescriptor(const cv::Mat& pImage, cv::Rect_<int> pCropRect) const
{
    vector<float> descriptor;

    cv::Mat roi;
    if (pCropRect.width && pCropRect.height)
        roi = cv::Mat(pImage, pCropRect);
    else if (_doCrop)
        roi = cv::Mat(pImage, _cropRect);
    else
        roi = pImage;

    cv::Mat gradientsH(roi.rows, roi.cols, pImage.type());
    cv::Mat gradientsV(roi.rows, roi.cols, pImage.type());
    uint cn = CV_MAT_CN(roi.type());

    // Get the horizontal and vertical gradients
    cv::Mat kernelH(1, 3, CV_32FC1);
    kernelH.at<float>(0, 0) = -1.f;
    kernelH.at<float>(0, 1) = 0.f;
    kernelH.at<float>(0, 2) = 1.f;
    cv::Mat kernelV = kernelH.t();

    cv::filter2D(roi, gradientsH, -1, kernelH);
    cv::filter2D(roi, gradientsV, -1, kernelV);

    // Compute the oriented gradient for each pixel
    for (uint x = 0; x < roi.cols; ++x)
    {
        for (uint y = 0; y < roi.rows; ++y)
        {
        }
    }

    // For each cell
    uint cNbrH = (roi.cols - 2*_margin) % _cellSize.width;
    uint cNbrV = (roi.rows - 2*_margin) % _cellSize.height;
    for (uint indexH = 0; indexH < cNbrH; ++indexH)
    {
        for (uint indexV = 0; indexV < cNbrV; ++indexV)
        {
        }
    }

    return descriptor;
}

/*************/
void Descriptor_Hog::setRoi(cv::Rect_<int> pCropRect, uint pMargin)
{
    if (pCropRect.height && pCropRect.width)
    {
        _doCrop = true;
        _cropRect = pCropRect;
        _margin = pMargin;
    }
}

/*************/
void Descriptor_Hog::setHogParams(const cv::Size_<uint> pBlockSize, const cv::Size_<uint> pCellSize, const unsigned int pBinsPerCell, const bool pSigned)
{
}
