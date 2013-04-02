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
        enum Hog_Norm {
            L1_NORM = 0,
            L2_NORM
        };

    public:
        /**
         * Constructor
         */
        Descriptor_Hog();

        /**
         * Specifies the image from which to create the descriptor(s).
         * Also, computes the oriented gradient at each pixel.
         * \param pImage Image
         */
        void setImage(const cv::Mat& pImage);

        /**
         * Gets the descriptor of the specified image, with the given cropping parameters
         * if they are set. Otherwise, default parameters set with setCrop() are used.
         * \param pPos Position of the region (defined by the hog parameters) from where to create the classifier
         * \return A vector of floats containing all components of the descriptor. Size is 0 if there was not enough space to build the whole descriptor
         */
        vector<float> getDescriptor(const cv::Point_<int> pPos = cv::Point_<int>()) const;

        /**
         * Sets the default crop and margin to use
         * \param pCropH Horizontal size of the crop
         * \param pCropV Vertical size of the crop
         * \param pTransH Horizontal translation of the crop
         * \param pTransV Horizontal translation of the crop
         * \param pMargin Margin to consider inside the cropped selection
         */
        void setRoi(cv::Rect_<int> pCropRect, int pMargin = 0);

        /**
         * Specifies various parameters for the histogram of oriented gradients creation.
         * See [Dalal et al. 2005] for more information about each one of them.
         * \param pDescriptorSize Size (in pixels) of a descriptor (default: 64x128)
         * \param pBlockSize Number of cells per block, in both dimensions (default: 3x3)
         * \param pCellSize Number of pixels per cell, in both dimensions (default: 8x8)
         * \param pBinsPerCell Number of bin (corresponding to orientations) stored in each cell (default: 9)
         * \param pSigned Set to true if signed gradients are desired, false otherwise (default: false)
         * \param pNorm Set the norm to use for block normalization (default: L2_NORM)
         */
        void setHogParams(const cv::Size_<int> pDescriptorSize, const cv::Size_<int> pBlockSize, const cv::Size_<int> pCellSize,
            const unsigned int pBinsPerCell, const bool pSigned = false, const Hog_Norm pNorm = L2_NORM);

    private:
        // Input image
        cv::Mat _image;
        // Gradients orientation and value image
        cv::Mat _gradients;

        // Crop parameters
        bool _doCrop;
        cv::Rect_<int> _cropRect;
        int _margin;

        // R-HOG parameters (see [Dalal et al. 2005])
        cv::Size_<int> _descriptorSize;
        cv::Size_<int> _blockSize;
        cv::Size_<int> _cellSize;
        int _binsPerCell;
        bool _signed;
        Hog_Norm _normType;

        float _epsilon; // A small value, used for normalization.

        // Method to compute the norm of a descriptor
        float getDescriptorNorm(vector<float> pDescriptor) const;
};

/*************/
Descriptor_Hog::Descriptor_Hog():
    _doCrop(false)
{
    _descriptorSize.width = 8;
    _descriptorSize.height = 16;
    _blockSize.width = 3;
    _blockSize.height = 3;
    _cellSize.width = 8;
    _cellSize.height = 8;
    _binsPerCell = 9;
    _signed = false;
    _normType = L1_NORM;

    _margin = max(_cellSize.width, _cellSize.height) * max(_blockSize.width/2, _blockSize.height/2);

    _epsilon = FLT_EPSILON;
}

/*************/
void Descriptor_Hog::setImage(const cv::Mat& pImage)
{
    cv::Mat tmpImage = pImage.clone();

    if (_doCrop)
        _image = cv::Mat(tmpImage, _cropRect);
    else
        _image = tmpImage;

    cv::Mat gradientsH(_image.rows, _image.cols, pImage.type());
    cv::Mat gradientsV(_image.rows, _image.cols, pImage.type());
    int cn = CV_MAT_CN(_image.type());

    // Get the horizontal and vertical gradients
    cv::Mat kernelH(1, 3, CV_32FC1);
    kernelH.at<float>(0, 0) = -1.f;
    kernelH.at<float>(0, 1) = 0.f;
    kernelH.at<float>(0, 2) = 1.f;
    cv::Mat kernelV = kernelH.t();

    cv::filter2D(_image, gradientsH, CV_32F, kernelH);
    cv::filter2D(_image, gradientsV, CV_32F, kernelV);

    // Compute the oriented gradient for each pixel
    cv::Mat channelsH[cn];
    cv::Mat channelsV[cn];
    cv::split(gradientsH, channelsH);
    cv::split(gradientsV, channelsV);

    _gradients = cv::Mat(_image.rows, _image.cols, CV_8UC2);
    int rowLength = tmpImage.cols;
    for (int x = 0; x < _image.cols; ++x)
    {
        for (int y = 0; y < _image.rows; ++y)
        {
            float hValue = 0.f;
            float vValue = 0.f;
            float length = 0.f;
            // We get the maximum gradient over the image
            for (int i = 0; i < cn; ++i)
            {
                float cnHValue = channelsH[i].at<float>(y, x);
                float cnVValue = channelsV[i].at<float>(y, x);
                float cnLength = sqrtf(cnHValue*cnHValue + cnVValue*cnVValue);

                if (cnLength > length)
                {
                    hValue = cnHValue;
                    vValue = cnVValue;
                    length = cnLength;
                }
            }

            float angle = 0.f;
            if (length > 0.f)
            {
                hValue /= length;
                angle = acos(hValue);
            }

            if (_signed && vValue < 0.f)
            {
                angle = 2*CV_PI - angle;
                angle = (int)(angle / CV_PI * 180) % 360;
            }
            else
            {
                angle = (int)(angle / CV_PI * 180) % 180;
            }

            _gradients.at<cv::Vec2b>(y, x)[0] = (char)angle;
            _gradients.at<cv::Vec2b>(y, x)[1] = (char)length;
        }
    }
}

/*************/
vector<float> Descriptor_Hog::getDescriptor(cv::Point_<int> pPos) const
{
    vector<float> descriptor;

    // Real position of the ROI, taking account for the margin
    cv::Point_<int> pos = pPos - cv::Point_<int>(_margin, _margin);
    // Angle covered per bin
    float anglePerBin = 180.f / (float)_binsPerCell;
    if (_signed)
        anglePerBin *= 2.f;

    // Check if we have enough room to build a complete descriptor
    int roiSizeH = _descriptorSize.width * _cellSize.width + _margin * 2;
    int roiSizeV = _descriptorSize.height * _cellSize.height + _margin * 2;
    if (pos.x + roiSizeH >= _image.cols || pos.y + roiSizeV >= _image.rows)
        return descriptor;
    // If position is too close to the border (no margin), we don't have enough room either
    if (pos.x < 0 || pos.y < 0)
        return descriptor;
    
    // For each cell, we compute its descriptor
    int windowH = _descriptorSize.width + (_blockSize.width-1); // (_blockSize-1) is added to handle margin
    int windowV = _descriptorSize.height + (_blockSize.height-1); // (_blockSize-1) is added to handle margin
    vector< vector<float> > cellsDescriptor;
    for (int cellH = 0; cellH < windowH; ++cellH)
        for (int cellV = 0; cellV < windowV; ++cellV)
        {
            cv::Point_<int> topLeft;
            topLeft.x = cellH * _cellSize.width + pos.x;
            topLeft.y = cellV * _cellSize.height + pos.y;
            
            // Creation of the histogram
            vector<float> cellDescriptor;
            cellDescriptor.assign(_binsPerCell, 0);
            cv::Mat hist = cv::Mat::zeros(1, _binsPerCell, CV_16UC1);
            for (int x = 0; x < _cellSize.width; ++x)
                for (int y = 0; y < _cellSize.height; ++y)
                {
                    int index = _gradients.at<cv::Vec2b>(topLeft.y + y, topLeft.x + x)[0] / anglePerBin;
                    float subPos = _gradients.at<cv::Vec2b>(topLeft.y + y, topLeft.x + x)[0] - index*anglePerBin;
                    int shift = (subPos < anglePerBin/2.f) ? -1 : 1;
                    if (shift + index < 0)
                        shift = _binsPerCell-1;
                    else if (shift + index >= _binsPerCell)
                        shift = 0;
                    else
                        shift += index;

                    float ratio = abs(subPos - anglePerBin/2.f)/anglePerBin;
                    cellDescriptor[index] += _gradients.at<cv::Vec2b>(topLeft.y + y, topLeft.x + x)[1] * (1.f - ratio);
                    cellDescriptor[shift] += _gradients.at<cv::Vec2b>(topLeft.y + y, topLeft.x + x)[1] * ratio;
                }

            cellsDescriptor.push_back(cellDescriptor);
        }

    // We have all cells descriptors. Now we normalize them to create the global descriptor
    // TODO: include a Gaussian spatial window over the block
    for (int cellH = 0; cellH < _descriptorSize.width; ++cellH)
        for (int cellV = 0; cellV < _descriptorSize.height; ++cellV)
        {
            int blockCenterH = cellH + _blockSize.width/2;
            int blockCenterV = cellV + _blockSize.height/2;

            vector<float> descriptorVector;
            descriptorVector.assign(_binsPerCell, 0);
            // We calculate the norm of the descriptor over the whole block
            // So we go over all the cells for the current block
            for (int i = -_blockSize.width/2; i < _blockSize.width/2; ++i)
                for (int j = -_blockSize.height/2; j < _blockSize.height/2; ++j)
                {
                    int index = blockCenterH+i + (blockCenterH+j)*windowV;
                    for (int orientation = 0; orientation < _binsPerCell; ++orientation)
                    {
                        descriptorVector[orientation] += cellsDescriptor[index][orientation];
                    }
                }

            // We need the norm for the current block descriptor
            float norm = getDescriptorNorm(descriptorVector);

            // And we scale the whole block, to get the descriptor for the current cell
            for (int i = 0; i < _binsPerCell; ++i)
                descriptor.push_back(descriptorVector[i] / norm);
        }

    return descriptor;
}

/*************/
void Descriptor_Hog::setRoi(cv::Rect_<int> pCropRect, int pMargin)
{
    if (pCropRect.height && pCropRect.width)
    {
        _doCrop = true;
        _cropRect = pCropRect;
        // We need enough margin to normalize descriptors at the border of the ROI
        _margin = max(pMargin, max(_cellSize.width, _cellSize.height)) * max(_blockSize.width/2, _blockSize.height/2);
    }
}

/*************/
void Descriptor_Hog::setHogParams(const cv::Size_<int> pDescriptorSize, const cv::Size_<int> pBlockSize, const cv::Size_<int> pCellSize,
    const unsigned int pBinsPerCell, const bool pSigned, const Hog_Norm pNorm)
{
    if (pDescriptorSize == cv::Size_<int>(0, 0) ||
        pBlockSize == cv::Size_<int>(0, 0) ||
        pCellSize == cv::Size_<int>(0, 0))
    {
        return;
    }

    _blockSize.width = (pBlockSize.width / 2)*2 + 1; // We ensure that we have odd values
    _blockSize.height = (pBlockSize.height / 2)*2 + 1; // Same here
    _cellSize = pCellSize;
    // We calculate the size of the descriptor (in cells), lower rounded
    _descriptorSize.width = pDescriptorSize.width / _cellSize.width;
    _descriptorSize.height = pDescriptorSize.height / _cellSize.height;

    _binsPerCell = pBinsPerCell;
    _signed = pSigned;
    _normType = pNorm;
}

/*************/
float Descriptor_Hog::getDescriptorNorm(vector<float> pDescriptor) const
{
    float norm;

    if (_normType == L1_NORM)
    {
        norm = _epsilon;
        for (int i = 0; i < _binsPerCell; ++i)
            norm += abs(pDescriptor[i]);
    }
    else if (_normType == L2_NORM)
    {
        norm = _epsilon;
        for (int i = 0; i < _binsPerCell; ++i)
            norm += pow(pDescriptor[i], 2.f);

        norm = sqrtf(norm);
    }

    return norm;
}
