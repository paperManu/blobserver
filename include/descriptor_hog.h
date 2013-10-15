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

#ifndef DESCRIPTOR_HOG_H
#define DESCRIPTOR_HOG_H

#include <opencv2/opencv.hpp>

#include "config.h"

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
        std::vector<float> getDescriptor(const cv::Point_<int> pPos = cv::Point_<int>()) const;

        /**
         * Sets the default crop and margin to use
         * \param pCropH Horizontal size of the crop
         * \param pCropV Vertical size of the crop
         * \param pTransH Horizontal translation of the crop
         * \param pTransV Horizontal translation of the crop
         */
        void setRoi(const cv::Rect_<int> pCropRect);

        /**
         * Specifies various parameters for the histogram of oriented gradients creation.
         * See [Dalal et al. 2005] for more information about each one of them.
         * \param pDescriptorSize Size (in pixels) of a descriptor (default: 64x128)
         * \param pBlockSize Number of cells per block, in both dimensions (default: 3x3)
         * \param pCellSize Number of pixels per cell, in both dimensions (default: 8x8)
         * \param pBinsPerCell Number of bin (corresponding to orientations) stored in each cell (default: 9)
         * \param pSigned Set to true if signed gradients are desired, false otherwise (default: false)
         * \param pNorm Set the norm to use for block normalization (default: L2_NORM)
         * \param pSigma Set the sigma parameter for the Gauss curve applied over the blocks during normalization (default: 1.0)
         */
        void setHogParams(const cv::Size_<int> pDescriptorSize, const cv::Size_<int> pBlockSize, const cv::Size_<int> pCellSize,
            const unsigned int pBinsPerCell, const bool pSigned = false, const Hog_Norm pNorm = L2_NORM, const float pSigma = 1.f);

        /**
         */
        void setMultiscaleParams(const cv::Size_<int> pCellMinSize, const cv::Size_<int> pCellMaxSize, const cv::Size_<float> pCellStep);

    private:
        // Input image
        cv::Mat _image;
        // Gradients orientation and value image
        cv::Mat _gradients;

        // Crop parameters
        bool _doCrop;
        cv::Rect_<int> _cropRect;

        // R-HOG parameters (see [Dalal et al. 2005])
        cv::Size_<int> _roiSize;
        cv::Size_<int> _blockSize;
        cv::Size_<int> _cellSize;
        int _binsPerCell;
        bool _signed;
        Hog_Norm _normType;
        float _gaussSigma;

        // Multiscale parameters
        cv::Size_<int> _cellMinSize, _cellMaxSize;
        cv::Size_<float> _cellStep;

        // Constant attributes
        float _epsilon; // A small value, used for normalization.
        cv::Mat _kernelH, _kernelV;

        // Computes a single scale descriptor
        std::vector<float> getSingleScaleDescriptor(cv::Point_<int> pPos, const cv::Size_<int> pCellSize) const;

        // Method to compute the norm of a descriptor
        float getDescriptorNorm(std::vector<float> pDescriptor) const;

        // Centered normal distribution
        float getGaussian(const float x, const float sigma) const;
};

#endif // DESCRIPTOR_HOG_H
