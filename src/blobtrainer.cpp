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

/**
 * @blobtrainer.cpp
 * This small utility is used to create models needed by the SVM classifier
 * used in some actuators.
 */

#include <chrono>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <memory>
#include <vector>

#include <glib.h>
#include <opencv2/opencv.hpp>

#include "config.h"
#include "descriptor_hog.h"

using namespace std;

static gboolean gVersion = FALSE;
static gboolean gVerbose = FALSE;
static gboolean gTable = FALSE;
static gchar* gTestFile = NULL;
static gchar* gExtension = NULL;
static gchar* gPositiveDir = NULL;
static gchar* gNegativeDir = NULL;
static gchar* gOutput = NULL;
static double gIterations = 1e6;
static double gEpsilon = 100.0;
static double gCPenalty = 0.1;
static int gBins = 9;
static gchar* gPosition = NULL;
static gchar* gCellSize = NULL;
static gchar* gCellMaxSize = NULL;
static gchar* gCellStep = NULL;
static gchar* gBlockSize = NULL;
static gchar* gRoiSize = NULL;
static double gSigma = 0.0;
static double gMargin = 0.0;

static double gPca = 1.0;
static double gCrossValidation = 1.0;

int _svmCriteria;
cv::Point_<int> _roiPosition;
cv::Point_<int> _cellSize;
cv::Point_<int> _cellMaxSize;
cv::Point_<float> _cellStep;
cv::Point_<int> _blockSize;
cv::Point_<int> _roiSize;

static GOptionEntry gEntries[] =
{
    {"version", 'v', 0, G_OPTION_ARG_NONE, &gVersion, "Shows the version of this software", NULL},
    {"verbose", 'V', 0, G_OPTION_ARG_NONE, &gVerbose, "More verbose output", NULL},
    {"table", 'T', 0, G_OPTION_ARG_NONE, &gTable, "Table output, for testing purposes", NULL},
    {"test", 't', 0, G_OPTION_ARG_STRING, &gTestFile, "Specifies a model file to test against the given images", NULL},
    {"extension", 'f', 0, G_OPTION_ARG_STRING, &gExtension, "Specifies the extension for the files to load (default: png)", NULL},
    {"positive", 'p', 0, G_OPTION_ARG_STRING, &gPositiveDir, "Specifies the subdirectory where to load positive images (default: ./positive)", NULL},
    {"negative", 'n', 0, G_OPTION_ARG_STRING, &gNegativeDir, "Specifies the subdirectory where to load negative images (default: ./negative)", NULL},
    {"output", 'o', 0, G_OPTION_ARG_STRING, &gOutput, "Specifies the output file for saving the SVM model (default: ./model.xml)", NULL},
    {"iterations", 'i', 0, G_OPTION_ARG_DOUBLE, &gIterations, "Specifies the maximum number of iteration for the SVM training (default: 1e6)", NULL},
    {"epsilon", 'E', 0, G_OPTION_ARG_DOUBLE, &gEpsilon, "Specifies the minimum tolerance between iteration to end training (default: 1e-1, although not used if not specified in cmd line)", NULL},
    {"c-penalty", 'c', 0, G_OPTION_ARG_DOUBLE, &gCPenalty, "Specifies the penalty to give to an outlier while training (default: 0.1)", NULL},
    {"bins", 'b', 0, G_OPTION_ARG_INT, &gBins, "Specifies number of bins for the HOG descriptor (default: 9)", NULL},
    {"position", 0, 0, G_OPTION_ARG_STRING, &gPosition, "Specifies the position where to create the positives descriptors (default: '16x16')", NULL},
    {"cell-size", 0, 0, G_OPTION_ARG_STRING, &gCellSize, "Specifies the size of the cells (in pixels) of descriptors (default: '8x8')", NULL},
    {"cell-max-size", 0, 0, G_OPTION_ARG_STRING, &gCellMaxSize, "Specifies the maximum size of the cells, when using multiscale training (default: 0x0)", NULL},
    {"cell-step", 0, 0, G_OPTION_ARG_STRING, &gCellStep, "Specifies the step factor between cell size, when using multiscale training (default: '2x2'", NULL},
    {"block-size", 0, 0, G_OPTION_ARG_STRING, &gBlockSize, "Specifies the size of the blocks over which cells are normalized (default: '3x3')", NULL},
    {"roi-size", 0, 0, G_OPTION_ARG_STRING, &gRoiSize, "Specifies the size (in pixels) of the ROI from which to create descriptors (default: '64x128')", NULL},
    {"sigma", 0, 0, G_OPTION_ARG_DOUBLE, &gSigma, "Specifies the sigma parameter for the gaussian kernel applied over blocks (default: 1.0)", NULL},
    {"margin", 0, 0, G_OPTION_ARG_DOUBLE, &gMargin, "Specifies the distance to the margin for positive images to be detected as such (default: 0.0)", NULL},
    {"pca", 0, 0, G_OPTION_ARG_DOUBLE, &gPca, "Enables PCA and specifies the ratio of components to keep (default: 1.0)", NULL},
    {"crossValidation", 'C', 0, G_OPTION_ARG_DOUBLE, &gCrossValidation, "Enables cross validation if value is lower than 1.0 (default: 1.0)", NULL},
    {NULL}
};

/*************/
int parseArgs(int argc, char** argv)
{
    GError *error = NULL;
    GOptionContext* context;

    context = g_option_context_new(" - blobtrainer, a utility to train SVM classifier for blobserver");
    g_option_context_add_main_entries(context, gEntries, NULL);

    if (!g_option_context_parse(context, &argc, &argv, &error))
    {
        cout << "Error while parsing options: " << error->message << endl;
        return 1;
    }

    if (gVersion)
    {
        cout << "blobtrainer " << PACKAGE_VERSION << ", from the blobserver suite." << endl;
        return 1;
    }

    if (gExtension == NULL)
        gExtension = (gchar*)"png";

    if (gPositiveDir == NULL)
        gPositiveDir = (gchar*)"positive";

    if (gNegativeDir == NULL)
        gNegativeDir = (gchar*)"negative";

    if (gOutput == NULL)
        gOutput = (gchar*)"model.xml";

    if (gEpsilon != 100)
        _svmCriteria = CV_TERMCRIT_EPS;
    else
        _svmCriteria = CV_TERMCRIT_ITER;

    if (gPosition == NULL)
        gPosition = (gchar*)"16x16";
    sscanf(gPosition, "%ix%i", &(_roiPosition.x), &(_roiPosition.y));

    if (gCellSize == NULL)
        gCellSize = (gchar*)"8x8";
    sscanf(gCellSize, "%ix%i", &(_cellSize.x), &(_cellSize.y));

    if (gCellMaxSize == NULL)
        gCellMaxSize = (gchar*)"0x0";
    sscanf(gCellMaxSize, "%ix%i", &(_cellMaxSize.x), &(_cellMaxSize.y));

    if (gCellStep == NULL)
        gCellStep = (gchar*)"2x2";
    sscanf(gCellStep, "%fx%f", &(_cellStep.x), &(_cellStep.y));

    if (gBlockSize == NULL)
        gBlockSize = (gchar*)"3x3";
    sscanf(gBlockSize, "%ix%i", &(_blockSize.x), &(_blockSize.y));

    if (gRoiSize == NULL)
        gRoiSize = (gchar*)"64x128";
    sscanf(gRoiSize, "%ix%i", &(_roiSize.x), &(_roiSize.y));

    return 0;
}

/*************/
vector<string> loadFileList(string pDirectory)
{
    vector<string> fileList;
    GError* error;
    GDir* directory;

    directory = g_dir_open((const gchar*)(string("./") + pDirectory).c_str(), 0, &error);
    const gchar* filename;
    while ((filename = g_dir_read_name(directory)) != NULL)
    {
        char* substr = strstr((char*)filename, (char*)gExtension);
        if (substr == filename + strlen((const char*)filename) - strlen((const char*)gExtension))
        {
            string strName = string("./") + pDirectory + string("/") + string((const char*)filename);
            fileList.push_back(strName);
        }
    }

    return fileList;
}

/*************/
unsigned timeSince(unsigned long long timestamp)
{
    auto now = chrono::high_resolution_clock::now();
    unsigned long long currentTime = chrono::duration_cast<chrono::microseconds>(now.time_since_epoch()).count();
    return (long long)currentTime - (long long)timestamp;
}

/*************/
void trainSVM(vector<string>& pPositiveFiles, vector<string>& pNegativeFiles, Descriptor_Hog& pDescriptor, CvSVM& pSvm, cv::PCA& pPca, float pPortion = 1.0)
{
    int result;

    if (!gTable)
    {
        cout << "Training parameters: " << endl;
        if (_svmCriteria == CV_TERMCRIT_ITER)
            cout << "   iterations = " << gIterations << endl;
        if (_svmCriteria == CV_TERMCRIT_EPS)
            cout << "   epsilon = " << gEpsilon << endl;
        cout << "   C penalty = " << gCPenalty << endl;
        cout << "   output file = " << gOutput << endl;
        cout << "   ROI position = " << gPosition << endl;
        cout << "   cell size = " << gCellSize << endl;
        cout << "   cell max size = " << _cellMaxSize.x << "x" << _cellMaxSize.y << endl;
        cout << "   cell step = " << gCellStep << endl;
        cout << "   block size = " << gBlockSize << endl;
        cout << "   roi size = " << gRoiSize << endl;
        cout << "   bins per cell = " << gBins << endl;
        cout << "   sigma = " << gSigma << endl;
        cout << "   pca = " << gPca << endl;
    }
    
    // Set up training data
    if (!gVerbose && !gTable)
        cout << "Loading training data... " << flush;

    int nbrImg = 0;
    vector<float> labels;
    vector<float> trainingData;
    int portionOfPositiveFiles = (int)((float)pPositiveFiles.size() * pPortion);
    for (int i = 0; i < portionOfPositiveFiles; ++i)
    {
        if (gVerbose)
            cout << "Analysing " << pPositiveFiles[i] << endl;

        cv::Mat image = cv::imread(pPositiveFiles[i]);
        pDescriptor.setImage(image);
        vector<float> description = pDescriptor.getDescriptor(_roiPosition);
        if (description.size() == 0)
            continue;

        labels.push_back(1.f);
        for (int j = 0; j < description.size(); ++j)
            trainingData.push_back(description[j]);

        nbrImg++;
    }
    if (!gVerbose && !gTable)
        cout << "Positive data loaded... " << flush;

    int portionOfNegativeFiles = (int)((float)pNegativeFiles.size() * pPortion);
    for (int i = 0; i < portionOfNegativeFiles; ++i)
    {
        if (gVerbose)
            cout << "Analysing " << pNegativeFiles[i] << endl;

        cv::Mat image = cv::imread(pNegativeFiles[i]);
        pDescriptor.setImage(image);
        for (int x = 0; x <= image.cols - _roiSize.x; x += image.cols/5)
        {
            for (int y = 0; y <= image.rows - _roiSize.y; y += image.rows/5)
            {
                vector<float> description = pDescriptor.getDescriptor(_roiPosition + cv::Point_<int>(x, y));
                if (description.size() == 0)
                    continue;

                labels.push_back(-1.f);
                for (int j = 0; j < description.size(); ++j)
                    trainingData.push_back(description[j]);

                nbrImg++;
            }
        }
    }
    if (!gVerbose && !gTable)
        cout << "Negative data loaded" << endl;

    // Applying the PCA and reformating input data
    if (gPca < 1.0)
    {
        cout << "Applying PCA before SVM train... ";
        cout.flush();

        int vectorSize = trainingData.size() / nbrImg;
        cv::Mat trainingPCA = cv::Mat::zeros(nbrImg, vectorSize, CV_32F);
        for (int i = 0; i < nbrImg; ++i)
        {
            for (int j = 0; j < vectorSize; ++j)
            {
                trainingPCA.at<float>(i, j) = trainingData[i*vectorSize + j];
            }
        }
        int keptComponents = (int)((float)vectorSize * gPca);
        pPca(trainingPCA, cv::noArray(), CV_PCA_DATA_AS_ROW, keptComponents);

        // Reproject all vectors
        cv::Mat projected;
        pPca.project(trainingPCA, projected);

        // Store the result as the training data
        trainingData.clear();
        for (int i = 0; i < projected.rows; ++i)
            for (int j = 0; j < projected.cols; ++j)
            {
                trainingData.push_back(projected.at<float>(i, j));
            }

        cout << "Kept " << keptComponents << " components" << endl;

        // Save the resulting space
        cv::FileStorage file("pca.xml", cv::FileStorage::WRITE);
        file << "eigenVectors" << pPca.eigenvectors;
        file << "mean" << pPca.mean;
    }
    
    if (!gTable)
        cout << "Training the SVM classifier..." << endl;
    // Set up the SVM train parameter
    CvSVMParams svmParams;
    svmParams.svm_type = CvSVM::C_SVC;
    svmParams.C = gCPenalty;
    svmParams.kernel_type = CvSVM::LINEAR;
    svmParams.term_crit = cvTermCriteria(_svmCriteria, gIterations, gEpsilon);

    // Train the SVM
    cv::Mat labelsMat((int)labels.size(), 1, CV_32FC1, &labels[0]);
    cv::Mat trainingMat(nbrImg, (int)trainingData.size() / nbrImg, CV_32FC1, &trainingData[0]);
    result = pSvm.train(trainingMat, labelsMat, cv::Mat(), cv::Mat(), svmParams);

    pSvm.save((const char*)gOutput);
}

/*************/
void testSVM(vector<string>& pPositiveFiles, vector<string>& pNegativeFiles, Descriptor_Hog& pDescriptor, CvSVM& pSvm, cv::PCA pPca, float pPortion = 1.0)
{
    float portion;
    if (pPortion < 1.f)
        portion = pPortion;
    else
        portion = 0.f;

    // We compute the total detection time (descriptor creation + prediction)
    unsigned long long totalTime = 0;
    unsigned long long chronoTime;
    auto chronoStart = std::chrono::high_resolution_clock::now();

    /***/
    if (!gTable)
        cout << "Testing positive files... " << flush;

    int positive = 0;
    int negative = 0;
    int portionOfPositiveFiles = (int)((float)pPositiveFiles.size() * portion);
    for (int i = portionOfPositiveFiles; i < pPositiveFiles.size(); ++i)
    {
        cv::Mat image = cv::imread(pPositiveFiles[i]);

        chronoStart = std::chrono::high_resolution_clock::now();
        chronoTime = chrono::duration_cast<chrono::microseconds>(chronoStart.time_since_epoch()).count();

        pDescriptor.setImage(image);

        vector<float> description = pDescriptor.getDescriptor(_roiPosition);
        cv::Mat descriptionMat(1, (int)description.size(), CV_32FC1, &description[0]);

        // If we used PCA for training
        if (gPca < 1.0)
            descriptionMat = pPca.project(descriptionMat);

        float value = pSvm.predict(descriptionMat, true);

        if (gVerbose)
            cout << pPositiveFiles[i] << " -> " << value << endl;

        if (value < -gMargin)
            positive++;

        totalTime += timeSince(chronoTime);
    }

    /***/
    if (!gTable)
        cout << "Testing negative files" << endl;

    int totalNegatives = 0;
    int portionOfNegativeFiles = (int)((float)pNegativeFiles.size() * portion);
    for (int i = portionOfNegativeFiles; i < pNegativeFiles.size(); ++i)
    {
        cv::Mat image = cv::imread(pNegativeFiles[i]);
        chronoStart = std::chrono::high_resolution_clock::now();
        chronoTime = chrono::duration_cast<chrono::microseconds>(chronoStart.time_since_epoch()).count();
        pDescriptor.setImage(image);

        for (int x = 0; x <= image.cols - _roiSize.x; x += image.cols/5)
        {
            for (int y = 0; y <= image.rows - _roiSize.y; y += image.rows/5)
            {
                vector<float> description = pDescriptor.getDescriptor(_roiPosition + cv::Point_<int>(x, y));
                cv::Mat descriptionMat(1, (int)description.size(), CV_32FC1, &description[0]);
                if (description.size() == 0)
                    continue;

                // If we used PCA for training
                if (gPca < 1.0)
                    descriptionMat = pPca.project(descriptionMat);

                float value = pSvm.predict(descriptionMat, true);

                if (gVerbose)
                    cout << pNegativeFiles[i] << " -> " << value << endl;

                totalNegatives++;
                if (value > -gMargin)
                    negative++;
            }
        }

        totalTime += timeSince(chronoTime);
    }

    float precision = (float)positive / ((float)positive + (float)totalNegatives - (float)negative);
    float recall = (float)positive/ ((float)pPositiveFiles.size() - (float)portionOfPositiveFiles);

    if (!gTable)
    {
        cout << "Positive: " << positive << " / " << pPositiveFiles.size() - portionOfPositiveFiles << endl;
        cout << "Negative: " << negative << " / " << totalNegatives << endl;
        cout << "Precision: " << precision << " -- Recall: " << recall << endl;
        cout << "Time per prediction (us): " << totalTime / (pPositiveFiles.size() - portionOfPositiveFiles + totalNegatives) << endl;
    }
    else
    {
        cout << gIterations << " ";
        cout << gEpsilon << " ";
        cout << gCPenalty << " ";
        cout << gOutput << " ";
        cout << gPosition << " ";
        cout << gCellSize << " ";
        cout << gCellMaxSize << " ";
        cout << gCellStep << " ";
        cout << gBlockSize << " ";
        cout << gRoiSize << " ";
        cout << gBins << " ";
        cout << gSigma << " ";
        cout << gPca << " ";
        cout << totalTime / (pPositiveFiles.size() - portionOfPositiveFiles + totalNegatives) << " ";
        cout << precision << " " << recall << endl;
    }
}

/*************/
void randomize(vector<string>& pFiles)
{
    srand(pFiles.size());
    for (unsigned int i = 0; i < pFiles.size(); ++i)
    {
        int index = rand() % pFiles.size();
        pFiles[i].swap(pFiles[index]);
    }
}

/*************/
int main(int argc, char** argv)
{
    int result = parseArgs(argc, argv);
    if (result)
        return result;

    CvSVM svm;

    // Set up the descriptor
    Descriptor_Hog descriptor;
    descriptor.setHogParams(_roiSize, _blockSize, _cellSize, gBins, false, Descriptor_Hog::L2_NORM, gSigma);
    if (_cellMaxSize.x >= _cellSize.x && _cellMaxSize.y >= _cellSize.y && _cellStep.x >= 1.f && _cellStep.y >= 1.f)
    {
        if (!gTable)
            cout << "Multiscale training activated" << endl;
        descriptor.setMultiscaleParams(_cellSize, _cellMaxSize, _cellStep);
    }

    // The PCA object
    cv::PCA pca;

    // Load both valid and invalid file list
    vector<string> positiveFiles = loadFileList(string(gPositiveDir));
    vector<string> negativeFiles = loadFileList(string(gNegativeDir));

    if (gCrossValidation != 1.f)
    {
        randomize(positiveFiles);
        randomize(negativeFiles);
    }

    // If no model file is specified for testing, we have to create one
    if (gTestFile == NULL)
        trainSVM(positiveFiles, negativeFiles, descriptor, svm, pca, gCrossValidation);
    else
        svm.load((const char*)gTestFile);

    testSVM(positiveFiles, negativeFiles, descriptor, svm, pca, gCrossValidation);

    return 0;
}
