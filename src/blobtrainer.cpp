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
 * used in some detectors.
 */

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
static gchar* gBlockSize = NULL;
static gchar* gRoiSize = NULL;

int _svmCriteria;
cv::Point_<int> _roiPosition;
cv::Point_<int> _cellSize;
cv::Point_<int> _blockSize;
cv::Point_<int> _roiSize;

static GOptionEntry gEntries[] =
{
    {"version", 'v', 0, G_OPTION_ARG_NONE, &gVersion, "Shows the version of this software", NULL},
    {"verbose", 'V', 0, G_OPTION_ARG_NONE, &gVerbose, "More verbose output", NULL},
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
    {"block-size", 0, 0, G_OPTION_ARG_STRING, &gBlockSize, "Specifies the size of the blocks over which cells are normalized (default: '3x3')", NULL},
    {"roi-size", 0, 0, G_OPTION_ARG_STRING, &gRoiSize, "Specifies the size (in pixels) of the ROI from which to create descriptors (default: '64x128')", NULL},
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
int main(int argc, char** argv)
{
    int result = parseArgs(argc, argv);
    if (result)
        return result;

    CvSVM svm;

    // Set up the descriptor
    Descriptor_Hog descriptor;
    descriptor.setHogParams(_roiSize, _blockSize, _cellSize, gBins, false, Descriptor_Hog::L2_NORM);

    // Load both valid and invalid file list
    vector<string> positiveFiles = loadFileList(string(gPositiveDir));
    vector<string> negativeFiles = loadFileList(string(gNegativeDir));

    // If no model file is specified for testing, we have to create one
    if (gTestFile == NULL)
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
        cout << "   block size = " << gBlockSize << endl;
        cout << "   roi size = " << gRoiSize << endl;
        cout << "   bins per cell = " << gBins << endl;
        
        // Set up training data
        if (!gVerbose)
            cout << "Loading training data... " << flush;

        int nbrImg = 0;
        vector<float> labels;
        vector<float> trainingData;
        for (int i = 0; i < positiveFiles.size(); ++i)
        {
            if (gVerbose)
                cout << "Analysing " << positiveFiles[i] << endl;

            cv::Mat image = cv::imread(positiveFiles[i]);
            descriptor.setImage(image);
            vector<float> description = descriptor.getDescriptor(_roiPosition);

            labels.push_back(1.f);
            for (int j = 0; j < description.size(); ++j)
                trainingData.push_back(description[j]);

            nbrImg++;
        }
        if (!gVerbose)
            cout << "Positive data loaded... " << flush;

        for (int i = 0; i < negativeFiles.size(); ++i)
        {
            if (gVerbose)
                cout << "Analysing " << negativeFiles[i] << endl;

            cv::Mat image = cv::imread(negativeFiles[i]);
            descriptor.setImage(image);
            for (int x = 0; x < 4; ++x)
            {
                for (int y = 0; y < 4; ++y)
                {
                    vector<float> description = descriptor.getDescriptor(cv::Point_<int>(8 + x*16, 8 + y*16));

                    labels.push_back(-1.f);
                    for (int j = 0; j < description.size(); ++j)
                        trainingData.push_back(description[j]);

                    nbrImg++;
                }
            }
        }
        if (!gVerbose)
            cout << "Negative data loaded" << endl;
        
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
        result = svm.train(trainingMat, labelsMat, cv::Mat(), cv::Mat(), svmParams);

        svm.save((const char*)gOutput);
    }
    else
    {
        svm.load((const char*)gTestFile);
    }

    cout << "Testing positive files... " << flush;
    int positive = 0;
    int negative = 0;
    for (int i = 0; i < positiveFiles.size(); ++i)
    {
        cv::Mat image = cv::imread(positiveFiles[i]);
        descriptor.setImage(image);
        vector<float> description = descriptor.getDescriptor(cv::Point_<int>(16, 16));
        cv::Mat descriptionMat(1, (int)description.size(), CV_32FC1, &description[0]);
        float value = svm.predict(descriptionMat);

        if (gVerbose)
            cout << positiveFiles[i] << " -> " << value << endl;

        if (value == 1)
            positive++;
    }

    cout << "Testing negative files" << endl;
    for (int i = 0; i < negativeFiles.size(); ++i)
    {
        cv::Mat image = cv::imread(negativeFiles[i]);
        descriptor.setImage(image);
        vector<float> description = descriptor.getDescriptor(cv::Point_<int>(8, 8));
        cv::Mat descriptionMat(1, (int)description.size(), CV_32FC1, &description[0]);
        float value = svm.predict(descriptionMat);

        if (gVerbose)
            cout << negativeFiles[i] << " -> " << value << endl;

        if (value == -1)
            negative++;
    }

    cout << "Positive: " << positive << " / " << positiveFiles.size() << endl;
    cout << "Negative: " << negative << " / " << negativeFiles.size() << endl;

    return 0;
}
