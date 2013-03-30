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
static gchar* gExtension = NULL;
static gchar* gPositiveDir = NULL;
static gchar* gNegativeDir = NULL;

static GOptionEntry gEntries[] =
{
    {"version", 'v', 0, G_OPTION_ARG_NONE, &gVersion, "Shows the version of this software", NULL},
    {"extension", 'e', 0, G_OPTION_ARG_STRING, &gExtension, "Specifies the extension for the files to load (default: png)", NULL},
    {"positive", 'p', 0, G_OPTION_ARG_STRING, &gPositiveDir, "Specifies the subdirectory where to load positive images (default: ./positive)", NULL},
    {"negative", 'n', 0, G_OPTION_ARG_STRING, &gNegativeDir, "Specifies the subdirectory where to load negative images (default: ./negative)", NULL},
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

    // Load both valid and invalid file list
    vector<string> positiveFiles = loadFileList(string(gPositiveDir));
    vector<string> negativeFiles = loadFileList(string(gNegativeDir));

    // Set up the descriptor
    Descriptor_Hog descriptor;
    int binsPerCell = 9;
    descriptor.setHogParams(cv::Size_<int>(8, 16), cv::Size_<int>(3, 3), cv::Size_<int>(8, 8), binsPerCell, false);

    // Set up training data
    int nbrImg = 0;
    vector<float> labels;
    vector<float> trainingData;
    for (int i = 0; i < positiveFiles.size(); ++i)
    {
        cout << "Analysing " << positiveFiles[i] << endl;
        cv::Mat image = cv::imread(positiveFiles[i]);
        descriptor.setImage(image);
        vector<float> description = descriptor.getDescriptor(cv::Point_<int>(16, 16));

        labels.push_back(1.f);
        for (int j = 0; j < description.size(); ++j)
            trainingData.push_back(description[j]);

        nbrImg++;
    }

    for (int i = 0; i < negativeFiles.size(); ++i)
    {
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
    
    // Set up the SVM train parameter
    CvSVMParams svmParams;
    svmParams.svm_type = CvSVM::C_SVC;
    svmParams.C = 0.1;
    svmParams.kernel_type = CvSVM::LINEAR;
    svmParams.term_crit = cvTermCriteria(CV_TERMCRIT_EPS, 1e7, 1e-1);

    // Train the SVM
    CvSVM svm;
    cv::Mat labelsMat((int)labels.size(), 1, CV_32FC1, &labels[0]);
    cv::Mat trainingMat(nbrImg, (int)trainingData.size() / nbrImg, CV_32FC1, &trainingData[0]);
    result = svm.train(trainingMat, labelsMat, cv::Mat(), cv::Mat(), svmParams);

    svm.save("model.xml");

    cout << endl << "--- Testing positive files ---" << endl;
    int positive = 0;
    int negative = 0;
    for (int i = 0; i < positiveFiles.size(); ++i)
    {
        cv::Mat image = cv::imread(positiveFiles[i]);
        descriptor.setImage(image);
        vector<float> description = descriptor.getDescriptor(cv::Point_<int>(16, 16));
        cv::Mat descriptionMat(1, (int)description.size(), CV_32FC1, &description[0]);
        float value = svm.predict(descriptionMat);
        cout << positiveFiles[i] << " -> " << value << endl;

        if (value == 1)
            positive++;
    }

    cout << endl << "--- Testing negative files ---" << endl;
    for (int i = 0; i < negativeFiles.size(); ++i)
    {
        cv::Mat image = cv::imread(negativeFiles[i]);
        descriptor.setImage(image);
        vector<float> description = descriptor.getDescriptor(cv::Point_<int>(8, 8));
        cv::Mat descriptionMat(1, (int)description.size(), CV_32FC1, &description[0]);
        float value = svm.predict(descriptionMat);
        cout << negativeFiles[i] << " -> " << value << endl;

        if (value == -1)
            negative++;
    }

    cout << endl << "Positive: " << positive << " / " << positiveFiles.size() << endl;
    cout << endl << "Negative: " << negative << " / " << negativeFiles.size() << endl;

    return 0;
}
