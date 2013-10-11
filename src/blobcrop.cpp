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
 * @blobcrop.cpp
 * This utility is designed to help with the creation of a database
 * for later training of recognition algorithms
 */

#include <iostream>
#include <memory>
#include <vector>

#include <glib.h>
#include <glib/gstdio.h>
#include <opencv2/opencv.hpp>

#include <config.h>

using namespace std;

char* gExtension = NULL;
char* gOutputDir = NULL;
char* gPrefix = NULL;
cv::Size gRoiSize;

int gCropIndex = 0;

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
struct CbData
{
    cv::Mat image;
    string filename;
    string output;
};

/*************/
void onMouseCb(int event, int x, int y, int flags, void* userdata)
{
    if (userdata == NULL)
        return;
   
    CbData* data = (CbData*)userdata;
    cv::Mat image = data->image;
    cv::Mat mask = cv::Mat::zeros(image.size(), CV_8U);

    cv::Rect roi(x, y, gRoiSize.width, gRoiSize.height);
    if (roi.x + roi.width >= image.cols || roi.y + roi.height >= image.rows)
    {
        cv::imshow(data->filename, image);
        return;
    }

    cv::rectangle(mask, roi, cv::Scalar(255), CV_FILLED);

    cv::Mat displayMat, inverse;
    image.copyTo(displayMat, mask);
    image.copyTo(inverse, 255 - mask);
    cv::imshow(data->filename, displayMat + inverse / 2);

    if (event != cv::EVENT_LBUTTONDOWN)
        return;

    char outputName[16];
    sprintf(outputName, "%s%05d", gPrefix, gCropIndex);
    cv::Mat crop = image(roi);
    string path = string(data->output) + string("/") + string(outputName) + string(".") + string(gExtension);
    cv::imwrite(path, crop);
    cout << path << " written" << endl;

    gCropIndex++;
}

/*************/
int main(int argc, char** argv)
{
    gExtension = (char*)malloc(8 * sizeof(char));
    gExtension = (char*)"png";

    gOutputDir = (char*)malloc(128 * sizeof(char));
    gOutputDir = (char*)"crops";

    gPrefix = (char*)malloc(64 * sizeof(char));
    gPrefix = (char*)"";

    gRoiSize = cv::Size(64, 128);

    for (int i = 1; i < argc;)
    {
        if (strcmp(argv[i], "--ext") == 0)
        {
            if (argc >= i + 1)
            {
                gExtension = argv[i+1];
            }
            cout << "Chosen extension: " << gExtension << endl;
            i += 2;
        }
        else if (strcmp(argv[i], "--size") == 0)
        {
            if (argc >= i + 2)
            {
                int h, w;
                try
                {
                    h = stoi(string(argv[i + 1]));
                    w = stoi(string(argv[i + 2]));
                }
                catch (exception invalid_argument)
                {
                }
                gRoiSize = cv::Size(h, w);
                cout << "Chosen size: " << h << "x" << w << endl;
            }
            i += 3;
        }
        else if (strcmp(argv[i], "--out") == 0)
        {
            if (argc >= i + 1)
            {
                gOutputDir = argv[i + 1];
            }
            i += 2;
        }
        else if (strcmp(argv[i], "--prefix") == 0)
        {
            if (argc >= i + 1)
            {
                gPrefix = argv[i + 1];
            }
            i += 2;
        }
        else if (strcmp(argv[i], "--help") == 0)
        {
            cout << "Blobcrop, useful utility to crop semi-automatically images for later training" << endl;
            cout << "Usage:" << endl;
            cout << "--ext extension \tSpecifies the image file extension to use (default: png)" << endl;
            cout << "--size h w \t\tSpecifies the size of the crops (default: 64 128)" << endl;
            cout << "--out name \t\tSet the output directory to contain cropped images (default: crops)" << endl;
            cout << "--prefix name \tSet a prefix to the output file names (default: none)" << endl;
            return 1;
        }
    }

    vector<string> fileList = loadFileList("./");
    if (fileList.empty())
    {
        cout << "Not a single image file has been found. Exiting" << endl;
        return 1;
    }
    cout << "Found " << fileList.size() << " images to process." << endl;
    cout << "Notice: use 'n' for next image, 'p' for the previous one" << endl;

    g_mkdir(gOutputDir, 0755);
   
    //for_each (fileList.begin(), fileList.end(), [&] (string filename)
    for (int i = 0; i < fileList.size(); ++i)
    {
        string filename = fileList[i];

        cv::Mat image = cv::imread(filename, -1);
        if (image.total() == 0)
            continue;

        cv::imshow(filename.c_str(), image);

        CbData data;
        data.image = image;
        data.filename = filename;
        data.output = string(gOutputDir);

        cv::setMouseCallback(filename.c_str(), onMouseCb, &data);

        bool goOn = true;
        while (goOn)
        {
            char key = cv::waitKey(16);
            if (key == 'n')
                goOn = false;
            else if (key == 'p')
            {
                goOn = false;
                i = max(-1, i - 2);
            }
            else if ((int)key == 27)
                return 1;
        }

        cv::destroyWindow(filename.c_str());
    }
}
