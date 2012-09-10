#include <iostream>
#include "opencv2/opencv.hpp"
#include "lo/lo.h"

using namespace cv;

int main(int argc, char** argv)
{
    // Argument variables
    int lCamNbr = 0;
    bool lShowCamera = false;
    bool lShowOutliers = false;
    int lFilterSize = 5;
    char lAddress[20];
    char lPort[8];

    strcpy(lAddress, "127.0.0.1");
    strcpy(lPort, "9000");

    /******************/
    // Argument parsing
    for(int i=1; i<argc; i++)
    {
        // Show capture
        if(strcmp(argv[i], "--showCam") == 0)
            lShowCamera = true;
         // Show outliers
        else if(strcmp(argv[i], "--showOutliers") == 0)
            lShowOutliers = true;
        // Camera selection
        else if(strcmp(argv[i], "--cam") == 0)
        {
            if(i+1 < argc)
            {
                int lTmp = atoi(argv[i+1]);
                if(lTmp < 0)
                    lTmp = 0;
                lCamNbr = lTmp;

                i++;
            }
            else
            {
                std::cout << "Wrong number of argument. Exiting." << std::endl;
                return 1;
            }
        }
        // Filtering size
        else if(strcmp(argv[i], "--filter") == 0)
        {
            if(i+1 < argc)
            {
                int lTmp = atoi(argv[i+1]);
                if(lFilterSize < 1)
                    lFilterSize = 1;
                lFilterSize = lTmp;

                i++;
            }
            else
            {
                std::cout << "Wrong number of argument. Exiting." << std::endl;
                return 1;
            }
        }
        // Output OSC address
        else if(strcmp(argv[i], "--ip") == 0)
        {
            if(i+1 < argc)
            {
                strcpy(lAddress, argv[i+1]);
                i++;
            }
            else
            {
                std::cout << "Wrong number of argument. Exiting." << std::endl;
                return 1;
            }
        }
        // Output OSC port
        else if(strcmp(argv[i], "--port") == 0)
        {
            if(i+1 < argc)
            {
                strcpy(lPort, argv[i+1]);
                i++;
            }
            else
            {
                std::cout << "Wrong number of argument. Exiting." << std::endl;
                return 1;
            }
        }
    }

    // OSC parameters
    lo_address lNet = lo_address_new(lAddress, lPort);

    /******************/
    // Camera allocation
    VideoCapture lCamera(lCamNbr);
    if(!lCamera.isOpened())
    {
        std::cout << "Error while opening camera number " << lCamNbr << ". Exiting." << std::endl;
        return 1;
    }

    // Buffers
    Mat lCamBuffer;
    Mat lMean, lStdDev;
    Mat lOutlier, lEroded, lFiltered;

    /***********/
    // Main loop
    bool lContinue = true;
    while(lContinue)
    {
        // Frame capture
        lCamera.read(lCamBuffer);

        // If the frame seems valid
        if(lCamBuffer.size[0] > 0 && lCamBuffer.size[1] > 0)
        {
            if(lShowCamera)
                imshow("Camera", lCamBuffer);

            // Eliminate the outliers : calculate the mean and std dev
            lOutlier = Mat::zeros(lCamBuffer.size[0], lCamBuffer.size[1], CV_8U);
            lEroded = lOutlier.clone();
            lFiltered = lOutlier.clone();
            cvtColor(lCamBuffer, lOutlier, CV_RGB2GRAY);

            meanStdDev(lCamBuffer, lMean, lStdDev);
            absdiff(lOutlier, lMean.at<double>(0), lOutlier);

            // Detect pixels far from the mean (> 2*stddev)
            threshold(lOutlier, lOutlier, 2*lStdDev.at<double>(0), 255, THRESH_BINARY);

            // Erode and dilate to suppress noise
            erode(lOutlier, lEroded, Mat(), Point(-1, -1), lFilterSize);
            dilate(lEroded, lFiltered, Mat(), Point(-1, -1), lFilterSize);

            if(lShowOutliers)
                imshow("mean", lFiltered);

            // Calculate the barycenter of the outliers
            int lNumber = 0;
            int lX=0, lY=0;

            for(int x=0; x<lFiltered.size[0]; x++)
                for(int y=0; y<lFiltered.size[1]; y++)
                {
                    if(lFiltered.at<uchar>(x, y) == 255)
                    {
                        lX += x;
                        lY += y;
                        lNumber++;
                    }
                }
    
            if(lNumber > 0)
            {
                lX /= lNumber;
                lY /= lNumber;
            }
            else
            {
                lX = lFiltered.size[0] / 2;
                lY = lFiltered.size[0] / 2;
            }
            std::cout << lX << " " << lY << std::endl;

            // Send the result
            lo_send(lNet, "/barycenter/position/", "ii", lX, lY);
            lo_send(lNet, "/barycenter/size/", "i", lNumber);
        }

        // Keyboard
        char lKey = waitKey(5);
        if(lKey == 'q')
            lContinue = false;
    }

    return 0;
}
