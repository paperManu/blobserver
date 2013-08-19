#include "actuator_stitch.h"

#include <iostream>

using namespace std;

std::string Actuator_Stitch::mClassName = "Actuator_Stitch";
std::string Actuator_Stitch::mDocumentation = "N/A";
unsigned int Actuator_Stitch::mSourceNbr = 1;

/*************/
Actuator_Stitch::Actuator_Stitch()
{
    make();
}

/*************/
Actuator_Stitch::Actuator_Stitch(int pParam)
{
    make();
}

/*************/
void Actuator_Stitch::make()
{
    defineOutputResolution = false;

    // we assume to get 2 cameras with resolution 640x480
    mOutputBuffer = cv::Mat::zeros(480, 1280, CV_8UC3);

    mName = mClassName;
    // OSC path for this detector
    mOscPath = "/blobserver/stitch";

    mFrameNumber = 0;

    for (int i=0; i<2; i++) 
    {
        source_crop[i] = false;
        for (int j=0; j<4; j++) 
        {
            source_crop_parameters[i][j] = 0;
        }
        
    }

    source_pos[0][0] = 0;
    source_pos[0][1] = 0;
    source_pos[1][0] = 640;
    source_pos[1][1] = 0;

}

/*************/
atom::Message Actuator_Stitch::detect(const vector< Capture_Ptr > pCaptures)
{

    vector<cv::Mat> captures = captureToMat(pCaptures);
    
    if (captures.size() == 0)
        return mLastMessage;

    
    // make sure there are 2 sources
    if (captures.size() == 1)
        mOutputBuffer = captures[0].clone();
    else
    {
        cv::Mat input1 = captures[0];
        cv::Mat input2 = captures[1];

        int px1 = source_pos[0][0];
        int py1 = source_pos[0][1];

        int px2 = source_pos[1][0];
        int py2 = source_pos[1][1];

        // only do this once (unless size of input changes)
        if (defineOutputResolution == false)    
        {
            int maxw = max(px1 + input1.cols - source_crop_parameters[0][0] - source_crop_parameters[0][2], px2 + input2.cols - source_crop_parameters[1][0] - source_crop_parameters[1][2]);
            int maxh = max(py1 + input1.rows - source_crop_parameters[0][1] - source_crop_parameters[0][3], py2 + input2.rows - source_crop_parameters[1][1] - source_crop_parameters[1][3]);
            mOutputBuffer = cv::Mat::zeros(maxh, maxw, CV_8UC3);
            defineOutputResolution = true;
        }

        // paste cropped images on output image
        if (source_crop[0])
        {
            int x1 = source_crop_parameters[0][0];
            int y1 = source_crop_parameters[0][1];
            int w1 = input1.cols - source_crop_parameters[0][2] - x1;
            int h1 = input1.rows - source_crop_parameters[0][3] - y1;
            w1 = min (w1, mOutputBuffer.cols - px1);
            h1 = min (h1, mOutputBuffer.rows - py1);
            cv::Mat crop1 = input1(cv::Rect(x1,y1,w1,h1)).clone();

            crop1.copyTo(mOutputBuffer(cv::Rect(px1,py1,crop1.cols,crop1.rows)));
        }
        else
            // mOutputBuffer = captures[1].clone();
            input1.copyTo(mOutputBuffer(cv::Rect(px1,py1,input1.cols,input1.rows)));

        if (source_crop[1])
        {
            int x2 = source_crop_parameters[1][0];
            int y2 = source_crop_parameters[1][1];
            int w2 = input2.cols - source_crop_parameters[1][2] - x2;
            int h2 = input2.rows - source_crop_parameters[1][3] - y2;
            w2 = min (w2, mOutputBuffer.cols - px2);
            h2 = min (h2, mOutputBuffer.rows - py2);

            cv::Mat crop2 = input2(cv::Rect(x2,y2,w2,h2)).clone();
            // cv::Mat mask = cv::Mat::zeros(h2, w2, CV_8UC1);
            // cv::Mat mask = cv::Mat::zeros(crop2.cols, crop2.rows, CV_8UC1);
            // for (int i=0; i<mask.rows; i++) 
            // {
            //     for (int j=0; j<mask.cols/2; j++)
            //     {
            //         // uchar& v = mask.at<uchar>(i,j);
            //         // v[0] = 0.5;
            //         mask.at<uchar>(i,j) = 0.01*j;    // int(255 * float(j) / (mask.cols/2))
            //     }
            // }
            // crop2.copyTo(mOutputBuffer(cv::Rect(px2,py2,crop2.cols,crop2.rows)), mask);
            crop2.copyTo(mOutputBuffer(cv::Rect(px2,py2,crop2.cols,crop2.rows)));
        }
        else
            // mOutputBuffer = captures[1].clone();
            input2.copyTo(mOutputBuffer(cv::Rect(px2,py2,input2.cols,input2.rows)));
    }


    mFrameNumber++;
    // no need to send out an OSC message
    // mLastMessage = atom::createMessage("iii", 1, 1, mFrameNumber);

    return mLastMessage;
}

/*************/
void Actuator_Stitch::setParameter(atom::Message pMessage)
{
    std::string cmd;

    try
    {
        cmd = toString(pMessage[0]);
    }
    catch (atom::BadTypeTagError exception)
    {
        return;
    }

    if (cmd == "cam0_crop")
    {
        if (pMessage.size() == 5)
        {
            try
            {
                source_crop_parameters[0][0] = atom::toInt(pMessage[1]);
                source_crop_parameters[0][1] = atom::toInt(pMessage[2]);
                source_crop_parameters[0][2] = atom::toInt(pMessage[3]);
                source_crop_parameters[0][3] = atom::toInt(pMessage[4]);
            }
            catch (atom::BadTypeTagError error)
            {
                return;
            }

            source_crop[0] = true;
        }
        else
            return;
    }
    else if (cmd == "cam1_crop")
    {
        if (pMessage.size() == 5)
        {
            try
            {
                source_crop_parameters[1][0] = atom::toInt(pMessage[1]);
                source_crop_parameters[1][1] = atom::toInt(pMessage[2]);
                source_crop_parameters[1][2] = atom::toInt(pMessage[3]);
                source_crop_parameters[1][3] = atom::toInt(pMessage[4]);
            }
            catch (atom::BadTypeTagError error)
            {
                return;
            }

            source_crop[1] = true;
        }
        else
            return;
    }
    else if (cmd == "cam0_pos")
    {
        if (pMessage.size() == 3)
        {
            try
            {
                source_pos[0][0] = atom::toInt(pMessage[1]);
                source_pos[0][1] = atom::toInt(pMessage[2]);
            }
            catch (atom::BadTypeTagError error)
            {
                return;
            }
        }
        else
            return;
    }
    else if (cmd == "cam1_pos")
    {
        if (pMessage.size() == 3)
        {
            try
            {
                source_pos[1][0] = atom::toInt(pMessage[1]);
                source_pos[1][1] = atom::toInt(pMessage[2]);
            }
            catch (atom::BadTypeTagError error)
            {
                return;
            }
        }
        else
            return;
    }
    else
        setBaseParameter(pMessage);

}
