#include "detector_stitch.h"

#include <iostream>

using namespace std;

std::string Detector_Stitch::mClassName = "Detector_Stitch";
std::string Detector_Stitch::mDocumentation = "N/A";
unsigned int Detector_Stitch::mSourceNbr = 1;

/*************/
Detector_Stitch::Detector_Stitch()
{
    make();
}

/*************/
Detector_Stitch::Detector_Stitch(int pParam)
{
    make();
}

/*************/
void Detector_Stitch::make()
{
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
atom::Message Detector_Stitch::detect(vector<cv::Mat> pCaptures)
{

    if (pCaptures.size() == 0)
        return mLastMessage;

    
    // make sure there are 2 sources
    if (pCaptures.size() == 1)
        mOutputBuffer = pCaptures[0].clone();
    else
    {
        cv::Mat input1 = pCaptures[0];
        cv::Mat input2 = pCaptures[1];

        int px1 = source_pos[0][0];
        int py1 = source_pos[0][1];

        int px2 = source_pos[1][0];
        int py2 = source_pos[1][1];

        // paste cropped images on same image
        if (source_crop[0])
        {
            int x1 = source_crop_parameters[0][0];
            int y1 = source_crop_parameters[0][1];
            int w1 = input1.cols - source_crop_parameters[0][2] - x1;
            int h1 = input1.rows - source_crop_parameters[0][3] - y1;
            cv::Mat crop1 = input1(cv::Rect(x1,y1,w1,h1)).clone();

            crop1.copyTo(mOutputBuffer(cv::Rect(px1,py1,crop1.cols,crop1.rows)));
        }
        else
            // mOutputBuffer = pCaptures[1].clone();
            input1.copyTo(mOutputBuffer(cv::Rect(px1,py1,input1.cols,input1.rows)));

        if (source_crop[1])
        {
            int x2 = source_crop_parameters[1][0];
            int y2 = source_crop_parameters[1][1];
            int w2 = input2.cols - source_crop_parameters[1][2] - x2;
            int h2 = input2.rows - source_crop_parameters[1][3] - y2;
            // mOutputBuffer = pCaptures[1](cv::Rect(x1,y1,w1,h1)).clone();

            cv::Mat crop2 = input2(cv::Rect(x2,y2,w2,h2)).clone();

            crop2.copyTo(mOutputBuffer(cv::Rect(px2,py2,crop2.cols,crop2.rows)));
        }
        else
            // mOutputBuffer = pCaptures[1].clone();
            input2.copyTo(mOutputBuffer(cv::Rect(px2,py2,input2.cols,input2.rows)));
    }


    mFrameNumber++;
    mLastMessage = atom::createMessage("iii", 1, 1, mFrameNumber);

    return mLastMessage;
}

/*************/
void Detector_Stitch::setParameter(atom::Message pMessage)
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
