#include "stitch.h"

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
    // we assume to get 2 cameras with resolution 640x480
    mOutputBuffer = cv::Mat::zeros(480, 640, CV_8UC3);

    mName = mClassName;
    // OSC path for this detector
    mOscPath = "stitch";

    mFrameNumber = 0;
}

/*************/
atom::Message Actuator_Stitch::detect(const vector< Capture_Ptr > pCaptures)
{
    vector<cv::Mat> captures = captureToMat(pCaptures);
    
    if (captures.size() == 0)
        return mLastMessage;

    // We first transform all the images (crop + rotation)
    int width = 0, height = 0;
    int type = captures[0].type();

    for (int index = 0; index < captures.size();)
    {
        if (captures[index].type() != type)
        {
            captures.erase(captures.begin() + index);
            continue;
        }

        if (mCameraCrop.find(index) != mCameraCrop.end())
        {
            cv::Mat crop = cv::Mat(captures[index], mCameraCrop[index]);
            captures[index] = crop;
        }
        else
        {
            index++;
            continue;
        }

        if (mCameraRotation.find(index) != mCameraRotation.end())
        {
            cv::Mat result = cv::Mat::zeros(captures[index].size(), captures[index].type());
            cv::Point2f center = cv::Point2f((float)captures[index].cols / 2.f, (float)captures[index].rows / 2.f);
            cv::Mat t = cv::getRotationMatrix2D(center, mCameraRotation[index], 1.0);

            cv::warpAffine(captures[index], result, t, result.size(), cv::INTER_LINEAR);
            captures[index] = result;

            width = max(width, (int)mCameraPosition[index].at<float>(0, 0) + captures[index].cols);
            height = max(height, (int)mCameraPosition[index].at<float>(1, 0) + captures[index].rows);
        }
        else
        {
            width = max(width, captures[index].cols);
            width = max(width, captures[index].rows);
        }

        index++;
    }

    // Then we put them at the right place in the image
    cv::Mat stitch = cv::Mat::zeros(height, width, type); 
    cv::Mat mask = cv::Mat::ones(height, width, CV_8U);
    for (int index = 0; index < captures.size(); ++index)
    {
        cv::Mat roi, roiMask;
        if (mCameraPosition.find(index) != mCameraPosition.end())
        {
            roi = stitch(cv::Rect(mCameraPosition[index].at<float>(0), mCameraPosition[index].at<float>(1),
                                  captures[index].cols, captures[index].rows));
            roiMask = mask(cv::Rect(mCameraPosition[index].at<float>(0), mCameraPosition[index].at<float>(1),
                                  captures[index].cols, captures[index].rows));
        }
        else
        {
            roi = stitch(cv::Rect(0, 0, captures[index].cols, captures[index].rows));
            roiMask = mask(cv::Rect(0, 0, captures[index].cols, captures[index].rows));
        }

        int nbrChannels = roi.channels();
        cv::Mat channels[nbrChannels];
        for (int i = 0; i < nbrChannels; ++i)
            channels[i] = cv::Mat::zeros(roi.rows, roi.cols, roi.depth());
        cv::split(roi, channels);

        cv::Mat roiSplit[nbrChannels];
        for (int i = 0; i < nbrChannels; ++i)
            channels[i].convertTo(roiSplit[i], CV_8U);

        cv::add(captures[index], roi, roi, roiMask);
        for (int x = 0; x < roiMask.cols; ++x)
            for (int y = 0; y < roiMask.rows; ++y)
            {
                bool isBlank = true;
                for (int c = 0; c < nbrChannels; ++c)
                    if (roiSplit[c].at<uchar>(y, x) != 0)
                        isBlank = false;

                if (isBlank)
                    roiMask.at<uchar>(y, x) = 0;
            }
        cv::addWeighted(captures[index], 0.5, roi, 0.5, 0.0, roi);
    }

    mOutputBuffer = stitch;

    mFrameNumber++;
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

    if (cmd == "cropInput")
    {
        float index;
        if (!readParam(pMessage, index))
            return;

        float pos[4];
        for (int i = 0; i < 4; ++i)
            if (!readParam(pMessage, pos[i], i + 2))
                return;

        cv::Rect roi;
        roi.x = pos[0];
        roi.y = pos[1];
        roi.width = pos[2];
        roi.height = pos[3];

        mCameraCrop[(int)index] = roi;
    }
    else if (cmd == "transform")
    {
        float index;
        if (!readParam(pMessage, index))
            return;

        float t[3];
        for (int i = 0; i < 3; ++i)
            if (!readParam(pMessage, t[i], i + 2))
                return;

        cv::Mat position = cv::Mat::zeros(2, 1, CV_32F);
        position.at<float>(0, 0) = t[0];
        position.at<float>(1, 0) = t[1];

        mCameraRotation[(int)index] = t[2];
        mCameraPosition[(int)index] = position;
    }
    else
        setBaseParameter(pMessage);

}
