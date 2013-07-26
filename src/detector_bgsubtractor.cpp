#include "detector_bgsubtractor.h"

#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>

using namespace std;

/*************/
// Definition of class Detector_BgSubtractor
/*************/
std::string Detector_BgSubtractor::mClassName = "Detector_BgSubtractor";
std::string Detector_BgSubtractor::mDocumentation = "N/A";
unsigned int Detector_BgSubtractor::mSourceNbr = 1;

/*************/
Detector_BgSubtractor::Detector_BgSubtractor()
{
    make();
}

/*************/
Detector_BgSubtractor::Detector_BgSubtractor(int pParam)
{
    make();
}

/*************/
void Detector_BgSubtractor::make()
{
    mOutputBuffer = cv::Mat::zeros(480, 640, CV_8UC3);

    mName = mClassName;
    mOscPath = "/blobserver/bgsubtractor";

    mFilterSize = 3;
    mFilterDilateCoeff = 2;

    mBlobLifetime = 30;
    mProcessNoiseCov = 1e-6;
    mMeasurementNoiseCov = 1e-4;

    mLearningRate = 300;
    mMinArea = 0.f;
    mMaxArea = 65535.f;
}

/*************/
atom::Message Detector_BgSubtractor::detect(const vector<cv::Mat> pCaptures)
{
    // For simplicity...
    cv::Mat input = pCaptures[0];

    // We get windows of interest, using BG subtraction
    // and previous blobs positions
    mBgSubtractor(input, mBgSubtractorBuffer, mLearningRate);

    // Info when learning should be done
    static int learnTimeElapsed = 0;
    if (learnTimeElapsed == 0)
    {
        learnTimeElapsed++;
        g_log(NULL, G_LOG_LEVEL_INFO, "%s: Background learning started", mClassName.c_str());
    }
    else if (learnTimeElapsed < mLearningTime)
        learnTimeElapsed++;
    else if (learnTimeElapsed == mLearningTime)
    {
        learnTimeElapsed++;
        g_log(NULL, G_LOG_LEVEL_INFO, "%s: Background learning done", mClassName.c_str());
    }

    // Erode and dilate to suppress noise
    cv::Mat lEroded;
    cv::erode(mBgSubtractorBuffer, lEroded, cv::Mat(), cv::Point(-1, -1), mFilterSize);
    cv::dilate(lEroded, mBgSubtractorBuffer, cv::Mat(), cv::Point(-1, -1), mFilterSize * mFilterDilateCoeff);
    cv::threshold(mBgSubtractorBuffer, mBgSubtractorBuffer, 250, 255, cv::THRESH_BINARY);

    vector< vector<cv::Point> > contours;
    cv::Mat buffer = mBgSubtractorBuffer.clone();
    cv::findContours(buffer, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    vector<Blob::properties> properties;
    for (unsigned int i = 0; i < contours.size(); ++i)
    {
        cv::Rect box = cv::boundingRect(contours[i]);
        float area = cv::contourArea(contours[i], false);

        if (area < mMinArea || area > mMaxArea)
            continue;

        Blob::properties property;
        property.position.x = box.x + box.width / 2;
        property.position.y = box.y + box.height / 2;
        property.size = area;
        property.speed.x = 0.f;
        property.speed.y = 0.f;

        properties.push_back(property);
    }

    // We want to track them
    trackBlobs<Blob2D>(properties, mBlobs, mBlobLifetime);

    // We make sure that the filtering parameters are set
    for (int i = 0; i < mBlobs.size(); ++i)
    {
        mBlobs[i].setParameter("processNoiseCov", mProcessNoiseCov);
        mBlobs[i].setParameter("measurementNoiseCov", mMeasurementNoiseCov);
    }

    cv::Mat resultMat = cv::Mat::zeros(input.rows, input.cols, CV_8UC3);
    for_each (mBlobs.begin(), mBlobs.end(), [&] (Blob2D blob)
    {
        Blob::properties props = blob.getBlob();
        cv::circle(resultMat, props.position, sqrtf(props.size), cv::Scalar(1, 1, 1), CV_FILLED);
    } );

    // The result is shown
    cv::multiply(input, resultMat, resultMat);

    // Constructing the message
    mLastMessage.clear();
    mLastMessage.push_back(atom::IntValue::create((int)mBlobs.size()));
    mLastMessage.push_back(atom::IntValue::create(6));

    for(int i = 0; i < mBlobs.size(); ++i)
    {
        int lX, lY, lSize, lId;
        float ldX, ldY;
        Blob::properties properties = mBlobs[i].getBlob();
        lX = (int)(properties.position.x);
        lY = (int)(properties.position.y);
        lSize = (int)(properties.size);
        ldX = properties.speed.x;
        ldY = properties.speed.y;
        lId = (int)mBlobs[i].getId();

        // Print the blob number on the blob
        if (mVerbose)
        {
            char lNbrStr[8];
            sprintf(lNbrStr, "%i", lId);
            cv::putText(resultMat, lNbrStr, cv::Point(lX, lY), cv::FONT_HERSHEY_COMPLEX, 0.66, cv::Scalar(128.0, 128.0, 128.0, 128.0));
        }

        // Add this blob to the message
        mLastMessage.push_back(atom::IntValue::create(lX));
        mLastMessage.push_back(atom::IntValue::create(lY));
        mLastMessage.push_back(atom::IntValue::create(lSize));
        mLastMessage.push_back(atom::FloatValue::create(ldX));
        mLastMessage.push_back(atom::FloatValue::create(ldY));
        mLastMessage.push_back(atom::IntValue::create(lId));
    }

    mOutputBuffer = resultMat.clone();

    return mLastMessage;
}

/*************/
void Detector_BgSubtractor::setParameter(atom::Message pMessage)
{
    std::string cmd;
    try
    {
        cmd = toString(pMessage[0]);
    }
    catch (atom::BadTypeTagError error)
    {
        return;
    }

    if (cmd == "filterSize")
    {
        float filterSize;
        if (readParam(pMessage, filterSize))
            mFilterSize = max(1, (int)filterSize);
    }
    else if (cmd == "lifetime")
    {
        float lifetime;
        if (readParam(pMessage, lifetime))
            mBlobLifetime = lifetime;
    }
    else if (cmd == "processNoiseCov")
    {
        float cov;
        if (readParam(pMessage, cov))
            mProcessNoiseCov = abs(cov);
    }
    else if (cmd == "measurementNoiseCov")
    {
        float cov;
        if (readParam(pMessage, cov))
            mMeasurementNoiseCov = abs(cov);
    }
    else if (cmd == "learningTime")
    {
        float time;
        if (readParam(pMessage, time))
        {
            float rate = 1.f/time * log(0.9);
            rate = 1.f - exp(rate);
            mLearningRate = min(1.0, (double)rate);
            mLearningTime = int(time);
        }
    }
    else if (cmd == "area")
    {
       float mini, maxi;
       if (!readParam(pMessage, mini, 1))
          return;
       if (!readParam(pMessage, maxi, 2))
          return;

       mMinArea = mini;
       mMaxArea = maxi;
    }
    else
        setBaseParameter(pMessage);
}
