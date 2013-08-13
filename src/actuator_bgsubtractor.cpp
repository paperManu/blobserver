#include "actuator_bgsubtractor.h"

#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>

using namespace std;

/*************/
// Definition of class Actuator_BgSubtractor
/*************/
std::string Actuator_BgSubtractor::mClassName = "Actuator_BgSubtractor";
std::string Actuator_BgSubtractor::mDocumentation = "N/A";
unsigned int Actuator_BgSubtractor::mSourceNbr = 1;

/*************/
Actuator_BgSubtractor::Actuator_BgSubtractor()
{
    make();
}

/*************/
Actuator_BgSubtractor::Actuator_BgSubtractor(int pParam)
{
    make();
}

/*************/
void Actuator_BgSubtractor::make()
{
    mOutputBuffer = cv::Mat::zeros(480, 640, CV_8UC3);

    mName = mClassName;
    mOscPath = "/blobserver/bgsubtractor";

    mFilterSize = 3;
    mFilterDilateCoeff = 2;

    mBlobLifetime = 30;
    mKeepOldBlobs = 0;
    mKeepMaxTime = 0;
    mProcessNoiseCov = 1e-6;
    mMeasurementNoiseCov = 1e-4;
    mMaxDistanceForColorDiff = 16;

    mLearningRate = 300;
    mMinArea = 0.f;
    mMaxArea = 65535.f;
}

/*************/
atom::Message Actuator_BgSubtractor::detect(const vector< Capture_Ptr > pCaptures)
{
    vector<cv::Mat> captures = captureToMat(pCaptures);
    if (captures.size() < mSourceNbr)
        return mLastMessage;

    // For simplicity...
    cv::Mat input = captures[0];

    // We get windows of interest, using BG subtraction
    // and previous blobs positions
    mBgSubtractor(input, mBgSubtractorBuffer, mLearningRate);
    
    // Info when learning should be done
    static int learnTimeElapsed = 0;
    if (learnTimeElapsed == 0)
    {
        learnTimeElapsed++;
        g_log(NULL, G_LOG_LEVEL_INFO, "%s: Background learning started", mClassName.c_str());
        atom::Message msg;
        return msg;
    }
    else if (learnTimeElapsed < mLearningTime)
    {
        learnTimeElapsed++;
        atom::Message msg;
        return msg;
    }
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

    // Preparing some parameters for histogram creation
    int channels[input.channels()];
    int histSize[input.channels()];
    float range[] = {0, 256};
    const float* ranges[input.channels()];

    for (int i = 0; i < input.channels(); ++i)
    {
        channels[i] = i;
        histSize[i] = 32;
        ranges[i] = range;
    }

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

        cv::Mat crop = cv::Mat(input, box);
        cv::Mat hist;
        cv::calcHist(&crop, 1, channels, cv::Mat(), hist, 3, histSize, ranges, true, false);
        property.colorHist = hist;

        properties.push_back(property);
    }

    // We want to track them
    trackBlobs<Blob2DColor>(properties, mBlobs, mBlobLifetime, mKeepOldBlobs, mKeepMaxTime);

    // We make sure that the filtering parameters are set
    for (int i = 0; i < mBlobs.size(); ++i)
    {
        mBlobs[i].setParameter("processNoiseCov", mProcessNoiseCov);
        mBlobs[i].setParameter("measurementNoiseCov", mMeasurementNoiseCov);
        mBlobs[i].setParameter("maxDistanceForColorDiff", mMaxDistanceForColorDiff);
    }

    // We delete blobs which are outside the frame
    for (int i = 0; i < mBlobs.size();)
    {
        Blob::properties prop = mBlobs[i].getBlob();
        if (prop.position.x > input.cols || prop.position.x < 0 || prop.position.y > input.rows || prop.position.y < 0)
            mBlobs.erase(mBlobs.begin() + i);
        else
            i++;
    }

    cv::Mat resultMat = cv::Mat::zeros(input.rows, input.cols, CV_8UC3);
    for_each (mBlobs.begin(), mBlobs.end(), [&] (Blob2DColor blob)
    {
        Blob::properties props = blob.getBlob();
        cv::circle(resultMat, props.position, sqrtf(props.size), cv::Scalar(1, 1, 1), CV_FILLED);
    } );

    // The result is shown
    cv::multiply(input, resultMat, resultMat);

    // Constructing the message
    mLastMessage.clear();
    mLastMessage.push_back(atom::IntValue::create((int)mBlobs.size()));
    mLastMessage.push_back(atom::IntValue::create(8));

    for(int i = 0; i < mBlobs.size(); ++i)
    {
        int lX, lY, lSize, lId, lAge, lLost;
        float ldX, ldY;
        Blob::properties properties = mBlobs[i].getBlob();
        lX = (int)(properties.position.x);
        lY = (int)(properties.position.y);
        lSize = (int)(properties.size);
        ldX = properties.speed.x;
        ldY = properties.speed.y;
        lId = (int)mBlobs[i].getId();
        lAge = (int)mBlobs[i].getAge();
        lLost = (int)mBlobs[i].getLostDuration();

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
        mLastMessage.push_back(atom::IntValue::create(lAge));
        mLastMessage.push_back(atom::IntValue::create(lLost));
    }

    mOutputBuffer = resultMat.clone();

    return mLastMessage;
}

/*************/
void Actuator_BgSubtractor::setParameter(atom::Message pMessage)
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
    if (cmd == "filterDilateCoeff")
    {
        float filterSize;
        if (readParam(pMessage, filterSize))
            mFilterDilateCoeff = max(0, (int)filterSize);
    }
    else if (cmd == "lifetime")
    {
        float lifetime;
        if (readParam(pMessage, lifetime))
            mBlobLifetime = lifetime;
    }
    else if (cmd == "keepOldBlobs")
    {
        float keep;
        if (readParam(pMessage, keep, 1))
            mKeepOldBlobs = (int)keep;
        if (readParam(pMessage, keep, 2))
            mKeepMaxTime = (int)keep;
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
    else if (cmd == "maxDistanceForColorDiff")
    {
        float dist;
        if (readParam(pMessage, dist))
            mMaxDistanceForColorDiff = abs(dist);
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
