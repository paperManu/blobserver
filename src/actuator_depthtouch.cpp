#include "actuator_depthtouch.h"

#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>

using namespace std;

/*************/
// Definition of class Actuator_DepthTouch
/*************/
std::string Actuator_DepthTouch::mClassName = "Actuator_DepthTouch";
std::string Actuator_DepthTouch::mDocumentation = "N/A";
unsigned int Actuator_DepthTouch::mSourceNbr = 1;

/*************/
Actuator_DepthTouch::Actuator_DepthTouch()
{
    make();
}

/*************/
Actuator_DepthTouch::Actuator_DepthTouch(int pParam)
{
    make();
}

/*************/
void Actuator_DepthTouch::make()
{
    mOutputBuffer = cv::Mat::zeros(480, 640, CV_8UC3);

    mName = mClassName;
    mOscPath = "/blobserver/depthtouch";

    mFilterSize = 2;
    mDetectionDistance = 25.f;
    mSigmaCoeff = 20.f;

    mBlobLifetime = 5;
    mProcessNoiseCov = 1e-6;
    mMeasurementNoiseCov = 1e-4;

    mLearningTime = 120;
    mLearningLeft = mLearningTime;
    mIsLearning = true;
    mJustLearnt = false;
}

/*************/
atom::Message Actuator_DepthTouch::detect(const vector< Capture_Ptr > pCaptures)
{
    vector<cv::Mat> captures = captureToMat(pCaptures);
    if (captures.size() < mSourceNbr)
        return mLastMessage;

    if (captures[0].channels() != 1)
        return mLastMessage;

    cv::Mat input;
    captures[0].convertTo(input, CV_32F);

    if (mBackgroundMean.total() != input.total())
    {
        mBackgroundMean = cv::Mat::zeros(input.rows, input.cols, CV_32F);
        mBackgroundStddev = cv::Mat::zeros(input.rows, input.cols, CV_32F);
        mLearningData.clear();
        mIsLearning = true;
        mJustLearnt = false;
        mLearningLeft = mLearningTime;

        g_log(NULL, G_LOG_LEVEL_INFO, "%s - Beginning to learn the background", mClassName.c_str());
    }

    if (mIsLearning)
    {
        learn(input);
        return mLastMessage;
    }
    else if (mJustLearnt == false)
    {
        mJustLearnt = true;
        g_log(NULL, G_LOG_LEVEL_INFO, "%s - Finished learning the background", mClassName.c_str());
    }

    // Difference to mean value
    cv::Mat distance = cv::Mat::zeros(input.rows, input.cols, CV_32F);
    distance = mBackgroundMean - input;
    distance = cv::max(0.0, distance);

    // Comparing distance with std dev
    cv::Mat mask;
    cv::compare(distance, (mBackgroundStddev + 1.0f) * 5.0, mask, cv::CMP_GT);
    mask.convertTo(mask, CV_32F, 1.f / 255.f);
    distance = distance.mul(mask);

    cv::compare(distance, 0.f, mask, cv::CMP_EQ);
    mask.convertTo(mask, CV_32F, 1.f / 255.f);
    distance += mask * 65535.f;

    // Comparing distance with detection distance, plus some morphological operations
    cv::Mat touch;
    cv::Mat touchDistance = cv::max((mBackgroundStddev + 1.0) * mSigmaCoeff, mDetectionDistance);
    cv::compare(distance, touchDistance, touch, cv::CMP_LE);
    cv::Mat lEroded;
    cv::erode(touch, lEroded, cv::Mat(), cv::Point(-1, -1), mFilterSize);
    cv::dilate(lEroded, touch, cv::Mat(), cv::Point(-1, -1), mFilterSize * 2);

    vector< vector<cv::Point> > contours;
    cv::Mat buffer = touch.clone();
    cv::findContours(buffer, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    vector<Blob::properties> properties;
    for (unsigned int i = 0; i < contours.size(); ++i)
    {
        cv::Rect box = cv::boundingRect(contours[i]);
        float area = cv::contourArea(contours[i], false);

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

    // Constructing the message
    mLastMessage.clear();
    mLastMessage.push_back(atom::IntValue::create((int)mBlobs.size()));
    mLastMessage.push_back(atom::IntValue::create(5));

    for(int i = 0; i < mBlobs.size(); ++i)
    {
        int lX, lY, lId;
        float ldX, ldY;
        Blob::properties properties = mBlobs[i].getBlob();
        lX = (int)(properties.position.x);
        lY = (int)(properties.position.y);
        ldX = properties.speed.x;
        ldY = properties.speed.y;
        lId = (int)mBlobs[i].getId();

        // Print the blob number on the blob
        if (mVerbose)
        {
            char lNbrStr[8];
            sprintf(lNbrStr, "%i", lId);
            cv::putText(touch, lNbrStr, cv::Point(lX, lY), cv::FONT_HERSHEY_COMPLEX, 0.66, cv::Scalar(128.0, 128.0, 128.0, 128.0));
        }

        // Add this blob to the message
        mLastMessage.push_back(atom::IntValue::create(lX));
        mLastMessage.push_back(atom::IntValue::create(lY));
        mLastMessage.push_back(atom::FloatValue::create(ldX));
        mLastMessage.push_back(atom::FloatValue::create(ldY));
        mLastMessage.push_back(atom::IntValue::create(lId));
    }

    mOutputBuffer = touch.clone();

    return mLastMessage;
}

/*************/
void Actuator_DepthTouch::learn(cv::Mat input)
{
    mLearningData.push_back(input);
    mLearningLeft--;

    if (mLearningLeft)
        return;

    mIsLearning = false;

    // Calculate the mean
    cv::Mat mean = cv::Mat::zeros(mLearningData[0].rows, mLearningData[1].cols, CV_32F);
    for_each (mLearningData.begin(), mLearningData.end(), [&] (cv::Mat data)
    {
        mean += data;
    });
    mean /= (float)mLearningTime;

    // Calculate the standard deviation
    cv::Mat stddev = cv::Mat::zeros(mLearningData[0].rows, mLearningData[1].cols, CV_32F);
    for_each (mLearningData.begin(), mLearningData.end(), [&] (cv::Mat& data)
    {
        cv::Mat converted;
        data.convertTo(converted, CV_32F);
        cv::absdiff(mean, converted, converted);
        cv::pow(converted, 2.0, data);
    });
    for_each (mLearningData.begin(), mLearningData.end(), [&] (cv::Mat& data)
    {
        stddev += data;
    });
    cv::sqrt(stddev / (float)mLearningTime, stddev);

    // If the stddev is superior to mDetectionDistance / 3, we mark the pixel as not measured
    cv::Mat mask;
    cv::threshold(stddev, mask, mDetectionDistance / 3.0, 1.0, cv::THRESH_BINARY_INV);

    mBackgroundMean = mean.mul(mask);
    mBackgroundStddev = stddev;
}

/*************/
void Actuator_DepthTouch::setParameter(atom::Message pMessage)
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
    else if (cmd == "detectionDistance")
    {
        float distance;
        if (readParam(pMessage, distance))
            mDetectionDistance = max(1.f, (float)distance);
    }
    else if (cmd == "stddevCoeff")
    {
        float coeff;
        if (readParam(pMessage, coeff))
            mSigmaCoeff = max(10.f, (float)coeff);
    }
    else if (cmd == "learningTime")
    {
        float time;
        if (readParam(pMessage, time))
            mLearningTime = max(0.f, time);
    }
    else if (cmd == "learn")
    {
        mLearningLeft = mLearningTime;
        mIsLearning = true;
    }
    else
        setBaseParameter(pMessage);
}
