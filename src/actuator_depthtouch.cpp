#include "actuator_depthtouch.h"

#include <algorithm>
#include <chrono>
#include <limits>
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

#define MAX_DEPTH 65535.f

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
    mOscPath = "depthtouch";

    mFilterSize = 2;
    mDetectionDistance = 100.f;
    mSigmaCoeff = 20.f;
    mClickDistance = 20.f;

    mBlobLifetime = 1;
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
    distance += mask * MAX_DEPTH;

    // Comparing distance with detection distance, plus some morphological operations
    cv::Mat touch;
    cv::Mat touchDistance = cv::max((mBackgroundStddev + 1.0) * mSigmaCoeff, mDetectionDistance);
    cv::compare(distance, touchDistance, touch, cv::CMP_LE);
    cv::Mat lEroded;
    cv::erode(touch, lEroded, cv::Mat(), cv::Point(-1, -1), 3);
    cv::dilate(lEroded, touch, cv::Mat(), cv::Point(-1, -1), mFilterSize);

    vector< vector<cv::Point> > contours;
    cv::Mat buffer = touch.clone();
    cv::findContours(buffer, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    vector<Property> properties;
    for (unsigned int i = 0; i < contours.size(); ++i)
    {
        cv::Rect box = cv::boundingRect(contours[i]);
        float area = cv::contourArea(contours[i], false);

        cv::Mat contourImg = cv::Mat::zeros(input.size(), CV_32F);
        cv::drawContours(contourImg, contours, i, cv::Scalar(1.f), CV_FILLED);
        cv::erode(contourImg, lEroded, cv::Mat(), cv::Point(-1, -1), mFilterSize);
        contourImg = lEroded;
        cv::Mat inverseContour = cv::Mat::zeros(input.size(), CV_32F);
        inverseContour = (1.f - contourImg) * MAX_DEPTH;
        contourImg = cv::max(contourImg.mul(distance), inverseContour);

        // We are looking for the extremity of the hand (ideally, the fingers)
        // So we search the part of the contour which is the nearest to the surface
        cv::Mat roi = contourImg(box);
        float contourMin = MAX_DEPTH;
        float contourMax = 0.f;
        for (unsigned int x = 0; x < roi.cols; ++x)
            for (unsigned int y = 0; y < roi.rows; ++y)
            {
                if (roi.at<float>(y, x) < contourMin && roi.at<float>(y, x) > 0.f)
                    contourMin = roi.at<float>(y, x);
                if (roi.at<float>(y, x) > contourMax && roi.at<float>(y, x) < mDetectionDistance)
                    contourMax = roi.at<float>(y, x);
            }

        // We get the mean position of points which are in the first tenth of [contourMin, contourMax]
        float limit = contourMin + (contourMax - contourMin) / 10.f;
        cv::Point2f fingers;
        fingers.x = 0.f;
        fingers.y = 0.f;
        float nbrPoints = 0.f;
        float meanDistance = 0.f; // Used to detect contact with the surface
        for (unsigned int x = 0; x < roi.cols; ++x)
            for (unsigned int y = 0; y < roi.rows; ++y)
            {
                if (roi.at<float>(y, x) < limit)
                {
                    fingers.x += (float)(x + box.x);
                    fingers.y += (float)(y + box.y);
                    meanDistance += roi.at<float>(y, x);
                    nbrPoints++;
                }
            }
        fingers.x /= nbrPoints;
        fingers.y /= nbrPoints;
        meanDistance /= nbrPoints;

        // We do the same to get the part of the contour which is the farthest from the surface
        limit = contourMax - (contourMax - contourMin) / 5.f;
        cv::Point2f wrist;
        wrist.x = 0.f;
        wrist.y = 0.f;
        nbrPoints = 0.f;
        for (unsigned int x = 0; x < roi.cols; ++x)
            for (unsigned int y = 0; y < roi.rows; ++y)
            {
                if (roi.at<float>(y, x) > limit && roi.at<float>(y, x) <= contourMax)
                {
                    wrist.x += (float)(x + box.x);
                    wrist.y += (float)(y + box.y);
                    nbrPoints++;
                }
            }
        wrist.x /= nbrPoints;
        wrist.y /= nbrPoints;

        // The finger point is moved a bit in the direction of the vector (wrist, fingers)
        cv::Vec2f direction;
        direction = fingers - wrist;
        direction /= norm(direction);
        fingers.x += direction[0] * 16.f;
        fingers.y += direction[1] * 16.f;

        // We store these information
        Property property;
        property.position.x = fingers.x;
        property.position.y = fingers.y;
        property.wrist.x = wrist.x;
        property.wrist.y = wrist.y;
        property.size = area;
        property.speed.x = 0.f;
        property.speed.y = 0.f;
        if (meanDistance < mClickDistance)
            property.contact = 1;
        else
            property.contact = 0;

        properties.push_back(property);
    }

    // We want to track them
    vector<Blob::properties> blobProps;
    for (int i = 0; i < properties.size(); ++i)
    {
        Blob::properties prop;
        prop.position.x = properties[i].position.x;
        prop.position.y = properties[i].position.y;
        prop.size = properties[i].size;
        prop.speed.x = properties[i].speed.x;
        prop.speed.y = properties[i].speed.y;

        blobProps.push_back(prop);
    }
    trackBlobs<Blob2D>(blobProps, mBlobs, mBlobLifetime);

    // We make sure that the filtering parameters are set
    for (int i = 0; i < mBlobs.size(); ++i)
    {
        mBlobs[i].setParameter("processNoiseCov", mProcessNoiseCov);
        mBlobs[i].setParameter("measurementNoiseCov", mMeasurementNoiseCov);
    }

    sort(properties.begin(), properties.end(), [&] (Blob::properties a, Blob::properties b)
    {
        return a.size > b.size;
    });

    sort(mBlobs.begin(), mBlobs.end(), [&] (Blob2D a, Blob2D b)
    {
        Blob::properties propA = a.getBlob();
        Blob::properties propB = b.getBlob();
        return propA.size > propB.size;
    });

    // Constructing the message
    mLastMessage.clear();
    mLastMessage.push_back(atom::IntValue::create((int)properties.size()));
    mLastMessage.push_back(atom::IntValue::create(6));

    for(int i = 0; i < properties.size(); ++i)
    {
        Blob::properties blobProp = mBlobs[i].getBlob();

        float lX, lY, lWX, lWY, ldX, ldY;;
        int contact;
        lX = (blobProp.position.x);
        lY = (blobProp.position.y);
        lWX = (properties[i].wrist.x);
        lWY = (properties[i].wrist.y);
        ldX = blobProp.speed.x;
        ldY = blobProp.speed.y;
        contact = properties[i].contact;

        // Print the blob number on the blob
        if (mVerbose)
        {
            char lNbrStr[8];
            sprintf(lNbrStr, "%i", i);
            cv::line(touch, cv::Point(lX, lY), cv::Point(lWX, lWY), cv::Scalar(128.0), 2);
            cv::putText(touch, lNbrStr, cv::Point(lWX, lWY), cv::FONT_HERSHEY_COMPLEX, 0.8, cv::Scalar(128.0, 128.0, 128.0, 128.0));
            if (properties[i].contact)
                cv::putText(touch, lNbrStr, cv::Point(lX, lY), cv::FONT_HERSHEY_COMPLEX, 0.8, cv::Scalar(128.0, 128.0, 128.0, 128.0));
        }

        // Add this blob to the message
        mLastMessage.push_back(atom::IntValue::create(i));
        mLastMessage.push_back(atom::FloatValue::create(lX));
        mLastMessage.push_back(atom::FloatValue::create(lY));
        mLastMessage.push_back(atom::FloatValue::create(ldX));
        mLastMessage.push_back(atom::FloatValue::create(ldY));
        mLastMessage.push_back(atom::IntValue::create(contact));
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
    for_each (mLearningData.begin(), mLearningData.end(), [&] (cv::Mat& data)
    {
        mean += data;
    });
    mean /= (float)mLearningTime;

    // Calculate the standard deviation
    cv::Mat stddev = cv::Mat::zeros(mLearningData[0].rows, mLearningData[1].cols, CV_32F);
    for_each (mLearningData.begin(), mLearningData.end(), [&] (cv::Mat& data)
    {
        cv::absdiff(mean, data, data);
        cv::pow(data, 2.0, data);
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
    else if (cmd == "clickDistance")
    {
        float distance;
        if (readParam(pMessage, distance))
            mClickDistance = max(-1.f, distance);
    }
    else if (cmd == "learn")
    {
        mLearningLeft = mLearningTime;
        mIsLearning = true;
    }
    else
        setBaseParameter(pMessage);
}
