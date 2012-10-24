#include "detector_lightSpots.h"

using namespace atom;

/*************/
Detector_LightSpots::Detector_LightSpots()
    :mDetectionLevel (2.f),
    mFilterSize (3),
    mMaxTrackedBlobs (8)
{
    mProcessNoiseCov = 1e-5;
    mMeasurementNoiseCov = 1e-5;

    // Set mLightDetector to indeed detect light
    cv::SimpleBlobDetector::Params lParams;
    lParams.filterByColor = true;
    lParams.blobColor = 255;
    lParams.minCircularity = 0.1f;
    lParams.maxCircularity = 1.f;
    lParams.minInertiaRatio = 0.f;
    lParams.maxInertiaRatio = 1.f;
    lParams.minArea = 0.f;
    lParams.maxArea = 65535.f;
    mLightBlobDetector = new cv::SimpleBlobDetector(lParams);
}

/*************/
atom::Message Detector_LightSpots::detect(cv::Mat pCapture)
{
    cv::Mat lMean, lStdDev;
    cv::Mat lOutlier, lLight;
    cv::Mat lEroded;
    std::vector<cv::KeyPoint> lKeyPoints;

    // Eliminate the outliers : calculate the mean and std dev
    lOutlier = cv::Mat::zeros(pCapture.size[0], pCapture.size[1], CV_8U);
    lLight = lOutlier.clone();
    cv::cvtColor(pCapture, lOutlier, CV_RGB2GRAY);

    cv::meanStdDev(pCapture, lMean, lStdDev);
    cv::absdiff(lOutlier, lMean.at<double>(0), lOutlier);

    // Detect pixels which values are superior to the mean
    cv::threshold(lOutlier, lLight, lMean.at<double>(0), 255, cv::THRESH_BINARY);
    
    // Detect pixels far from the mean (> 2*stddev by default)
    cv::threshold(lOutlier, lOutlier, mDetectionLevel * lStdDev.at<double>(0), 255, cv::THRESH_BINARY);
    
    // Combinaison of both previous conditions
    cv::bitwise_and(lOutlier, lLight, lLight);

    // Erode and dilate to suppress noise
    cv::erode(lLight, lEroded, cv::Mat(), cv::Point(-1, -1), mFilterSize);
    cv::dilate(lEroded, lLight, cv::Mat(), cv::Point(-1, -1), mFilterSize);

    // Now we have to detect blobs
    mLightBlobDetector->detect(lLight, lKeyPoints);
    
    // We use Blob::properties, not cv::KeyPoints
    std::vector<Blob::properties> lProperties;
    for(int i = 0; i < std::min((int)(lKeyPoints.size()), mMaxTrackedBlobs); ++i)
    {
        Blob::properties properties;
        properties.position.x = lKeyPoints[i].pt.x;
        properties.position.y = lKeyPoints[i].pt.y;
        properties.size = lKeyPoints[i].size;
        properties.speed.x = 0.f;
        properties.speed.y = 0.f;

        lProperties.push_back(properties);
    }

    // We want to track them
    trackBlobs<Blob2D>(lProperties, mLightBlobs);

    // Make sure their covariances are correctly set
    std::vector<Blob2D>::iterator lIt = mLightBlobs.begin();
    for(; lIt != mLightBlobs.end(); ++lIt)
    {
        lIt->setParameter("processNoiseCov", mProcessNoiseCov);
        lIt->setParameter("measurementNoiseCov", mMeasurementNoiseCov);
    }

    //if(gVerbose)
    //    std::cout << "--- Light blobs detection:" << std::endl;

    // And we send and print them
    Message message;
    // Include the number and size of each blob in the message
    message.push_back(IntValue::create((int)mLightBlobs.size()));
    message.push_back(IntValue::create(6));

    for(int i = 0; i < mLightBlobs.size(); ++i)
    {
        int lX, lY, lSize, ldX, ldY, lId;
        Blob::properties properties = mLightBlobs[i].getBlob();
        lX = (int)(properties.position.x);
        lY = (int)(properties.position.y);
        lSize = (int)(properties.size);
        ldX = (int)(properties.speed.x);
        ldY = (int)(properties.speed.y);
        lId = (int)mLightBlobs[i].getId();

        // Print the blob number on the blob
        char lNbrStr[8];
        sprintf(lNbrStr, "%i", lId);
        cv::putText(lLight, lNbrStr, cv::Point(lX, lY), cv::FONT_HERSHEY_COMPLEX, 0.66, cv::Scalar(128.0, 128.0, 128.0, 128.0));

        // Add this blob to the message
        message.push_back(IntValue::create(lX));
        message.push_back(IntValue::create(lY));
        message.push_back(IntValue::create(lSize));
        message.push_back(IntValue::create(ldX));
        message.push_back(IntValue::create(ldY));
        message.push_back(IntValue::create(lId));
    }

    // Save the result in a buffer
    mOutputBuffer = lLight;

    return message;
}

/*************/
void Detector_LightSpots::setParameter(atom::Message pMessage)
{
    Message::const_iterator iter = pMessage.begin();

    if ((*iter).get()->getTypeTag() == StringValue::TYPE_TAG)
    {
        std::string cmd = StringValue::convert(*iter)->getString();

        ++iter;
        if (iter == pMessage.end())
            return;

        if (cmd == "setDetectionLevel" && (*iter).get()->getTypeTag() == FloatValue::TYPE_TAG)
        {
            float param = FloatValue::convert(*iter)->getFloat();
            mDetectionLevel = std::max(0.f, param);
        }
        else if (cmd == "setFilterSize" && (*iter).get()->getTypeTag() == FloatValue::TYPE_TAG)
        {
            float param = FloatValue::convert(*iter)->getFloat();
            mFilterSize= std::max(0.f, param);
        }
        else if (cmd == "setProcessNoiseCov" && (*iter).get()->getTypeTag() == FloatValue::TYPE_TAG)
        {
            mProcessNoiseCov = FloatValue::convert(*iter)->getFloat();
        }
        else if (cmd == "setMeasurementNoiseCov" && (*iter).get()->getTypeTag() == FloatValue::TYPE_TAG)
        {
            mMeasurementNoiseCov = FloatValue::convert(*iter)->getFloat();
        }
    }
}
