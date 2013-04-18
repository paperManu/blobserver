#include "detector_lightSpots.h"

using namespace std;

std::string Detector_LightSpots::mClassName = "Detector_LightSpots";
std::string Detector_LightSpots::mDocumentation = "N/A";
unsigned int Detector_LightSpots::mSourceNbr = 1;

/*************/
Detector_LightSpots::Detector_LightSpots()
{
    make();
}

/*************/
Detector_LightSpots::Detector_LightSpots(int pParam)
{
    make();
}

/*************/
void Detector_LightSpots::make()
{
    mDetectionLevel = 2.f;
    mFilterSize = 3;
    mMaxTrackedBlobs = 8;

    mName = mClassName;
    // OSC path for this detector
    mOscPath = "/blobserver/lightSpots";

    mProcessNoiseCov = 1e-5;
    mMeasurementNoiseCov = 1e-5;

    // Set mLightBlobDetector to indeed detect light
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
atom::Message Detector_LightSpots::detect(const std::vector<cv::Mat> pCaptures)
{
    cv::Mat lMean, lStdDev;
    cv::Mat lOutlier, lLight;
    cv::Mat lEroded;
    std::vector<cv::KeyPoint> lKeyPoints;

    // Eliminate the outliers : calculate the mean and std dev
    lOutlier = cv::Mat::zeros(pCaptures[0].size[0], pCaptures[0].size[1], CV_8U);
    lLight = lOutlier.clone();
    cv::cvtColor(pCaptures[0], lOutlier, CV_RGB2GRAY);

    cv::meanStdDev(pCaptures[0], lMean, lStdDev);
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

    // Apply the mask
    cv::Mat lMask = getMask(lLight, CV_INTER_NN);
    for (int x = 0; x < lLight.cols; ++x)
        for (int y = 0; y < lLight.rows; ++y)
        {
            if (lMask.at<uchar>(y, x) == 0)
                lLight.at<uchar>(y, x) = 0;
        }

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

    // And we send and print them
    // Include the number and size of each blob in the message
    mLastMessage.clear();
    mLastMessage.push_back(atom::IntValue::create((int)mLightBlobs.size()));
    mLastMessage.push_back(atom::IntValue::create(6));

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
        if (mVerbose)
        {
            char lNbrStr[8];
            sprintf(lNbrStr, "%i", lId);
            cv::putText(lLight, lNbrStr, cv::Point(lX, lY), cv::FONT_HERSHEY_COMPLEX, 0.66, cv::Scalar(128.0, 128.0, 128.0, 128.0));
        }

        // Add this blob to the message
        mLastMessage.push_back(atom::IntValue::create(lX));
        mLastMessage.push_back(atom::IntValue::create(lY));
        mLastMessage.push_back(atom::IntValue::create(lSize));
        mLastMessage.push_back(atom::IntValue::create(ldX));
        mLastMessage.push_back(atom::IntValue::create(ldY));
        mLastMessage.push_back(atom::IntValue::create(lId));
    }

    // Save the result in a buffer
    mOutputBuffer = lLight;

    return mLastMessage;
}

/*************/
void Detector_LightSpots::setParameter(atom::Message pMessage)
{
    atom::Message::const_iterator iter = pMessage.begin();

    std::string cmd;
    try
    {
        cmd = toString(pMessage[0]);
    }
    catch (atom::BadTypeTagError error)
    {
        return;
    }

    if (cmd == "detectionLevel")
    {
        float level;
        if (readParam(pMessage, level))
            mDetectionLevel = max(0.f, level);
    }
    else if (cmd == "filterSize")
    {
        float size;
        if (readParam(pMessage, size))
            mFilterSize = max(0.f, size);
    }
    else if (cmd == "processNoiseCov")
    {
        readParam(pMessage, mProcessNoiseCov);
    }
    else if (cmd == "measurementNoiseCov")
    {
        readParam(pMessage, mMeasurementNoiseCov);
    }
    else
        setBaseParameter(pMessage);
}
