#include "detector_meanOutliers.h"

using namespace atom;

std::string Detector_MeanOutliers::mClassName = "Detector_MeanOutliers";
std::string Detector_MeanOutliers::mDocumentation = "N/A";

/*************/
Detector_MeanOutliers::Detector_MeanOutliers()
{
    make();
}

/*************/
Detector_MeanOutliers::Detector_MeanOutliers(int pParam)
{
    make();
}

/*************/
void Detector_MeanOutliers::make()
{
    isInitialized = false;
    mDetectionLevel = 2.f;
    mFilterSize = 3;

    mName = mClassName;
    // OSC path for this detector
    mOscPath = "/blobserver/meanOutliers";

    mMeanBlob.setParameter("processNoiseCov", 1e-6);
    mMeanBlob.setParameter("measurementNoiseCov", 1e-4);
}

/*************/
atom::Message Detector_MeanOutliers::detect(std::vector<cv::Mat> pCaptures)
{
    cv::Mat lMean, lStdDev;
    cv::Mat lOutlier, lEroded, lFiltered;

    // Eliminate the outliers : calculate the mean and std dev
    lOutlier = cv::Mat::zeros(pCaptures[0].size[0], pCaptures[0].size[1], CV_8U);
    lEroded = lOutlier.clone();
    lFiltered = lOutlier.clone();
    cv::cvtColor(pCaptures[0], lOutlier, CV_RGB2GRAY);

    cv::meanStdDev(pCaptures[0], lMean, lStdDev);
    cv::absdiff(lOutlier, lMean.at<double>(0), lOutlier);

    // Detect pixels far from the mean (> 2*stddev)
    cv::threshold(lOutlier, lOutlier, mDetectionLevel * lStdDev.at<double>(0), 255, cv::THRESH_BINARY);

    // Erode and dilate to suppress noise
    cv::erode(lOutlier, lEroded, cv::Mat(), cv::Point(-1, -1), mFilterSize);
    cv::dilate(lEroded, lFiltered, cv::Mat(), cv::Point(-1, -1), mFilterSize);

    // Apply the mask
    cv::Mat lMask = getMask(lFiltered, CV_INTER_LINEAR);
    for (int x = 0; x < lFiltered.cols; ++x)
        for (int y = 0; y < lFiltered.rows; ++y)
        {
            if (lMask.at<uchar>(y, x) == 0)
                lFiltered.at<uchar>(y, x) = 0;
        }
 
    // Save the result in a buffer
    mOutputBuffer = lFiltered;

    // Calculate the barycenter of the outliers
    int lNumber = 0;
    int lX = 0, lY = 0;

    for (int x = 0; x < lFiltered.size[1]; ++x)
        for (int y = 0; y < lFiltered.size[0]; ++y)
        {
            if (lFiltered.at<uchar>(y, x) == 255)
            {
                lX += x;
                lY += y;
                lNumber++;
            }
        }

    bool isDetected = false;
    if(lNumber > 0)
    {
        isDetected = true;
        lX /= lNumber;
        lY /= lNumber;
    }
    else
    {
        lX = lFiltered.size[1] / 2;
        lY = lFiltered.size[0] / 2;
    }

    // Filtering using a Blob2D
    Blob::properties props;
    props.position.x = lX;
    props.position.y = lY;
    props.size = lNumber;

    if(!isInitialized)
    {
        mMeanBlob.init(props);
        isInitialized = true;
    }
    else
    {
        mMeanBlob.predict();
        mMeanBlob.setNewMeasures(props);
        props = mMeanBlob.getBlob();
    }

    // Extract values computed by the filter
    lX = (int)(props.position.x);
    lY = (int)(props.position.y);
    lNumber = (int)(props.size);
    int lSpeedX = (int)(props.speed.x);
    int lSpeedY = (int)(props.speed.y);

    // Constructing the message
    Message message;
    // Two first values are the number and size of each (the...) blob
    message = createMessage("iiiiiii", 1, 5, lX, lY, lNumber, lSpeedX, lSpeedY);

    return message;
}

/*************/
void Detector_MeanOutliers::setParameter(atom::Message pMessage)
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
            float param = FloatValue::convert(*iter)->getFloat();
            mMeanBlob.setParameter("processNoiseCov", param);
        }
        else if (cmd == "setMeasurementNoiseCov" && (*iter).get()->getTypeTag() == FloatValue::TYPE_TAG)
        {
            float param = FloatValue::convert(*iter)->getFloat();
            mMeanBlob.setParameter("measurementNoiseCov", param);
        }
    }
}
