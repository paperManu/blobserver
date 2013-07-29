#include "detector_meanOutliers.h"

using namespace std;

std::string Detector_MeanOutliers::mClassName = "Detector_MeanOutliers";
std::string Detector_MeanOutliers::mDocumentation = "N/A";
unsigned int Detector_MeanOutliers::mSourceNbr = 1;

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
atom::Message Detector_MeanOutliers::detect(const vector< Capture_Ptr > pCaptures)
{
    vector<cv::Mat> captures = captureToMat(pCaptures);
    if (captures.size() < mSourceNbr)
        return mLastMessage;

    cv::Mat lMean, lStdDev;
    cv::Mat lOutlier, lEroded, lFiltered;

    // Eliminate the outliers : calculate the mean and std dev
    lOutlier = cv::Mat::zeros(captures[0].size[0], captures[0].size[1], CV_8U);
    lEroded = lOutlier.clone();
    lFiltered = lOutlier.clone();
    cv::cvtColor(captures[0], lOutlier, CV_RGB2GRAY);

    cv::meanStdDev(captures[0], lMean, lStdDev);
    cv::absdiff(lOutlier, lMean.at<double>(0), lOutlier);

    // Detect pixels far from the mean (> 2*stddev)
    cv::threshold(lOutlier, lOutlier, mDetectionLevel * lStdDev.at<double>(0), 255, cv::THRESH_BINARY);

    // Erode and dilate to suppress noise
    cv::erode(lOutlier, lEroded, cv::Mat(), cv::Point(-1, -1), mFilterSize);
    cv::dilate(lEroded, lFiltered, cv::Mat(), cv::Point(-1, -1), mFilterSize);

    // Apply the mask
    cv::Mat lMask = getMask(lFiltered, cv::INTER_NEAREST);
    cv::parallel_for_(cv::Range(0, lFiltered.rows), Parallel_Mask<uchar>(&lFiltered, &lMask));

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
    // Two first values are the number and size of each (the...) blob
    mLastMessage = atom::createMessage("iiiiiii", 1, 5, lX, lY, lNumber, lSpeedX, lSpeedY);

    // Save the result in a buffer
    if (mVerbose)
        cv::putText(lFiltered, string("x"), cv::Point(lX, lY), cv::FONT_HERSHEY_COMPLEX, 0.66, cv::Scalar(128.0, 128.0, 128.0, 128.0));

    mOutputBuffer = lFiltered;

    return mLastMessage;
}

/*************/
void Detector_MeanOutliers::setParameter(atom::Message pMessage)
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
        float cov;
        if (readParam(pMessage, cov))
            mMeanBlob.setParameter("processNoiseCov", cov);
    }
    else if (cmd == "measurementNoiseCov")
    {
        float cov;
        if (readParam(pMessage, cov))
            mMeanBlob.setParameter("measurementNoiseCov", cov);
    }
    else
        setBaseParameter(pMessage);
}
