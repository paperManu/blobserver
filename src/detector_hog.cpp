#include "detector_hog.h"

using namespace std;

std::string Detector_Hog::mClassName = "Detector_Hog";
std::string Detector_Hog::mDocumentation = "N/A";
unsigned int Detector_Hog::mSourceNbr = 1;

/*************/
Detector_Hog::Detector_Hog()
{
    make();
}

/*************/
Detector_Hog::Detector_Hog(int pParam)
{
    make();
}

/*************/
void Detector_Hog::make()
{
    mOutputBuffer = cv::Mat::zeros(480, 640, CV_8UC3);

    mName = mClassName;
    mOscPath = "/blobserver/hog";

    mFilterSize = 3;

    mRoiSize = cv::Point_<int>(64, 128);
    mBlockSize = cv::Point_<int>(2, 2);
    mCellSize = cv::Point_<int>(8, 8);
    mBins = 9;
    mSigma = 0.f;
    updateDescriptorParams();
}

/*************/
atom::Message Detector_Hog::detect(vector<cv::Mat> pCaptures)
{
    if (pCaptures.size() == 0)
        return mLastMessage;

    // We first get windows of interest, using BG subtraction
    // and previous blobs positions
    mBgSubtractor(pCaptures[0], mBgSubtractorBuffer);
    // Erode and dilate to suppress noise
    cv::Mat lEroded;
    cv::erode(mBgSubtractorBuffer, lEroded, cv::Mat(), cv::Point(-1, -1), mFilterSize);
    cv::dilate(lEroded, mBgSubtractorBuffer, cv::Mat(), cv::Point(-1, -1), mFilterSize);

    cv::threshold(mBgSubtractorBuffer, mBgSubtractorBuffer, 250, 255, cv::THRESH_BINARY);

    // Draw positions of existing blobs
    for_each (mBlobs.begin(), mBlobs.end(), [&] (Blob2D blob)
    {
        Blob::properties props = blob.getBlob();
        cv::Rect rect(props.position.x, props.position.y, props.size, props.size);
        cv::rectangle(mBgSubtractorBuffer, rect, 255, CV_FILLED);
    } );

    // The result is resized according to cell size
    cv::Size outputSize;
    outputSize.width = mBgSubtractorBuffer.cols / mCellSize.width;
    outputSize.height = mBgSubtractorBuffer.rows / mCellSize.height;
    cv::Mat resizedBuffer;
    cv::resize(mBgSubtractorBuffer, resizedBuffer, outputSize, 0, 0, cv::INTER_NEAREST);

    // We feed the image to the descriptor
    mDescriptor.setImage(pCaptures[0]);

    // We can now go through the frame to test windows
    vector<float> description;
    cv::Mat descriptionMat;
    for (int x = 0; x < resizedBuffer.cols; ++x)
        for (int y = 0; y < resizedBuffer.rows; ++y)
        {
            cv::Point_<int> pos;
            pos.x = x * mCellSize.width;
            pos.y = y * mCellSize.height;
            if (resizedBuffer.at<char>(x, y) < 127)
                continue;

            // Get the descriptor for thhis position
            description = mDescriptor.getDescriptor(pos);
            descriptionMat = cv::Mat(1, description.size(), CV_32FC1, &description[0]);

            if (description.size() == 0)
                continue;

            float value = mSvm.predict(descriptionMat);
        }

    mOutputBuffer = resizedBuffer.clone();

    return mLastMessage;
}

/*************/
void Detector_Hog::setParameter(atom::Message pMessage)
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

    setBaseParameter(pMessage);
}

/*************/
void Detector_Hog::updateDescriptorParams()
{
    mDescriptor.setHogParams(mRoiSize, mBlockSize, mCellSize, mBins, false, Descriptor_Hog::L2_NORM, mSigma);
}
