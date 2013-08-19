#include "actuator_fiducialtracker.h"

using namespace std;

std::string Actuator_FiducialTracker::mClassName = "Actuator_FiducialTracker";
std::string Actuator_FiducialTracker::mDocumentation = "N/A";
unsigned int Actuator_FiducialTracker::mSourceNbr = 1;

/*************/
Actuator_FiducialTracker::Actuator_FiducialTracker()
{
    make();
}

/*************/
Actuator_FiducialTracker::Actuator_FiducialTracker(int pParam)
{
    make();
}

/*************/
void Actuator_FiducialTracker::make()
{
    mWidth = 640;
    mHeight = 480;

    mOutputBuffer = cv::Mat::zeros(mHeight, mWidth, CV_8UC3);

    mName = mClassName;
    // OSC path for this actuator
    mOscPath = "/blobserver/fiducialtracker";

    mFrameNumber = 0;

    // Initialize libfidtrack objects
    initFidtracker();
}

/*************/
void Actuator_FiducialTracker::initFidtracker()
{
    if (mDmap != NULL)
        delete mDmap;
    mDmap = new ShortPoint[mWidth * mHeight];
    initialize_treeidmap(&mFidTreeidmap);
    initialize_segmenter(&mFidSegmenter, mWidth, mHeight, mFidTreeidmap.max_adjacencies);
    initialize_fidtrackerX(&mFidTrackerx, &mFidTreeidmap, mDmap);

    for (int x = 0; x < mWidth; ++x)
        for (int y = 0; y < mHeight; ++y)
        {
            mDmap[y*mWidth+x].x = x;
            mDmap[y*mWidth+x].y = y;
        }
}

/*************/
atom::Message Actuator_FiducialTracker::detect(vector< Capture_Ptr > pCaptures)
{
    vector<cv::Mat> captures = captureToMat(pCaptures);
    if (captures.size() < mSourceNbr)
        return mLastMessage;

    cv::Mat input = captures[0];

    if (input.rows != mHeight || input.cols != mWidth)
    {
        mHeight = input.rows;
        mWidth = input.cols;
        initFidtracker();
    }

    cv::Mat gray;
    cv::cvtColor(input, gray, CV_RGB2GRAY);
    int roi_size = 32;
    for (int pX = 0; pX < mWidth - 1; pX += roi_size)
        for (int pY = 0; pY < mHeight - 1; pY += roi_size)
        {
            cv::Rect roi;
            roi.x = pX;
            roi.y = pY;
            roi.width = min(roi_size, (int)(mWidth - pX));
            roi.height = min(roi_size, (int)(mHeight - pY));

            cv::Mat tmpImg(gray, roi);
            double min, max;
            cv::minMaxIdx(tmpImg, &min, &max);
            int lvl = (int)((min + max) / 2.0);

            for (int x = 0; x < roi.width; ++x)
                for (int y = 0; y < roi.height; ++y)
                {
                    if (tmpImg.at<uchar>(y, x) < lvl)
                        tmpImg.at<uchar>(y, x) = 0;
                    else
                        tmpImg.at<uchar>(y, x) = 255;
                }
        }

    mOutputBuffer = gray.clone();

    step_segmenter(&mFidSegmenter, (unsigned char*)gray.data);
    int fidCount = find_fiducialsX(mFiducials, MAX_FIDUCIAL_COUNT, &mFidTrackerx, &mFidSegmenter, mWidth, mHeight);
    g_log(NULL, G_LOG_LEVEL_DEBUG, "%s: Number of marker found: %i", mClassName.c_str(), fidCount);

    mLastMessage.clear();
    mLastMessage.push_back(atom::IntValue::create(fidCount));
    mLastMessage.push_back(atom::IntValue::create(4));

    for (int i = 0; i < fidCount; ++i)
    {
        char buffer[16];
        sprintf(buffer, "%i", mFiducials[i].id);
        cv::putText(mOutputBuffer, string(buffer), cv::Point(mFiducials[i].x, mFiducials[i].y), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(128), 3);
        cv::putText(mOutputBuffer, string(buffer), cv::Point(mFiducials[i].x, mFiducials[i].y), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255), 1);

        mLastMessage.push_back(atom::IntValue::create(mFiducials[i].id));
        mLastMessage.push_back(atom::FloatValue::create(mFiducials[i].x));
        mLastMessage.push_back(atom::FloatValue::create(mFiducials[i].y));
        mLastMessage.push_back(atom::FloatValue::create(mFiducials[i].angle));
    }

    mFrameNumber++;

    return mLastMessage;
}

/*************/
void Actuator_FiducialTracker::setParameter(atom::Message pMessage)
{
    setBaseParameter(pMessage);
}
