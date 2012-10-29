#include "source_opencv.h"

std::string Source_OpenCV::mClassName = "Source_OpenCV";
std::string Source_OpenCV::mDocumentation = "N/A";

/*************/
Source_OpenCV::Source_OpenCV()
{
    mName = mClassName;
    mBuffer = cv::Mat::zeros(0, 0, CV_8U);
}

/*************/
Source_OpenCV::Source_OpenCV(int pParam)
{
    Source_OpenCV();
}

/*************/
Source_OpenCV::~Source_OpenCV()
{
    disconnect();
}

/*************/
bool Source_OpenCV::connect()
{
    mCamera.open(mSubsourceNbr);
    if (!mCamera.isOpened())
    {
        return false;
    }

    // Get current camera parameters
    mWidth = (unsigned int)(mCamera.get(CV_CAP_PROP_FRAME_WIDTH));
    mHeight = (unsigned int)(mCamera.get(CV_CAP_PROP_FRAME_HEIGHT));
    mFramerate = (unsigned int)(mCamera.get(CV_CAP_PROP_FPS));
    
    int channels;
    channels = (int)(mCamera.get(CV_CAP_PROP_FORMAT));
    mFramerate = (unsigned int)((channels >> 3) + 1); // See CV_MAKETYPE in types_c.h in OpenCV

    return true;    
}

/*************/
bool Source_OpenCV::disconnect()
{
    mCamera.release();
    return true;
}

/*************/
bool Source_OpenCV::grabFrame()
{
    bool result = mCamera.grab();
    mUpdated = true;
    return result;
}

/*************/
cv::Mat Source_OpenCV::retrieveFrame()
{
    if (mUpdated)
        mCamera.retrieve(mBuffer);

    mUpdated = false;
    return mBuffer.clone();
}

/*************/
void Source_OpenCV::setParameter(const char* pParam, float pValue)
{
    if (strcmp(pParam, "width") == 0)
    {
        mCamera.set(CV_CAP_PROP_FRAME_WIDTH, pValue);
        mWidth = (unsigned int)(mCamera.get(CV_CAP_PROP_FRAME_WIDTH));
    }
    else if (strcmp(pParam, "height") == 0)
    {
        mCamera.set(CV_CAP_PROP_FRAME_HEIGHT, pValue);
        mHeight = (unsigned int)(mCamera.get(CV_CAP_PROP_FRAME_HEIGHT));
    }
    else if (strcmp(pParam, "framerate") == 0)
    {
        mCamera.set(CV_CAP_PROP_FPS, pValue);
        mFramerate = (unsigned int)(mCamera.get(CV_CAP_PROP_FPS));
    }
    else if (strcmp(pParam, "camera number") == 0)
    {
        mSubsourceNbr = (unsigned int)pValue;
    }
}

/*************/
void Source_OpenCV::setParameter(atom::Message pParam)
{
    if (pParam.size() < 2)
        return;

    std::string paramName;
    float paramValue;

    try
    {
        paramName = atom::toString(pParam[0]);
        paramValue = atom::toFloat(pParam[1]);
    }
    catch (atom::BadTypeTagError exception)
    {
        return;
    }

    if (paramName == "width")
    {
        mCamera.set(CV_CAP_PROP_FRAME_WIDTH, paramValue);
        mWidth = (unsigned int)(mCamera.get(CV_CAP_PROP_FRAME_WIDTH));
    }
    else if (paramName == "height")
    {
        mCamera.set(CV_CAP_PROP_FRAME_HEIGHT, paramValue);
        mHeight = (unsigned int)(mCamera.get(CV_CAP_PROP_FRAME_HEIGHT));
    }
    else if (paramName == "framerate")
    {
        mCamera.set(CV_CAP_PROP_FPS, paramValue);
        mFramerate = (unsigned int)(mCamera.get(CV_CAP_PROP_FPS));
    }
    else if (paramName == "camera number")
    {
        mSubsourceNbr = (unsigned int)paramValue;
    }
}
