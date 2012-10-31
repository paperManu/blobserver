#include "source_opencv.h"

std::string Source_OpenCV::mClassName = "Source_OpenCV";
std::string Source_OpenCV::mDocumentation = "N/A";

/*************/
Source_OpenCV::Source_OpenCV()
{
    make(0);
}

/*************/
Source_OpenCV::Source_OpenCV(int pParam)
{
    make(pParam);
}

/*************/
void Source_OpenCV::make(int pParam)
{
    mName = mClassName;
    mSubsourceNbr = pParam;
    mBuffer = cv::Mat::zeros(0, 0, CV_8U);
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

    mId = (unsigned int)(mCamera.get(CV_CAP_PROP_GUID));

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

/*************/
atom::Message Source_OpenCV::getParameter(atom::Message pParam)
{
    atom::Message msg;

    if (pParam.size() < 1)
        return msg;

    std::string paramName;
    try
    {
        paramName = atom::toString(pParam[0]);
    }
    catch (atom::BadTypeTagError exception)
    {
        return msg;
    }

    msg.push_back(pParam[0]);
    if (paramName == "width")
        msg.push_back(atom::IntValue::create(mWidth));
    else if (paramName == "height")
        msg.push_back(atom::IntValue::create(mWidth));
    else if (paramName == "framerate")
        msg.push_back(atom::IntValue::create(mFramerate));
    else if (paramName == "subsourcenbr")
        msg.push_back(atom::IntValue::create(mSubsourceNbr));
    else if (paramName == "id")
        msg.push_back(atom::FloatValue::create(mId));

    return msg;
}
