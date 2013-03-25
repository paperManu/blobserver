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
}

/*************/
Source_OpenCV::~Source_OpenCV()
{
    disconnect();
}

/*************/
bool Source_OpenCV::connect()
{
    if (mSubsourceNbr == 0)
        return false;

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
    channels = (unsigned int)((channels >> 3) + 1); // See CV_MAKETYPE in types_c.h in OpenCV

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
    if (!mCamera.isOpened())
        return false;

    bool result = mCamera.grab();
    mUpdated = result;
    return result;
}

/*************/
cv::Mat Source_OpenCV::retrieveFrame()
{
    cv::Mat buffer;
    mCamera.retrieve(buffer);
    mBuffer = buffer;

    return mBuffer.get().clone();
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
    }
    catch (atom::BadTypeTagError exception)
    {
        return;
    }

    if (paramName == "width")
    {
        try
        {
            paramValue = atom::toFloat(pParam[1]);
        }
        catch (atom::BadTypeTagError exception)
        {
            return;
        }

        mCamera.set(CV_CAP_PROP_FRAME_WIDTH, paramValue);
        mWidth = (unsigned int)(mCamera.get(CV_CAP_PROP_FRAME_WIDTH));
    }
    else if (paramName == "height")
    {
        try
        {
            paramValue = atom::toFloat(pParam[1]);
        }
        catch (atom::BadTypeTagError exception)
        {
            return;
        }

        mCamera.set(CV_CAP_PROP_FRAME_HEIGHT, paramValue);
        mHeight = (unsigned int)(mCamera.get(CV_CAP_PROP_FRAME_HEIGHT));
    }
    else if (paramName == "framerate")
    {
        try
        {
            paramValue = atom::toFloat(pParam[1]);
        }
        catch (atom::BadTypeTagError exception)
        {
            return;
        }

        mCamera.set(CV_CAP_PROP_FPS, paramValue);
        mFramerate = (unsigned int)(mCamera.get(CV_CAP_PROP_FPS));
    }
    else if (paramName == "iso")
    {
        try
        {
            paramValue = atom::toFloat(pParam[1]);
        }
        catch (atom::BadTypeTagError exception)
        {
            return;
        }

        mCamera.set(CV_CAP_PROP_ISO_SPEED, paramValue);
    }
    else if (paramName == "exposureTime")
    {
        try
        {
            paramValue = atom::toFloat(pParam[1]);
        }
        catch (atom::BadTypeTagError exception)
        {
            return;
        }

        mCamera.set(CV_CAP_PROP_AUTO_EXPOSURE, 0);
        mCamera.set(CV_CAP_PROP_EXPOSURE, paramValue);
        mExposureTime = (float)(mCamera.get(CV_CAP_PROP_EXPOSURE));
        mExposureParam = paramValue;
    }
    else if (paramName == "gain")
    {
        try
        {
            paramValue = atom::toFloat(pParam[1]);
        }
        catch (atom::BadTypeTagError exception)
        {
            return;
        }

        mCamera.set(CV_CAP_PROP_GAIN, paramValue);
        mGain = (float)(mCamera.get(CV_CAP_PROP_GAIN));
    }
    else if (paramName == "gamma")
    {
        try
        {
            paramValue = atom::toFloat(pParam[1]);
        }
        catch (atom::BadTypeTagError exception)
        {
            return;
        }

        mCamera.set(CV_CAP_PROP_GAMMA, paramValue*1024.f);
        mGamma = (float)(mCamera.get(CV_CAP_PROP_GAMMA));
    }
    else if (paramName == "whiteBalanceRed")
    {
        try
        {
            paramValue = atom::toFloat(pParam[1]);
        }
        catch (atom::BadTypeTagError exception)
        {
            return;
        }

        mCamera.set(CV_CAP_PROP_WHITE_BALANCE_BLUE_U, paramValue);
    }
    else if (paramName == "whiteBalanceBlue")
    {
        try
        {
            paramValue = atom::toFloat(pParam[1]);
        }
        catch (atom::BadTypeTagError exception)
        {
            return;
        }

        mCamera.set(CV_CAP_PROP_WHITE_BALANCE_RED_V, paramValue);
    }
    else if (paramName == "cameraNumber")
    {
        try
        {
            paramValue = atom::toFloat(pParam[1]);
        }
        catch (atom::BadTypeTagError exception)
        {
            return;
        }

        mSubsourceNbr = (unsigned int)paramValue;
    }
    else
        setBaseParameter(pParam);
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
    else if (paramName == "exposureTime")
    {
        int value = mCamera.get(CV_CAP_PROP_EXPOSURE);
        mExposureTime = (float)value;
        msg.push_back(atom::IntValue::create(value));
    }
    else if (paramName == "subsourcenbr")
        msg.push_back(atom::IntValue::create(mSubsourceNbr));
    else
        msg = getBaseParameter(pParam);

    return msg;
}

/*************/
atom::Message Source_OpenCV::getSubsources()
{
    atom::Message message;

    // We need to test all possible CV sources
    int srcNbr = 13;
    int sources[] = {100, 200, 300, 400, 500, 600, 700, 800, 900, 910, 1000, 1100, 1200, 1300};

    for (int i = 0; i < srcNbr; ++i)
    {
        int index = 0;

        while (true)
        {
            cv::VideoCapture camera;

            bool available;
            available = camera.open(sources[i] + index);

            if (!available)
                break;

            message.push_back(atom::IntValue::create(sources[i] + index));

            unsigned int id;
            char name[64];
            id = (unsigned int)(camera.get(CV_CAP_PROP_GUID));
            sprintf(name, "%i", id);
            message.push_back(atom::StringValue::create(name));

            camera.release();

            index++;
        }
    }

    return message;
}
