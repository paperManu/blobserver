#include "source_2d_opencv.h"

using namespace std;

string Source_2D_OpenCV::mClassName = "Source_2D_OpenCV";
string Source_2D_OpenCV::mDocumentation = "N/A";

/*************/
Source_2D_OpenCV::Source_2D_OpenCV()
{
    make(0);
}

/*************/
Source_2D_OpenCV::Source_2D_OpenCV(string pParam)
{
    make(pParam);
}

/*************/
void Source_2D_OpenCV::make(string pParam)
{
    mName = mClassName;
    mSubsourceNbr = pParam;

    mVideoUrl = string("");
}

/*************/
Source_2D_OpenCV::~Source_2D_OpenCV()
{
    disconnect();
}

/*************/
bool Source_2D_OpenCV::connect()
{
    if (mSubsourceNbr == "")
        return true;

    mCamera.open(stoi(mSubsourceNbr));

    if (!mCamera.isOpened())
    {
        g_log(NULL, G_LOG_LEVEL_WARNING, "%s - Unable to open subsource %s", mClassName.c_str(), mSubsourceNbr.c_str());
        return false;
    }

    // Get current camera parameters
    mWidth = (unsigned int)(mCamera.get(CV_CAP_PROP_FRAME_WIDTH));
    mHeight = (unsigned int)(mCamera.get(CV_CAP_PROP_FRAME_HEIGHT));
    mFramerate = (unsigned int)(mCamera.get(CV_CAP_PROP_FPS));
    mExposureTime = (float)(mCamera.get(CV_CAP_PROP_EXPOSURE));
    
    int channels;
    channels = (int)(mCamera.get(CV_CAP_PROP_FORMAT));
    mChannels = (unsigned int)((channels >> 3) + 1); // See CV_MAKETYPE in types_c.h in OpenCV

    mId = to_string((int)(mCamera.get(CV_CAP_PROP_GUID)));

    return true;
}

/*************/
bool Source_2D_OpenCV::disconnect()
{
    mCamera.release();
    return true;
}

/*************/
bool Source_2D_OpenCV::grabFrame()
{
    if (!mCamera.isOpened())
        return false;

    bool result = mCamera.grab();
    mUpdated = result;
    return result;
}

/*************/
cv::Mat Source_2D_OpenCV::retrieveRawFrame()
{
    cv::Mat buffer;
    mCamera.retrieve(buffer);
    mBuffer = buffer;

    // If in-camera autoexposure is on, this needs to be done at each frame
    mExposureTime = (float)(mCamera.get(CV_CAP_PROP_EXPOSURE));

    return mBuffer.get().clone();
}

/*************/
void Source_2D_OpenCV::setParameter(atom::Message pParam)
{
    string paramName;
    float paramValue;

    try
    {
        paramName = atom::toString(pParam[0]);
    }
    catch (atom::BadTypeTagError exception)
    {
        return;
    }

    if (paramName == "url")
    {
        string url;
        if (!readParam(pParam, url))
            return;
        mVideoUrl = url;

        if (mCamera.isOpened())
            return;
        mCamera.open(url);
    }
    // Next parameters are all numbers
    else if (!readParam(pParam, paramValue))
    {
        setBaseParameter(pParam);
        return;
    }
    else if (paramName == "width")
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
    else if (paramName == "iso")
    {
        mCamera.set(CV_CAP_PROP_ISO_SPEED, paramValue);
    }
    else if (paramName == "exposureTime")
    {
        // If a LUT is set, exposureTime is set in ms, otherwise it is a
        // value with no specific scale (camera dependant)
        if (mExposureLUT.isSet())
        {
            float param = mExposureLUT[paramValue];
            if (mExposureLUT.isOutOfRange())
            {
                g_log(NULL, G_LOG_LEVEL_WARNING, "%s - Exposure value out of bounds", mClassName.c_str());
                return;
            }
            mCamera.set(CV_CAP_PROP_AUTO_EXPOSURE, 0);
            mCamera.set(CV_CAP_PROP_EXPOSURE, param);
            float value = (float)(mCamera.get(CV_CAP_PROP_EXPOSURE));
            mExposureTime = mExposureLUT.inverse(value);
            mExposureParam = paramValue;
        }
        else
        {
            mCamera.set(CV_CAP_PROP_AUTO_EXPOSURE, 0);
            mCamera.set(CV_CAP_PROP_EXPOSURE, paramValue);
            mExposureTime = (float)(mCamera.get(CV_CAP_PROP_EXPOSURE));
            mExposureParam = paramValue;
        }
    }
    else if (paramName == "gain")
    {
        // If a LUT is set, the gain is set in dB, otherwise it is
        // a value with no specific scale (camera dependant)
        if (mGainLUT.isSet())
        {
            float param = mGainLUT[paramValue];
            if (mGainLUT.isOutOfRange())
            {
                g_log(NULL, G_LOG_LEVEL_WARNING, "%s - Gain value out of bounds", mClassName.c_str());
                return;
            }
            mCamera.set(CV_CAP_PROP_GAIN, param);
            float value = (float)(mCamera.get(CV_CAP_PROP_GAIN));
            mGain = mGainLUT.inverse(value);
        }
        else
        {
            mCamera.set(CV_CAP_PROP_GAIN, paramValue);
            mGain = (float)(mCamera.get(CV_CAP_PROP_GAIN));
        }
    }
    else if (paramName == "gamma")
    {
        mCamera.set(CV_CAP_PROP_GAMMA, paramValue*1024.f);
        mGamma = (float)(mCamera.get(CV_CAP_PROP_GAMMA));
    }
    else if (paramName == "whiteBalanceRed")
    {
        mCamera.set(CV_CAP_PROP_WHITE_BALANCE_BLUE_U, paramValue);
    }
    else if (paramName == "whiteBalanceBlue")
    {
        mCamera.set(CV_CAP_PROP_WHITE_BALANCE_RED_V, paramValue);
    }
    else
    {
        setBaseParameter(pParam);
    }
}

/*************/
atom::Message Source_2D_OpenCV::getParameter(atom::Message pParam) const
{
    atom::Message msg;

    if (pParam.size() < 1)
        return msg;

    string paramName;
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
        msg.push_back(atom::FloatValue::create(mExposureTime));
    else if (paramName == "subsourcenbr")
        msg.push_back(atom::StringValue::create(mSubsourceNbr.c_str()));
    else
        msg = getBaseParameter(pParam);

    return msg;
}

/*************/
atom::Message Source_2D_OpenCV::getSubsources() const
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
