#include "source_2d_image.h"

using namespace std;

string Source_2D_Image::mClassName = "Source_2D_Image";
string Source_2D_Image::mDocumentation = "N/A";

/*************/
Source_2D_Image::Source_2D_Image()
{
    make(0);
}

/*************/
Source_2D_Image::Source_2D_Image(string pParam)
{
    make(pParam);
}

/*************/
void Source_2D_Image::make(string pParam)
{
    mName = mClassName;
    mSubsourceNbr = pParam;
}

/*************/
Source_2D_Image::~Source_2D_Image()
{
    disconnect();
}

/*************/
bool Source_2D_Image::connect()
{
    if (mSubsourceNbr == "")
        return true;

    mWidth = 0;
    mHeight = 0;
    mFramerate = 0;
    mExposureTime = 0;

    return true;
}

/*************/
bool Source_2D_Image::disconnect()
{
    return true;
}

/*************/
bool Source_2D_Image::grabFrame()
{
    mUpdated = true;
    return true;
}

/*************/
cv::Mat Source_2D_Image::retrieveRawFrame()
{
    return mImage.clone();
}

/*************/
void Source_2D_Image::setParameter(atom::Message pParam)
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

        cv::Mat img = cv::imread(url);
        if (img.total() > 0, -1)
        {
            g_log(NULL, G_LOG_LEVEL_DEBUG, "%s - Successfully loaded image from path %s", mClassName.c_str(), url.c_str());
            mImage = img;
            mWidth = img.cols;
            mHeight = img.rows;
            mChannels = img.channels();
        }
        else
        {
            g_log(NULL, G_LOG_LEVEL_WARNING, "%s - Unable to load image from path %s", mClassName.c_str(), url.c_str());
        }
    }
    else
        setBaseParameter(pParam);
}

/*************/
atom::Message Source_2D_Image::getParameter(atom::Message pParam) const
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
atom::Message Source_2D_Image::getSubsources() const
{
    atom::Message message;
    return message;
}
