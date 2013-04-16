#include "detector.h"

using namespace std;

std::string Detector::mClassName = "Detector";
std::string Detector::mDocumentation = "N/A";
unsigned int Detector::mSourceNbr = 1;

/*************/
Detector::Detector()
{
    mName = mClassName;
    mSourceNbr = 1;

    mVerbose = true;

    mOutputBuffer = cv::Mat::zeros(0, 0, CV_8U);
    // By default, the mask is all white (all pixels are used)
    mSourceMask = cv::Mat::ones(1, 1, CV_8U);
}

/*****************/
Detector::Detector(int pParam)
{
    Detector();
}

/*****************/
void Detector::setMask(cv::Mat pMask)
{
    mSourceMask = pMask.clone();
    mMask = pMask.clone();
}

/*****************/
atom::Message Detector::getParameter(atom::Message pParam)
{
    atom::Message message;

    if (pParam.size() < 1)
        return message;
        
    std::string param;
    try
    {
        param = atom::toString(pParam[0]);
    }
    catch (...)
    {
        return message;
    }

    message.push_back(pParam[0]);
    if (param == "name")
        message.push_back(atom::StringValue::create(mClassName.c_str()));
    else if (param == "osc path")
        message.push_back(atom::StringValue::create(mOscPath.c_str()));

    return message;
}

/*****************/
void Detector::setBaseParameter(const atom::Message pParam)
{
    atom::Message message;

    if (pParam.size() < 2)
        return;
    
    std::string paramName;
    try
    {
        paramName = atom::toString(pParam[0]);
    }
    catch (atom::BadTypeTagError error)
    {
        return;
    }

    if (paramName == "verbose")
    {
        int value;
        try
        {
            value = atom::toInt(pParam[1]);
        }
        catch (atom::BadTypeTagError error)
        {
            return;
        }
        mVerbose = value;
    }
}

/*****************/
void Detector::addSource(shared_ptr<Source> source)
{
    mSources.push_back(source);
}

/*****************/
cv::Mat Detector::getMask(cv::Mat pCapture, int pInterpolation)
{
    if (pCapture.rows != mMask.rows || pCapture.cols != mMask.cols)
        cv::resize(mSourceMask, mMask, pCapture.size(), 0, 0, pInterpolation);

    return mMask;
}
