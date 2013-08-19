#include "actuator.h"

using namespace std;

std::string Actuator::mClassName = "Actuator";
std::string Actuator::mDocumentation = "N/A";
unsigned int Actuator::mSourceNbr = 1;

/**************/
Actuator::Actuator()
{
    mName = mClassName;
    mSourceNbr = 1;

    mVerbose = true;

    mOutputBuffer = cv::Mat::zeros(480, 640, CV_8U);
    // By default, the mask is all white (all pixels are used)
    mSourceMask = cv::Mat::ones(1, 1, CV_8U);
}

/**************/
Actuator::Actuator(int pParam)
{
    Actuator();
}

/**************/
void Actuator::setMask(cv::Mat pMask)
{
    mSourceMask = pMask.clone();
    mMask = pMask.clone();
}

/**************/
atom::Message Actuator::getParameter(atom::Message pParam) const
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

/**************/
void Actuator::setBaseParameter(const atom::Message pMessage)
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

    if (cmd == "verbose")
    {
        int value;
        if (readParam(pMessage, value))
            mVerbose = value;
    }
}

/**************/
void Actuator::addSource(shared_ptr<Source> source)
{
    mSources.push_back(source);
}

/**************/
cv::Mat Actuator::getMask(cv::Mat pCapture, int pInterpolation)
{
    if (pCapture.rows != mMask.rows || pCapture.cols != mMask.cols)
        cv::resize(mSourceMask, mMask, pCapture.size(), 0, 0, pInterpolation);

    return mMask;
}

/**************/
vector<cv::Mat> Actuator::captureToMat(vector< Capture_Ptr > pCaptures)
{
    vector<cv::Mat> images;
    for_each (pCaptures.begin(), pCaptures.end(), [&] (Capture_Ptr capture)
    {
        Capture_2D_Mat_Ptr capture2D = dynamic_pointer_cast<Capture_2D_Mat>(capture);
        if (capture2D.get() != NULL)
            images.push_back(capture2D->get());
    });

    return images;
}
