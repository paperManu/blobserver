#include "detector.h"

using namespace std;

std::string Detector::mClassName = "Detector";
std::string Detector::mDocumentation = "N/A";
unsigned int Detector::mSourceNbr = 1;

/**************/
Detector::Detector()
{
    mName = mClassName;
    mSourceNbr = 1;

    mVerbose = true;

    mOutputBuffer = cv::Mat::zeros(0, 0, CV_8U);
    // By default, the mask is all white (all pixels are used)
    mSourceMask = cv::Mat::ones(1, 1, CV_8U);
}

/**************/
Detector::Detector(int pParam)
{
    Detector();
}

/**************/
void Detector::setMask(cv::Mat pMask)
{
    mSourceMask = pMask.clone();
    mMask = pMask.clone();
}

/**************/
atom::Message Detector::getParameter(atom::Message pParam) const
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
void Detector::setBaseParameter(const atom::Message pMessage)
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
void Detector::addSource(shared_ptr<Source_2D> source)
{
    mSources.push_back(source);
}

/**************/
cv::Mat Detector::getMask(cv::Mat pCapture, int pInterpolation)
{
    if (pCapture.rows != mMask.rows || pCapture.cols != mMask.cols)
        cv::resize(mSourceMask, mMask, pCapture.size(), 0, 0, pInterpolation);

    return mMask;
}

/**************/
vector<cv::Mat> Detector::captureToMat(vector< shared_ptr<Capture> > pCaptures)
{
    vector<cv::Mat> images;
    for_each (pCaptures.begin(), pCaptures.end(), [&] (shared_ptr<Capture> capture)
    {
        shared_ptr<Capture_2D_Mat> capture2D = dynamic_pointer_cast<Capture_2D_Mat>(capture);
        if (capture2D.get() != NULL)
            images.push_back(capture2D->get());
    });

    return images;
}
