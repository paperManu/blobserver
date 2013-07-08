#include "detector_stitch.h"

using namespace std;

std::string Detector_Nop::mClassName = "Detector_Stitch";
std::string Detector_Nop::mDocumentation = "N/A";
unsigned int Detector_Nop::mSourceNbr = 1;

/*************/
Detector_Stitch::Detector_Stitch()
{
    make();
}

/*************/
Detector_Stitch::Detector_Stitch(int pParam)
{
    make();
}

/*************/
void Detector_Stitch::make()
{
    mOutputBuffer = cv::Mat::zeros(480, 640, CV_8UC3);

    mName = mClassName;
    // OSC path for this detector
    mOscPath = "/blobserver/stitch";

    mFrameNumber = 0;
}

/*************/
atom::Message Detector_Stitch::detect(vector<cv::Mat> pCaptures)
{
    if (pCaptures.size() == 0)
        return mLastMessage;

    mOutputBuffer = pCaptures[0].clone();

    mFrameNumber++;
    mLastMessage = atom::createMessage("iii", 1, 1, mFrameNumber);

    return mLastMessage;
}

/*************/
void Detector_Stitch::setParameter(atom::Message pMessage)
{
    setBaseParameter(pMessage);
}
