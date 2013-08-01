#include "detector_nop.h"

using namespace std;

std::string Detector_Nop::mClassName = "Detector_Nop";
std::string Detector_Nop::mDocumentation = "N/A";
unsigned int Detector_Nop::mSourceNbr = 1;

/*************/
Detector_Nop::Detector_Nop()
{
    make();
}

/*************/
Detector_Nop::Detector_Nop(int pParam)
{
    make();
}

/*************/
void Detector_Nop::make()
{
    mOutputBuffer = cv::Mat::zeros(480, 640, CV_8UC3);

    mName = mClassName;
    // OSC path for this detector
    mOscPath = "/blobserver/nop";

    mFrameNumber = 0;
}

/*************/
atom::Message Detector_Nop::detect(vector< Capture_Ptr > pCaptures)
{
    if (pCaptures.size() == 0)
        return mLastMessage;
    mCapture = pCaptures[0];

    mFrameNumber++;
    mLastMessage = atom::createMessage("iii", 1, 1, mFrameNumber);

    return mLastMessage;
}

/*************/
void Detector_Nop::setParameter(atom::Message pMessage)
{
    setBaseParameter(pMessage);
}
