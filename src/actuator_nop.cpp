#include "actuator_nop.h"

using namespace std;

std::string Actuator_Nop::mClassName = "Actuator_Nop";
std::string Actuator_Nop::mDocumentation = "N/A";
unsigned int Actuator_Nop::mSourceNbr = 1;

/*************/
Actuator_Nop::Actuator_Nop()
{
    make();
}

/*************/
Actuator_Nop::Actuator_Nop(int pParam)
{
    make();
}

/*************/
void Actuator_Nop::make()
{
    mOutputBuffer = cv::Mat::zeros(480, 640, CV_8UC3);

    mName = mClassName;
    // OSC path for this actuator
    mOscPath = "/blobserver/nop";

    mFrameNumber = 0;
}

/*************/
atom::Message Actuator_Nop::detect(vector< Capture_Ptr > pCaptures)
{
    if (pCaptures.size() == 0)
        return mLastMessage;
    mCapture = pCaptures[0];

    mFrameNumber++;
    mLastMessage = atom::createMessage("iii", 1, 1, mFrameNumber);

    return mLastMessage;
}

/*************/
void Actuator_Nop::setParameter(atom::Message pMessage)
{
    setBaseParameter(pMessage);
}
