#include "python.h"


using namespace std;

std::string Actuator_Python::mClassName = "Actuator_Python";
std::string Actuator_Python::mDocumentation = "N/A";
unsigned int Actuator_Python::mSourceNbr = 1;

/*************/
Actuator_Python::Actuator_Python()
{
    make();
}

/*************/
Actuator_Python::Actuator_Python(int pParam)
{
    make();
}

/*************/
void Actuator_Python::make()
{
    mOutputBuffer = cv::Mat::zeros(480, 640, CV_8UC3);

    mName = mClassName;
    // OSC path for this actuator
    mOscPath = "nop";

    mFrameNumber = 0;
}

/*************/
atom::Message Actuator_Python::detect(vector< Capture_Ptr > pCaptures)
{
    if (pCaptures.size() == 0)
        return mLastMessage;
    mCapture = pCaptures[0];

    mFrameNumber++;
    mLastMessage = atom::createMessage("iii", 1, 1, mFrameNumber);

    return mLastMessage;
}

/*************/
void Actuator_Python::setParameter(atom::Message pMessage)
{
    setBaseParameter(pMessage);
}
