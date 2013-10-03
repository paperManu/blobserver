#include "glsl.h"

using namespace std;

std::string Actuator_GLSL::mClassName = "Actuator_GLSL";
std::string Actuator_GLSL::mDocumentation = "N/A";
unsigned int Actuator_GLSL::mSourceNbr = 1;

/*************/
Actuator_GLSL::Actuator_GLSL()
{
    make();
}

/*************/
Actuator_GLSL::Actuator_GLSL(int pParam)
{
    make();
}

/*************/
void Actuator_GLSL::make()
{
    mOutputBuffer = cv::Mat::zeros(480, 640, CV_8UC3);

    mName = mClassName;
    // OSC path for this actuator
    mOscPath = "glsl";

    mFrameNumber = 0;

    isGlfw = false;
    if (!glfwInit())
        g_log(NULL, G_LOG_LEVEL_ERROR, "%s - Unable to initialize GLFW", mClassName.c_str());
    else
        isGlfw = true;

    mWindow = glfwCreateWindow(640, 480, "Actuator_GLSL", NULL, NULL);
    if (!mWindow)
        g_log(NULL, G_LOG_LEVEL_ERROR, "%s - Unable to create a window", mClassName.c_str());
}

/*************/
Actuator_GLSL::~Actuator_GLSL()
{
    if (isGlfw)
        glfwTerminate();
}

/*************/
atom::Message Actuator_GLSL::detect(vector< Capture_Ptr > pCaptures)
{
    if (pCaptures.size() == 0)
        return mLastMessage;
    mCapture = pCaptures[0];

    glfwMakeContextCurrent(mWindow);
    glfwSwapBuffers(mWindow);

    mFrameNumber++;
    mLastMessage = atom::createMessage("iii", 1, 1, mFrameNumber);

    return mLastMessage;
}

/*************/
void Actuator_GLSL::setParameter(atom::Message pMessage)
{
    setBaseParameter(pMessage);
}
