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

    glfwWindowHint(GLFW_VISIBLE, GL_FALSE);
    mGLSize = cv::Size(640, 480);
    mWindow = glfwCreateWindow(mGLSize.width, mGLSize.height, "Actuator_GLSL", NULL, NULL);
    if (!mWindow)
        g_log(NULL, G_LOG_LEVEL_ERROR, "%s - Unable to create a window", mClassName.c_str());

    initGL();
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
    vector<cv::Mat> captures = captureToMat(pCaptures);
    if (captures.size() < mSourceNbr)
        return mLastMessage;
    cv::Mat capture = captures[0];
    
    if (mGLSize.width != capture.cols || mGLSize.height != capture.rows)
    {
        mGLSize = cv::Size(capture.cols, capture.rows);
        glfwSetWindowSize(mWindow, mGLSize.width, mGLSize.height);
    }

    glfwMakeContextCurrent(mWindow);
    glfwSwapBuffers(mWindow);

    mFrameNumber++;
    mLastMessage = atom::createMessage("iii", 1, 1, mFrameNumber);

    return mLastMessage;
}

/*************/
void Actuator_GLSL::initGL()
{
    GLfloat points[] = {-1.f, -1.f, 0.f, 1.f,
                        -1.f, 1.f, 0.f, 1.f,
                        1.f, 1.f, 0.f, 1.f,
                        1.f, 1.f, 0.f, 1.f,
                        1.f, -1.f, 0.f, 1.f,
                        -1.f, -1.f, 0.f, 1.f};

    GLfloat tex[] = {0.f, 0.f,
                     0.f, 1.f,
                     1.f, 1.f,
                     1.f, 1.f,
                     1.f, 0.f,
                     0.f, 0.f};

    glGenVertexArrays(1, &mVertexArray);
    glBindVertexArray(mVertexArray);
    glGenBuffers(2, mVertexBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, mVertexBuffer[0]);
    glBufferData(GL_ARRAY_BUFFER, 6*4*sizeof(float), points, GL_STATIC_DRAW);
    glVertexAttribPointer(mVertexBuffer[0], 4, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(mVertexBuffer[0]);

    glBindBuffer(GL_ARRAY_BUFFER, mVertexBuffer[1]);
    glBufferData(GL_ARRAY_BUFFER, 6*2*sizeof(float), tex, GL_STATIC_DRAW);
    glVertexAttribPointer(mVertexBuffer[1], 2, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(mVertexBuffer[1]);
}

/*************/
void Actuator_GLSL::setParameter(atom::Message pMessage)
{
    setBaseParameter(pMessage);
}
