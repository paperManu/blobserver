#include "glsl.h"

char DEFAULT_VERTEX_SHADER[] =
    "#version 150 core\n"
    "\n"
    "in vec4 vVertex;\n"
    "in vec2 vTexCoord;\n"
    "\n"
    "uniform mat4 vMVP;\n"
    "uniform vec2 vMouse;\n"
    "\n"
    "smooth out vec2 finalTexCoord;\n"
    "\n"
    "void main(void)\n"
    "{\n"
    "    gl_Position.xyz = (vMVP*vVertex).xyz;\n"
    "\n"
    "    finalTexCoord = vTexCoord;\n"
    "}\n";

char DEFAULT_FRAGMENT_SHADER[] =
    "#version 150 core\n"
    "\n"
    "uniform sampler2D vTexMap;\n"
    "uniform sampler2D vHUDMap;\n"
    "uniform sampler2D vFBOMap;\n"
    "\n"
    "uniform vec2 vMouse;\n"
    "uniform float vTimer;\n"
    "uniform vec2 vResolution;\n"
    "uniform int vPass;\n"
    "\n"
    "in vec2 finalTexCoord;\n"
    "\n"
    "out vec4 fragColor;\n"
    "\n"
    "void main(void)\n"
    "{\n"
    "    float lHUDScale = vResolution.y/32.f;\n"
    "    fragColor = vec4(1.0, 0.0, 0.0, 1.0);\n"
    "    fragColor += texture(vHUDMap, vec2(finalTexCoord.s, finalTexCoord.t*lHUDScale));\n"
    "}\n";

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

    mIsInitDone = false;
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

    if (!mIsInitDone)
        return mLastMessage;

    glfwMakeContextCurrent(mWindow);
    uploadTextures(captures);
    
    if (mGLSize.width != capture.cols || mGLSize.height != capture.rows)
    {
        mGLSize = cv::Size(capture.cols, capture.rows);
        glfwSetWindowSize(mWindow, mGLSize.width, mGLSize.height);
    }

    if (mShader->activate(this))
    {
        glClear(GL_COLOR_BUFFER_BIT);
        glClearColor(1.f, 1.f, 1.f, 1.f);
        glfwSwapBuffers(mWindow);
    }
    else
    {
        glClear(GL_COLOR_BUFFER_BIT);
        glfwSwapBuffers(mWindow);
    }

    mFrameNumber++;
    mLastMessage = atom::createMessage("iii", 1, 1, mFrameNumber);

    return mLastMessage;
}

/*************/
void Actuator_GLSL::initGL()
{
    isGlfw = false;
    if (!glfwInit())
        g_log(NULL, G_LOG_LEVEL_ERROR, "%s - Unable to initialize GLFW", mClassName.c_str());
    else
        isGlfw = true;

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GL_TRUE);

    glfwWindowHint(GLFW_VISIBLE, GL_TRUE);
    mGLSize = cv::Size(640, 480);
    mWindow = glfwCreateWindow(mGLSize.width, mGLSize.height, "Actuator_GLSL", NULL, NULL);
    if (!mWindow)
        g_log(NULL, G_LOG_LEVEL_ERROR, "%s - Unable to create a window", mClassName.c_str());

    glfwSetWindowTitle(mWindow, mClassName.c_str());
    glfwMakeContextCurrent(mWindow);

    glClearColor(0.f, 1.f, 0.f, 1.f);
    initGeometry();

    mShader.reset(new Shader());
    mShader->setShader(string(DEFAULT_VERTEX_SHADER), "vertex");
    mShader->setShader(string(DEFAULT_FRAGMENT_SHADER), "fragment");
    mShader->activate(this);

    mIsInitDone = true;
}

/*************/
void Actuator_GLSL::initGeometry()
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
void Actuator_GLSL::initShader()
{
}

/*************/
void Actuator_GLSL::uploadTextures(vector<cv::Mat> pImg)
{
    if (pImg.size() != mTextures.size())
        mTextures.resize(pImg.size());

    for (uint i = 0; i < pImg.size(); ++i)
        mTextures[i] = pImg[i];
}

/*************/
void Actuator_GLSL::setParameter(atom::Message pMessage)
{
    setBaseParameter(pMessage);
}

/*************/
// Texture
/*************/
Texture::Texture()
{
    glGenTextures(1, &mGLTex);
    mSize = cv::Size(0, 0);
}

/*************/
Texture::~Texture()
{
    if (glIsTexture(mGLTex))
        glDeleteTextures(1, &mGLTex);
}

/*************/
Texture& Texture::operator=(const cv::Mat& pImg)
{
    if (pImg.size() == cv::Size(0, 0))
        return *this;

    cv::Mat buffer;
    cv::flip(pImg, buffer, 0);

    glGetError();
    if (mSize != pImg.size() || !glIsTexture(mGLTex))
    {
        glDeleteTextures(1, &mGLTex);
        glGenTextures(1, &mGLTex);

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, mGLTex);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, buffer.cols, buffer.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, buffer.data);
        glGenerateMipmap(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, 0);

        mSize = pImg.size();
    }
    else
    {
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, mGLTex);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, buffer.cols, buffer.rows, GL_BGR, GL_UNSIGNED_BYTE, buffer.data);
        glGenerateMipmap(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, 0);
    }

    GLenum error = glGetError();
    if (error)
        g_log(NULL, G_LOG_LEVEL_WARNING, "%s - Error %i while setting GL texture", __FUNCTION__, error); 

    return *this;
}

/*************/
// Shader
/*************/
Shader::Shader()
{
    mVertex = glCreateShader(GL_VERTEX_SHADER);
    mGeometry = glCreateShader(GL_GEOMETRY_SHADER);
    mFragment = glCreateShader(GL_FRAGMENT_SHADER);
    mProgram = glCreateProgram();

    mIsLinked = false;
}

/*************/
Shader::~Shader()
{
    if (glIsProgram(mProgram))
        glDeleteProgram(mProgram);

    if (glIsShader(mVertex))
        glDeleteShader(mVertex);
    if (glIsShader(mGeometry))
        glDeleteShader(mGeometry);
    if (glIsShader(mFragment))
        glDeleteShader(mFragment);
}

/*************/
void Shader::setShader(const string src, const string type)
{
    GLuint shader;
    if (type == string("vertex"))
        shader = mVertex;
    else if (type == string("geometry"))
        shader = mGeometry;
    else if (type == string("fragment"))
        shader = mFragment;
    else
        return;

    const char* shaderSrc = src.c_str();
    glShaderSource(shader, 1, (const GLchar**)&shaderSrc, 0);
    glCompileShader(shader);

    GLint status;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &status);
    if (status)
        g_log(NULL, G_LOG_LEVEL_DEBUG, "%s - Shader of type %s compiled successfully", __FUNCTION__, type.c_str());
    else
    {
        g_log(NULL, G_LOG_LEVEL_WARNING, "%s - Error while compiling shader of type %s", __FUNCTION__, type.c_str());

        GLint length;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &length);
        char* log = (char*)malloc(length);
        glGetShaderInfoLog(shader, length, &length, log);
        g_log(NULL, G_LOG_LEVEL_WARNING, "%s - Error log: \n %s", __FUNCTION__, log);
        free(log);
    }

    mIsLinked = false;
}

/*************/
bool Shader::activate(void* pContext)
{
    GLint status;
    Actuator_GLSL* context = (Actuator_GLSL*)pContext;

    if (mIsLinked)
    {
        glUseProgram(mProgram);
        return true;
    }
    else if (glIsProgram(mProgram))
        glDeleteProgram(mProgram);

    mProgram = glCreateProgram();
    if (glIsShader(mVertex))
    {
        glGetShaderiv(mVertex, GL_COMPILE_STATUS, &status);
        if (status == GL_TRUE)
            glAttachShader(mProgram, mVertex);
    }
    if (glIsShader(mGeometry))
    {
        glGetShaderiv(mGeometry, GL_COMPILE_STATUS, &status);
        if (status == GL_TRUE)
            glAttachShader(mProgram, mGeometry);
    }
    if (glIsShader(mFragment))
    {
        glGetShaderiv(mGeometry, GL_COMPILE_STATUS, &status);
        if (status == GL_TRUE)
            glAttachShader(mProgram, mGeometry);
    }
    
    glBindAttribLocation(mProgram, context->mVertexBuffer[0], "vVertex");
    glBindAttribLocation(mProgram, context->mVertexBuffer[1], "vTexCoord");
    glLinkProgram(mProgram);

    glGetProgramiv(mProgram, GL_LINK_STATUS, &status);
    if (status == GL_TRUE)
    {
        g_log(NULL, G_LOG_LEVEL_DEBUG, "%s - Shader program linked successfully", __FUNCTION__);
        mIsLinked = true;
        glUseProgram(mProgram);
        return true;
    }
    else
    {
        g_log(NULL, G_LOG_LEVEL_WARNING, "%s - Error while linking shader program", __FUNCTION__);
        
        GLint length;
        glGetProgramiv(mProgram, GL_INFO_LOG_LENGTH, &length);
        char* log = (char*)malloc(length);
        glGetProgramInfoLog(mProgram, length, &length, log);
        g_log(NULL, G_LOG_LEVEL_WARNING, "%s - Error log: \n %s", __FUNCTION__, log);
        free(log);

        mIsLinked = false;
        return false;
    }
}

/*************/
void Shader::bindTexture(GLuint pTexture, uint pTextureUnit, string pName)
{
    glActiveTexture(GL_TEXTURE0 + pTextureUnit);
    glBindTexture(GL_TEXTURE_2D, pTexture);
    GLint uniform = glGetUniformLocation(mProgram, pName.c_str());
    glUniform1i(uniform, pTextureUnit);
}
