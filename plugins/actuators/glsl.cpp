#include "glsl.h"

#include <fstream>
#include <cerrno>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

char DEFAULT_VERTEX_SHADER[] =
    "#version 150 core\n"
    "\n"
    "in vec4 vVertex;\n"
    "in vec2 vTexCoord;\n"
    "\n"
    "uniform mat4 vViewProjectionMatrix;\n"
    "\n"
    "smooth out vec2 finalTexCoord;\n"
    "\n"
    "void main(void)\n"
    "{\n"
    "    gl_Position.xyz = (vViewProjectionMatrix*vVertex).xyz;\n"
    "    finalTexCoord = vTexCoord;\n"
    "}\n";

char DEFAULT_FRAGMENT_SHADER[] =
    "#version 150 core\n"
    "\n"
    "uniform sampler2D vTex0;\n"
    "\n"
    "in vec2 finalTexCoord;\n"
    "\n"
    "out vec4 fragColor;\n"
    "\n"
    "void main(void)\n"
    "{\n"
    "    fragColor = vec4(finalTexCoord.x, finalTexCoord.y, 0.0, 1.0);\n"
    "    fragColor *= texture(vTex0, finalTexCoord);\n"
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

    mIsGLVisible = GL_FALSE;
    mWindow = NULL;
    mOverrideSize = false;
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
atom::Message Actuator_GLSL::detect(vector<Capture_Ptr> pCaptures)
{
    vector<cv::Mat> captures = captureToMat(pCaptures);
    if (captures.size() < mSourceNbr)
        return mLastMessage;
    cv::Mat capture = captures[0];

    if (!mIsInitDone)
        return mLastMessage;

    glfwMakeContextCurrent(mWindow);
    uploadTextures(captures);

    int width, height;
    glfwGetFramebufferSize(mWindow, &width, &height);
    
    if (mIsGLVisible == GL_FALSE)
        glfwHideWindow(mWindow);

    if (!mOverrideSize && (width != capture.cols || height != capture.rows))
    {
        mGLSize = cv::Size(capture.cols, capture.rows);
        glfwSetWindowSize(mWindow, mGLSize.width, mGLSize.height);
        updateFBO();
    }
    else if (mOverrideSize && (width != mGLSize.width || height != mGLSize.height))
    {
        glfwShowWindow(mWindow);
        glfwSetWindowSize(mWindow, mGLSize.width, mGLSize.height);
        updateFBO();
    }

    glViewport(0, 0, mGLSize.width, mGLSize.height);
    mShader->setUniform("vDim", glm::ivec2(mGLSize.width, mGLSize.height));

    if (mShader->activate(this))
    {
        glm::mat4 viewProjectionMatrix = glm::ortho(-1.f, 1.f, -1.f, 1.f);
        mShader->setViewProjectionMatrix(viewProjectionMatrix);

        GLenum fboBuffers[] = {GL_COLOR_ATTACHMENT0};
        glBindFramebuffer(GL_FRAMEBUFFER, mFBO);
        glDrawBuffers(1, fboBuffers);
        glClear(GL_COLOR_BUFFER_BIT);
        glDrawArrays(GL_TRIANGLES, 0, 6);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, mFBOTexture);
        cv::Mat buffer = cv::Mat::zeros(mGLSize, CV_8UC3);
        glGetTexImage(GL_TEXTURE_2D, 0, GL_BGR, GL_UNSIGNED_BYTE, buffer.data);
        cv::flip(buffer, mOutputBuffer, 0);
        glBindTexture(GL_TEXTURE_2D, 0);

        if (mIsGLVisible)
        {
            GLenum renderBuffers[] = {GL_BACK};
            glDrawBuffers(1, renderBuffers);
            glClear(GL_COLOR_BUFFER_BIT);
            glDrawArrays(GL_TRIANGLES, 0, 6);
        }

        glfwSwapBuffers(mWindow);
    }
    else
    {
        glClear(GL_COLOR_BUFFER_BIT);
        glfwSwapBuffers(mWindow);
    }

    glfwMakeContextCurrent(NULL);

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
    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GL_FALSE);

    glfwWindowHint(GLFW_VISIBLE, mIsGLVisible);
    mGLSize = cv::Size(640, 480);
    mWindow = glfwCreateWindow(mGLSize.width, mGLSize.height, "Actuator_GLSL", NULL, NULL);
    if (!mWindow)
        g_log(NULL, G_LOG_LEVEL_ERROR, "%s - Unable to create a window", mClassName.c_str());

    glfwMakeContextCurrent(mWindow);
    glClearColor(0.f, 0.f, 0.1f, 1.f);

    initGeometry();
    initShader();
    initFBO();

    mIsInitDone = true;

    glfwMakeContextCurrent(NULL);
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
    mShader.reset(new Shader());
    mShader->setShader(string(DEFAULT_VERTEX_SHADER), "vertex");
    mShader->setShader(string(DEFAULT_FRAGMENT_SHADER), "fragment");
    mShader->activate(this);
}

/*************/
void Actuator_GLSL::initFBO()
{
    glGenFramebuffers(1, &mFBO);
    glBindFramebuffer(GL_FRAMEBUFFER, mFBO);

    glGenTextures(1, &mFBOTexture);
    glBindTexture(GL_TEXTURE_2D, mFBOTexture);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, mGLSize.width, mGLSize.height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);

    glBindTexture(GL_TEXTURE_2D, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, mFBOTexture, 0);

    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if (status != GL_FRAMEBUFFER_COMPLETE)
        g_log(NULL, G_LOG_LEVEL_WARNING, "%s - Error while initializing framebuffer object", mClassName.c_str());
    else
        g_log(NULL, G_LOG_LEVEL_WARNING, "%s - Framebuffer object successfully initialized", mClassName.c_str());

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

/*************/
void Actuator_GLSL::uploadTextures(vector<cv::Mat> pImg)
{
    if (pImg.size() != mTextures.size())
        mTextures.resize(pImg.size());

    for (uint i = 0; i < pImg.size(); ++i)
    {
        mTextures[i] = pImg[i];
        mShader->bindTexture(mTextures[i].getGLTex(), i, string("vTex") + to_string(i));
        mShader->setUniform(string("vTex") + to_string(i) + string("Size"), glm::ivec2(pImg[i].cols, pImg[i].rows));
    }
}

/*************/
void Actuator_GLSL::updateFBO()
{
    if (glIsTexture(mFBOTexture) == GL_FALSE)
        return;

    glBindTexture(GL_TEXTURE_2D, mFBOTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, mGLSize.width, mGLSize.height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    glBindTexture(GL_TEXTURE_2D, 0);
}

/*************/
string Actuator_GLSL::readFile(string pName)
{
    ifstream in(pName, ios::in | ios::binary);
    if (in)
    {
        string contents;
        in.seekg(0, ios::end);
        contents.resize(in.tellg());
        in.seekg(0, ios::beg);
        in.read(&contents[0], contents.size());
        in.close();
        return contents;
    }

    g_log(NULL, G_LOG_LEVEL_ERROR, "%s - File %s cannot be opened", mClassName.c_str(), pName.c_str());
}

/*************/
void Actuator_GLSL::setParameter(atom::Message pMessage)
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

    glfwMakeContextCurrent(mWindow);

    if (cmd == "vertexFile")
    {
        string name;
        if (readParam(pMessage, name))
        {
            string code = readFile(name);
            mShader->setShader(code, "vertex");
        }
    }
    else if (cmd == "geometryFile")
    {
        string name;
        if (readParam(pMessage, name))
        {
            string code = readFile(name);
            mShader->setShader(code, "geometry");
        }
    }
    else if (cmd == "fragmentFile")
    {
        string name;
        if (readParam(pMessage, name))
        {
            string code = readFile(name);
            mShader->setShader(code, "fragment");
        }
    }
    else if (cmd == "glSize")
    {
        float w, h;
        if (!readParam(pMessage, w, 1))
            return;
        if (!readParam(pMessage, h, 2))
            return;
        if (w > 0 && h > 0)
        {
            mOverrideSize = true;
            mGLSize = cv::Size(w, h);
        }
    }
    else if (cmd == "uniform")
    {
        string name;
        int card;
        if (!readParam(pMessage, name, 1))
            return;
        card = pMessage.size() - 2;

        vector<float> values;
        values.resize(card);
        for (int i = 0; i < card; ++i)
            if (!readParam(pMessage, values[i], i+2))
                return;

        if (card == 1)
            mShader->setUniform(name, values[0]);
        else if (card == 2)
            mShader->setUniform(name, glm::dvec2(values[0], values[1]));
        else if (card == 2)
            mShader->setUniform(name, glm::dvec3(values[0], values[1], values[2]));
        else if (card == 2)
            mShader->setUniform(name, glm::dvec4(values[0], values[1], values[2], values[3]));
    }
    else if (cmd == "visible")
    {
        float visible;
        if (readParam(pMessage, visible))
            mIsGLVisible = (visible > 0.f) ? GL_TRUE : GL_FALSE;
    }
    else
        setBaseParameter(pMessage);
        
    glfwMakeContextCurrent(NULL);
}

/*************/
// Texture
/*************/
Texture::Texture()
{
    glGenTextures(1, &mGLTex);
    mSize = cv::Size(0, 0);
    mType = -1;
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
    if (mSize != pImg.size() || mType != pImg.type() || !glIsTexture(mGLTex))
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

        if (pImg.type() == CV_8UC3)
        {
            g_log(NULL, G_LOG_LEVEL_DEBUG, "%s - Created a new texture of type GL_UNSIGNED_BYTE, format GL_BGR", "Texture");
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, buffer.cols, buffer.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, buffer.data);
        }
        else if (pImg.type() == CV_8UC1)
        {
            g_log(NULL, G_LOG_LEVEL_DEBUG, "%s - Created a new texture of type GL_UNSIGNED_BYTE, format GL_RED", "Texture");
            glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, buffer.cols, buffer.rows, 0, GL_RED, GL_UNSIGNED_BYTE, buffer.data);
        }
        else if (pImg.type() == CV_32FC3)
        {
            g_log(NULL, G_LOG_LEVEL_DEBUG, "%s - Created a new texture of type GL_FLOAT, format GL_BGR", "Texture");
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, buffer.cols, buffer.rows, 0, GL_BGR, GL_FLOAT, buffer.data);
        }
        else if (pImg.type() == CV_32FC1)
        {
            g_log(NULL, G_LOG_LEVEL_DEBUG, "%s - Created a new texture of type GL_FLOAT, format GL_RED", "Texture");
            glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, buffer.cols, buffer.rows, 0, GL_RED, GL_FLOAT, buffer.data);
        }
        else
        {
            g_log(NULL, G_LOG_LEVEL_ERROR, "%s - Unsupported texture format", "Texture");
            return *this;
        }

        glGenerateMipmap(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, 0);

        mSize = pImg.size();
        mType = pImg.type();
    }
    else
    {
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, mGLTex);

        if (mType == CV_8UC3)
            glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, buffer.cols, buffer.rows, GL_BGR, GL_UNSIGNED_BYTE, buffer.data);
        else if (mType == CV_8UC1)
            glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, buffer.cols, buffer.rows, GL_RED, GL_UNSIGNED_BYTE, buffer.data);
        else if (mType == CV_32FC3)
            glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, buffer.cols, buffer.rows, GL_BGR, GL_FLOAT, buffer.data);
        else if (mType = CV_32FC1)
            glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, buffer.cols, buffer.rows, GL_RED, GL_FLOAT, buffer.data);
        else
            return *this; // This should never happen due to the previous test of type

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

    if (glIsProgram(mProgram) == GL_TRUE && mIsLinked)
    {
        glUseProgram(mProgram);
        return true;
    }
    else if (glIsProgram(mProgram) == GL_TRUE)
        glDeleteProgram(mProgram);

    mProgram = glCreateProgram();
    if (glIsShader(mVertex))
    {
        glGetShaderiv(mVertex, GL_COMPILE_STATUS, &status);
        if (status == GL_TRUE)
        {
            glAttachShader(mProgram, mVertex);
            g_log(NULL, G_LOG_LEVEL_DEBUG, "%s - Vertex shader attached to the shader program", __FUNCTION__);
        }
    }
    if (glIsShader(mGeometry))
    {
        glGetShaderiv(mGeometry, GL_COMPILE_STATUS, &status);
        if (status == GL_TRUE)
        {
            glAttachShader(mProgram, mGeometry);
            g_log(NULL, G_LOG_LEVEL_DEBUG, "%s - Geometry shader attached to the shader program", __FUNCTION__);
        }
    }
    if (glIsShader(mFragment))
    {
        glGetShaderiv(mFragment, GL_COMPILE_STATUS, &status);
        if (status == GL_TRUE)
        {
            glAttachShader(mProgram, mFragment);
            g_log(NULL, G_LOG_LEVEL_DEBUG, "%s - Fragment shader attached to the shader program", __FUNCTION__);
        }
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
    }
    else
    {
        g_log(NULL, G_LOG_LEVEL_WARNING, "%s - Error while linking the shader program", __FUNCTION__);
        
        GLint length;
        glGetProgramiv(mProgram, GL_INFO_LOG_LENGTH, &length);
        char* log = (char*)malloc(length);
        glGetProgramInfoLog(mProgram, length, &length, log);
        g_log(NULL, G_LOG_LEVEL_WARNING, "%s - Error log: \n %s", __FUNCTION__, log);
        free(log);

        mIsLinked = false;
        return false;
    }

    mLocationMVP = glGetUniformLocation(mProgram, "vViewProjectionMatrix");

    return true;
}

/*************/
void Shader::bindTexture(GLuint pTexture, uint pTextureUnit, string pName)
{
    glActiveTexture(GL_TEXTURE0 + pTextureUnit);
    glBindTexture(GL_TEXTURE_2D, pTexture);
    GLint uniform = glGetUniformLocation(mProgram, pName.c_str());
    glUniform1i(uniform, pTextureUnit);
}

/*************/
void Shader::setViewProjectionMatrix(const glm::mat4& matrix)
{
    glUniformMatrix4fv(mLocationMVP, 1, GL_FALSE, glm::value_ptr(matrix));
}

/*************/
void Shader::setUniform(string name, int v)
{
    GLint location;
    location = glGetUniformLocation(mProgram, name.c_str());
    if (location == -1)
        return;
    glUniform1i(location, v);
}

/*************/
void Shader::setUniform(string name, glm::ivec2 v)
{
    GLint location;
    location = glGetUniformLocation(mProgram, name.c_str());
    if (location == -1)
        return;
    glUniform2i(location, v[0], v[1]);
}

/*************/
void Shader::setUniform(string name, glm::ivec3 v)
{
    GLint location;
    location = glGetUniformLocation(mProgram, name.c_str());
    if (location == -1)
        return;
    glUniform3i(location, v[0], v[1], v[2]);
}

/*************/
void Shader::setUniform(string name, glm::ivec4 v)
{
    GLint location;
    location = glGetUniformLocation(mProgram, name.c_str());
    if (location == -1)
        return;
    glUniform4i(location, v[0], v[1], v[2], v[3]);
}

/*************/
void Shader::setUniform(string name, float v)
{
    GLint location;
    location = glGetUniformLocation(mProgram, name.c_str());
    if (location == -1)
        return;
    glUniform1f(location, v);
}

/*************/
void Shader::setUniform(string name, glm::dvec2 v)
{
    GLint location;
    location = glGetUniformLocation(mProgram, name.c_str());
    if (location == -1)
        return;
    glUniform2f(location, v[0], v[1]);
}

/*************/
void Shader::setUniform(string name, glm::dvec3 v)
{
    GLint location;
    location = glGetUniformLocation(mProgram, name.c_str());
    if (location == -1)
        return;
    glUniform3f(location, v[0], v[1], v[2]);
}

/*************/
void Shader::setUniform(string name, glm::dvec4 v)
{
    GLint location;
    location = glGetUniformLocation(mProgram, name.c_str());
    if (location == -1)
        return;
    glUniform4f(location, v[0], v[1], v[2], v[3]);
}
