/*
 * Copyright (C) 2012 Emmanuel Durand
 *
 * This file is part of blobserver.
 *
 * This program is free software: you can redistribute it and/or modify
 *
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * blobserver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with blobserver.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * @glsl.h
 * The Actuator_GLSL class.
 */

#ifndef GLSL_H
#define GLSL_H

#define GLFW_NO_GLU
#define GL_GLEXT_PROTOTYPES

#include "actuator.h"

#include <memory>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

/*************/
// Class Texture
class Texture
{
    public:
        Texture();
        ~Texture();
        Texture& operator=(const cv::Mat& pImg);
        GLuint getGLTex() {return mGLTex;}

    private:
        GLuint mGLTex;
        cv::Size mSize;
};

/*************/
// Class Shader
class Shader
{
    public:
        Shader();
        ~Shader();
        void setShader(const std::string src, const std::string type);
        bool activate(void* pContext);
        void bindTexture(GLuint pTexture, uint pTextureUnit, std::string pName);

        void setViewProjectionMatrix(const glm::mat4& matrix);
        void setUniform(std::string name, float value);
        void setUniform(std::string name, glm::dvec2 v);
        void setUniform(std::string name, glm::dvec3 v); 
        void setUniform(std::string name, glm::dvec4 v);

    private:
        GLuint mVertex;
        GLuint mGeometry;
        GLuint mFragment;
        GLuint mProgram;
        bool mIsLinked;

        GLint mLocationMVP;
};

/*************/
// Class Actuator_GLSL
class Actuator_GLSL : public Actuator
{
    public:
        friend Shader;

        Actuator_GLSL();
        Actuator_GLSL(int pParam);
        ~Actuator_GLSL();

        static std::string getClassName() {return mClassName;}
        static std::string getDocumentation() {return mDocumentation;}

        atom::Message detect(std::vector< Capture_Ptr > pCaptures);
        void setParameter(atom::Message pMessage);

    private:
        // Attributes
        static std::string mClassName;
        static std::string mDocumentation;

        static unsigned int mSourceNbr;
        unsigned int mFrameNumber;

        Capture_Ptr mCapture;

        bool isGlfw, mIsInitDone;
        GLFWwindow* mWindow;
        cv::Size mGLSize;
        bool mOverrideSize;

        GLuint mVertexArray;
        GLuint mVertexBuffer[2];
        std::vector<Texture> mTextures;

        std::shared_ptr<Shader> mShader;

        GLuint mFBO;
        GLuint mFBOTexture;

        // Methods
        void make();
        void initGL();
        void initGeometry();
        void initShader();
        void initFBO();

        void uploadTextures(std::vector<cv::Mat> pImg);
        void updateFBO();

        std::string readFile(std::string pName);
};

REGISTER_ACTUATOR(Actuator_GLSL)

#endif // GLSL_H
