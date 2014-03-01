#include "face.h"

#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>

using namespace std;

/*************/
// Definition of class Actuator_Face
/*************/
std::string Actuator_Face::mClassName = "Actuator_Face";
std::string Actuator_Face::mDocumentation = "N/A";
unsigned int Actuator_Face::mSourceNbr = 1;

/*************/
Actuator_Face::Actuator_Face()
{
    make();
}

/*************/
Actuator_Face::Actuator_Face(int pParam)
{
    make();
}

/*************/
void Actuator_Face::make()
{
    mOutputBuffer = cv::Mat::zeros(480, 640, CV_8UC3);

    mName = mClassName;
    mOscPath = "face";

    mFilterSize = 3;
    mFilterDilateCoeff = 2;

    mBlobLifetime = 30;
    mKeepOldBlobs = 0;
    mKeepMaxTime = 0;
    mProcessNoiseCov = 1e-6;
    mMeasurementNoiseCov = 1e-4;
    mMaxDistanceForColorDiff = 16;

    mFaceCascade.load((string(DATADIR) + "haarcascade_frontalface_alt2.xml").c_str());
    mEyeCascade.load((string(DATADIR) + "haarcascade_eye.xml").c_str());
    mMouthCascade.load((string(DATADIR) + "haarcascade_mcs_mouth.xml").c_str());
}

/*************/
atom::Message Actuator_Face::detect(const vector< Capture_Ptr > pCaptures)
{
    vector<cv::Mat> captures = captureToMat(pCaptures);
    if (captures.size() < mSourceNbr)
        return mLastMessage;

    if (mFaceCascade.empty() || mEyeCascade.empty() || mMouthCascade.empty())
        return mLastMessage;

    // For simplicity...
    cv::Mat input = captures[0];
    cv::Mat grayImg;
    cv::cvtColor(input, grayImg, CV_BGR2GRAY);

    vector<cv::Rect> faces;
    mFaceCascade.detectMultiScale(grayImg, faces, 1.1, 2, CV_HAAR_SCALE_IMAGE, cv::Size(32, 32));

    mPersons.clear();
    for (auto& face : faces)
    {
        face.y -= face.height / 2;
        face.height *= 2;

        Person person;
        person.face = face;

        vector<cv::Rect> eyes;
        mEyeCascade.detectMultiScale(grayImg(face), eyes, 1.1, 2, CV_HAAR_SCALE_IMAGE, cv::Size(16, 16));
        for (auto& eye : eyes)
            person.eyes.push_back(eye + cv::Point(face.x, face.y));

        vector<cv::Rect> mouth;
        mMouthCascade.detectMultiScale(grayImg(face), mouth, 1.1, 2, CV_HAAR_SCALE_IMAGE, cv::Size(16, 16));
        if (mouth.size() > 0)
            person.mouth = mouth[0];

        mPersons.push_back(person);
    }

    // Draw faces and eyes to the output
    cv::Mat resultMat = input.clone();
    for (auto& person : mPersons)
    {
        cv::rectangle(resultMat, person.face, cv::Scalar(1.0), 3);
        for (auto& eye : person.eyes)
            cv::rectangle(resultMat, eye, cv::Scalar(1.0), 1);
        cv::rectangle(resultMat, person.mouth, cv::Scalar(1.0), 1);
    }

    mOutputBuffer = resultMat.clone();

    return mLastMessage;
}

/*************/
void Actuator_Face::setParameter(atom::Message pMessage)
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

    if (cmd == "filterSize")
    {
        float filterSize;
        if (readParam(pMessage, filterSize))
            mFilterSize = max(1, (int)filterSize);
    }
    else if (cmd == "filterDilateCoeff")
    {
        float filterSize;
        if (readParam(pMessage, filterSize))
            mFilterDilateCoeff = max(0, (int)filterSize);
    }
    else if (cmd == "lifetime")
    {
        float lifetime;
        if (readParam(pMessage, lifetime))
            mBlobLifetime = lifetime;
    }
    else if (cmd == "keepOldBlobs")
    {
        float keep;
        if (readParam(pMessage, keep, 1))
            mKeepOldBlobs = (int)keep;
        if (readParam(pMessage, keep, 2))
            mKeepMaxTime = (int)keep;
    }
    else if (cmd == "processNoiseCov")
    {
        float cov;
        if (readParam(pMessage, cov))
            mProcessNoiseCov = abs(cov);
    }
    else if (cmd == "measurementNoiseCov")
    {
        float cov;
        if (readParam(pMessage, cov))
            mMeasurementNoiseCov = abs(cov);
    }
    else if (cmd == "maxDistanceForColorDiff")
    {
        float dist;
        if (readParam(pMessage, dist))
            mMaxDistanceForColorDiff = abs(dist);
    }
    else
        setBaseParameter(pMessage);
}
