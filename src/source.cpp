#include "source.h"

std::string Source::mClassName = "Source";
std::string Source::mDocumentation = "N/A";

/*************/
Source::Source()
{
    mName = mClassName;
    mDocumentation = "N/A";

    mBuffer = cv::Mat::zeros(1, 1, CV_8U);

    mWidth = 0;
    mHeight = 0;
    mChannels = 0;
    mFramerate = 0;
    mSubsourceNbr = 0;
    mId = 0;

    mCorrectDistortion = false;
    mCorrectVignetting = false;

    mOpticalDesc.distortion = 0.0;
    mOpticalDesc.vignetting = 0.0;

    mRecomputeVignettingMat = false;
    mRecomputeDistortionMat = false;

    mICCTransform = NULL;
}

/************/
Source::Source(int pParam)
{
    Source();
}

/************/
Source::~Source()
{
    if (mICCTransform != NULL)
        cmsDeleteTransform(mICCTransform);
}

/************/
cv::Mat Source::retrieveCorrectedFrame()
{
    if (mUpdated)
    {
        mBuffer = retrieveFrame();

        if (mICCTransform != NULL)
            cmsDoTransform(mICCTransform, mBuffer.data, mBuffer.data, mBuffer.total());
        if (mCorrectVignetting)
            mBuffer = correctVignetting(mBuffer);
        if (mCorrectDistortion)
            mBuffer = correctDistortion(mBuffer);

        mUpdated = false;
    }

    return mBuffer.clone();
}

/************/
void Source::setBaseParameter(atom::Message pParam)
{
    if (pParam.size() < 2)
        return;
    
    std::string paramName;
    try
    {
        paramName = atom::toString(pParam[0]);
    }
    catch (atom::BadTypeTagError error)
    {
        return;
    }

    if (paramName == "vignetting")
    {
        if (pParam.size() == 4)
        {
            try
            {
                mOpticalDesc.vignetting[0] = atom::toFloat(pParam[1]);
                mOpticalDesc.vignetting[1] = atom::toFloat(pParam[2]);
                mOpticalDesc.vignetting[2] = atom::toFloat(pParam[3]);
            }
            catch (atom::BadTypeTagError error)
            {
                return;
            }

            mCorrectVignetting = true;
            mRecomputeVignettingMat = true;
        }
        else
            return;
    }
    else if (paramName == "distortion")
    {
        // Only one param, we correct only the 4th order
        // coefficient
        if (pParam.size() == 2)
        {
            float value;
            try
            {
                value = atom::toFloat(pParam[1]);
            }
            catch (atom::BadTypeTagError error)
            {
                return;
            }

            mOpticalDesc.distortion[0] = 0.0;
            mOpticalDesc.distortion[1] = value;
            mOpticalDesc.distortion[2] = 0.0;
        }
        // Three params, we correct all params
        else if (pParam.size() == 4)
        {
            try
            {
                mOpticalDesc.distortion[0] = atom::toFloat(pParam[1]);
                mOpticalDesc.distortion[1] = atom::toFloat(pParam[2]);
                mOpticalDesc.distortion[2] = atom::toFloat(pParam[3]);
            }
            catch (atom::BadTypeTagError error)
            {
                return;
            }
        }
        else
            return;

        mCorrectDistortion = true;
        mRecomputeDistortionMat = true;
    }
    else if (paramName == "iccInputProfile")
    {
        std::string filename;
        try
        {
            filename = atom::toString(pParam[1]);
        }
        catch (atom::BadTypeTagError error)
        {
            return;
        }

        if (mICCTransform != NULL)
            cmsDeleteTransform(mICCTransform);
        mICCTransform = loadICCTransform(filename);
    }
}

/************/
atom::Message Source::getBaseParameter(atom::Message pParam)
{
    atom::Message msg;

    if (pParam.size() < 1)
        return msg;

    std::string paramName;
    try
    {
        paramName = atom::toString(pParam[0]);
    }
    catch (atom::BadTypeTagError exception)
    {
        return msg;
    }

    msg.push_back(pParam[0]);
    if (paramName == "id")
        msg.push_back(atom::FloatValue::create(mId));

    return msg;
}

/************/
cv::Mat Source::correctVignetting(cv::Mat pImg)
{
    if (mRecomputeVignettingMat == true || mVignettingMat.size() != mBuffer.size())
    {
        // Vignetting description has changed, or grabbed image has not the same resolution
        mVignettingMat = cv::Mat::zeros(mHeight, mWidth, CV_32FC3);

        cv::Point2f center;
        center.x = (float)mWidth / 2.f;
        center.y = (float)mHeight / 2.f;
        
        float sqfactor = pow(center.y, 2.f) + pow(center.x, 2.f);

        for (int x = 0; x < (int)mWidth; ++x)
        {
            for (int y = 0; y < (int)mHeight; ++y)
            {
                float sqradius = (pow((float)x-center.x, 2.f) + pow((float)y-center.y, 2.f))/sqfactor;
                float correction = 1.f + mOpticalDesc.vignetting[0] * sqradius
                    + mOpticalDesc.vignetting[1] * pow(sqradius, 2.f)
                    + mOpticalDesc.vignetting[2] * pow(sqradius, 4.f);

                mVignettingMat.at<cv::Vec3f>(y, x)[0] = 1.f / correction;
                mVignettingMat.at<cv::Vec3f>(y, x)[1] = 1.f / correction;
                mVignettingMat.at<cv::Vec3f>(y, x)[2] = 1.f / correction;
            }
        }

        mRecomputeVignettingMat = false;
    }

    cv::Mat resultMat;
    cv::multiply(pImg, mVignettingMat, resultMat, 1.0, mBuffer.type());

    return resultMat;
}

/************/
cv::Mat Source::correctDistortion(cv::Mat pImg)
{
    if (mRecomputeDistortionMat == true || mDistortionMat.size() != mBuffer.size())
    {
        // Distortion description has changed, or grabbed image has not the same resolution
        mDistortionMat = cv::Mat::zeros(mHeight, mWidth, CV_32FC2);

        float a, b, c;
        a = mOpticalDesc.distortion[0];
        b = mOpticalDesc.distortion[1];
        c = mOpticalDesc.distortion[2];

        cv::Point2f center;
        center.x = (float)mWidth / 2.f;
        center.y = (float)mHeight / 2.f;

        float radius = std::min(center.x, center.y);
        
        for (int x = 0; x < (int)mWidth; ++x)
        {
            for (int y = 0; y < (int)mHeight; ++y)
            {
                // Compute the distance to center in normalized value
                // See http://wiki.panotools.org/Lens_correction_model for information
                float dstRadius = (sqrtf(pow((float)x-center.x, 2.f) + pow((float)y-center.y, 2.f)));
                cv::Vec2f dir;
                dir[0] = ((float)x-center.x)/dstRadius;
                dir[1] = ((float)y-center.y)/dstRadius;
                
                float srcRadius = a*pow(dstRadius/radius, 4.f)
                    + b*pow(dstRadius/radius, 3.f)
                    + c*pow(dstRadius/radius, 2.f)
                    + dstRadius/radius;

                mDistortionMat.at<cv::Vec2f>(y, x)[0] = center.x + srcRadius*radius*dir[0];
                mDistortionMat.at<cv::Vec2f>(y, x)[1] = center.y + srcRadius*radius*dir[1];
            }
        }

        mRecomputeDistortionMat = false;
    }

    cv::Mat resultMat;
    cv::remap(pImg, resultMat, mDistortionMat, cv::Mat(), cv::INTER_LINEAR);

    return resultMat;
}

/************/
cmsHTRANSFORM Source::loadICCTransform(std::string pFile)
{
    cmsHTRANSFORM transform = NULL;
    cmsHPROFILE inProfile, outProfile;
    
    // Load the specified ICC profile
    inProfile = cmsOpenProfileFromFile(pFile.c_str(), "r");
    if (inProfile == NULL)
    {
        std::cout << "Error while loading ICC profile " << pFile << std::endl;
        return transform;
    }

    outProfile = cmsCreate_sRGBProfile();
    transform = cmsCreateTransform(inProfile, TYPE_BGR_8, outProfile, TYPE_BGR_8, INTENT_RELATIVE_COLORIMETRIC, 0);

    cmsCloseProfile(inProfile);
    cmsCloseProfile(outProfile);

    std::cout << "ICC profile " << pFile << " correctly loaded" << std::endl;
    return transform;
}
