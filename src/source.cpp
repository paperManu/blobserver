#include "source.h"

std::string Source::mClassName = "Source";
std::string Source::mDocumentation = "N/A";

/*************/
Source::Source():
    mUpdated(false)
{
    mName = mClassName;
    mDocumentation = "N/A";

    mCorrectedBuffer = cv::Mat::zeros(480, 640, CV_8UC3);

    mWidth = 0;
    mHeight = 0;
    mChannels = 0;
    mFramerate = 0;

    mExposureTime = 1.f;
    mAperture = 1.f;
    mGain = 0.f;
    mISO = 100.f;

    mSubsourceNbr = 0;
    mId = 0;

    mFilterNoise = false;

    mCorrectDistortion = false;
    mCorrectVignetting = false;

    mOpticalDesc.distortion = 0.0;
    mOpticalDesc.vignetting = 0.0;

    mRecomputeVignettingMat = false;
    mRecomputeDistortionMat = false;

    mICCTransform = NULL;

    mHdriActive = false;
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
        cv::Mat buffer = retrieveFrame();

        if (mFilterNoise)
            filterNoise(buffer);
        if (mCorrectVignetting)
            correctVignetting(buffer);
        if (mICCTransform != NULL)
            cmsDoTransform(mICCTransform, buffer.data, buffer.data, buffer.total());
        if (mCorrectDistortion)
            correctDistortion(buffer);
        if (mHdriActive)
            createHdri(buffer);

        // Some modifiers will not output a valid image every frame
        if (buffer.rows != 0 && buffer.cols != 0)
            mCorrectedBuffer = buffer;

        mUpdated = false;

        return mCorrectedBuffer;
    }
    else
    {
        return mCorrectedBuffer;
    }
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
    else if (paramName == "noiseFiltering")
    {
        if (pParam.size() == 2)
        {
            int value;
            try
            {
                value = atom::toInt(pParam[1]);
            }
            catch (atom::BadTypeTagError error)
            {
                return;
            }

            if (value)
                mFilterNoise = true;
            else
                mFilterNoise = false;
        }
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
    else if (paramName == "hdri")
    {
        if (pParam.size() != 4)
            return;

        try
        {
            mHdriStartExposure = atom::toFloat(pParam[1]);
            mHdriStepSize = atom::toFloat(pParam[2]);
            mHdriSteps = atom::toInt(pParam[3]);
        }
        catch (atom::BadTypeTagError error)
        {
            return;
        }

        mHdriActive = true;
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
float Source::getEV()
{
    return log2(mAperture*mAperture*(1/mExposureTime)*100/mISO)-mGain/6.f;
}

/************/
void Source::filterNoise(cv::Mat& pImg)
{
    // We apply a simple median filter of size 1px to reduce noise
    cv::medianBlur(pImg, pImg, 3);
}

/************/
void Source::correctVignetting(cv::Mat& pImg)
{
    if (mRecomputeVignettingMat == true || mVignettingMat.size() != pImg.size())
    {
        // Vignetting description has changed, or grabbed image has not the same resolution
        int nbrChannels = (pImg.type() >> CV_CN_SHIFT) + 1;
        int type = CV_MAKE_TYPE(CV_32F, nbrChannels);
        mVignettingMat = cv::Mat::zeros(mHeight, mWidth, type);

        cv::Point2f center;
        center.x = (float)mWidth / 2.f;
        center.y = (float)mHeight / 2.f;
        
        float sqfactor = pow(center.y, 2.f) + pow(center.x, 2.f);

        for (int x = 0; x < (int)mWidth; ++x)
        {
            for (int y = 0; y < (int)mHeight; ++y)
            {
                float sqradius = (pow((float)x-center.x, 2.f) + pow((float)y-center.y, 2.f))/sqfactor;
                float correction = 1.f / (1.f + mOpticalDesc.vignetting[0] * sqradius
                    + mOpticalDesc.vignetting[1] * pow(sqradius, 2.f)
                    + mOpticalDesc.vignetting[2] * pow(sqradius, 4.f));

                for (int c = 0; c < nbrChannels; ++c)
                    mVignettingMat.at<float>(x*nbrChannels + mWidth*nbrChannels*y) = correction;
            }
        }

        mRecomputeVignettingMat = false;
    }

    cv::multiply(pImg, mVignettingMat, pImg, 1.0, pImg.type());
}

/************/
void Source::correctDistortion(cv::Mat& pImg)
{
    if (mRecomputeDistortionMat == true || mDistortionMat.size() != pImg.size())
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

    pImg = resultMat;
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
    transform = cmsCreateTransform(inProfile, TYPE_BGR_8, outProfile, TYPE_BGR_8, INTENT_PERCEPTUAL, 0);

    cmsCloseProfile(inProfile);
    cmsCloseProfile(outProfile);

    std::cout << "ICC profile " << pFile << " correctly loaded" << std::endl;
    return transform;
}

/*************/
void Source::createHdri(cv::Mat& pImg)
{
    static int ldriCount = -1;
    // If we just started HDRI capture, we need to set the exposure to the start value
    atom::Message message;
    message.push_back(atom::StringValue::create("exposureTime"));
    if (ldriCount == -1)
    {
        message.push_back(atom::FloatValue::create(mHdriStartExposure));
        setParameter(message);
        return;
    }

    // Add current frame to the HdriBuilder
    mHdriBuilder.addLDR(&pImg, getEV());
    ldriCount++;

    // Change the exposure time for the next frame
    if (ldriCount < mHdriSteps)
    {
        message.push_back(atom::FloatValue::create(mExposureTime*pow(2.0, mHdriStepSize)));
        pImg.create(480, 640, CV_32FC3);
    }
    else
    {
        mHdriBuilder.computeHDRI();
        pImg = mHdriBuilder.getHDRI();

        message.push_back(atom::FloatValue::create(mHdriStartExposure));
        ldriCount = 0;
    }
    setParameter(message);
}

/*************/
MatBuffer::MatBuffer(unsigned int size)
{
    _mats.resize(size);
    cv::Mat dummyMat = cv::Mat::zeros(480, 640, CV_8UC3);
    _mats[0] = dummyMat;
    _head = 0;
}

/*************/
MatBuffer& MatBuffer::operator=(cv::Mat& mat)
{
    unsigned int head = _head;
    unsigned int loc = (head+1) % _mats.size();
    _mats[loc] = mat;
    _head = loc;

    return *this;
}

/*************/
cv::Mat MatBuffer::get()
{
    unsigned int loc = _head;
    return _mats[loc];
}
