#include "source_2d.h"

using namespace std;

std::string Source_2D::mClassName = "Source_2D";
std::string Source_2D::mDocumentation = "N/A";

/*************/
Source_2D::Source_2D()
{
    mName = mClassName;
    mDocumentation = "N/A";

    mCorrectedBuffer = cv::Mat::zeros(480, 640, CV_8UC3);

    mWidth = 0;
    mHeight = 0;
    mChannels = 0;
    mFramerate = 0;

    mExposureTime = 1.f;
    mExposureParam = 1.f;
    mAperture = 1.f;
    mGain = 0.f;
    mISO = 100.f;
    mGamma = 2.2f;

    mFilterNoise = false;

    mScale = 1.f;
    mRotation = 0.f;

    mCorrectDistortion = false;
    mCorrectFisheye = false;
    mCorrectVignetting = false;

    mOpticalDesc.distortion = 0.0;
    mOpticalDesc.fisheye = 0.0;
    mOpticalDesc.vignetting = 0.0;

    mRecomputeVignettingMat = false;
    mRecomputeDistortionMat = false;
    mRecomputeFisheyeMat = false;

    mICCTransform = NULL;

    mAutoExposureRoi = cv::Rect(0, 0, 0, 0);
    mAutoExposureTarget = 118.f; // Middle gray value, as perceived in sRGB
    mAutoExposureThreshold = 16.f;
    mAutoExposureStep = 0.05f;

    mHdriActive = false;

    mSaveToFile = false;
    mSaveIndex = 0;
    mSavePhase = 0;
}

/************/
Source_2D::Source_2D(int pParam)
{
    Source_2D();
}

/************/
Source_2D::~Source_2D()
{
    if (mICCTransform != NULL)
        cmsDeleteTransform(mICCTransform);
}

/************/
Capture_Ptr Source_2D::retrieveFrame()
{
    if (mUpdated)
    {
        cv::Mat buffer = retrieveRawFrame();
        
        if (mAutoExposureRoi.width != 0 && mAutoExposureRoi.height != 0)
            applyAutoExposure(buffer);
        if (mMask.total() != 0)
            applyMask(buffer);
        // Noise filtering and vignetting correction, as well as ICC transform and lense
        // distortion correction have to be done before any geometric transformation
        if (mFilterNoise)
            filterNoise(buffer);
        if (mCorrectVignetting)
            correctVignetting(buffer);
        if (mICCTransform != NULL)
            cmsDoTransform(mICCTransform, buffer.data, buffer.data, buffer.total());
        if (mCorrectDistortion)
            correctDistortion(buffer);
        if (mCorrectFisheye)
            correctFisheye(buffer);
        if (mScale != 1.f)
            scale(buffer);
        if (mRotation != 0.f)
            rotate(buffer);
        if (mHdriActive)
            createHdri(buffer);

        // Some modifiers will not output a valid image every frame
        if (buffer.rows != 0 && buffer.cols != 0)
            mCorrectedBuffer = buffer.clone();

        if (mSaveToFile)
            saveToFile(buffer);

        mUpdated = false;
    }

    Capture_2D_Mat_Ptr capture(new Capture_2D_Mat(mCorrectedBuffer));
    return capture;
}

/************/
void Source_2D::setBaseParameter(atom::Message pParam)
{
    std::string paramName;
    try
    {
        paramName = atom::toString(pParam[0]);
    }
    catch (atom::BadTypeTagError error)
    {
        return;
    }

    if (paramName == "mask")
    {
        string filename;
        if (!readParam(pParam, filename))
            return;

        mMask = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
    }
    else if (paramName == "vignetting")
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
    else if (paramName == "scale")
    {
        float scale;
        if (readParam(pParam, scale))
            mScale = max(0.1f, scale);
    }
    else if (paramName == "rotation")
    {
        readParam(pParam, mRotation);
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
    else if (paramName == "fisheye")
    {
        if (pParam.size() >= 2)
        {
            try
            {
                mOpticalDesc.fisheye[0] = atom::toFloat(pParam[1]);
                mOpticalDesc.fisheye[1] = atom::toFloat(pParam[2]);
            }
            catch (atom::BadTypeTagError error)
            {
                return;
            }
        }
        else
            return;

        mCorrectFisheye = true;
        mRecomputeFisheyeMat = true;
    }
    else if (paramName == "iccInputProfile")
    {
        std::string filename;
        if (!readParam(pParam, filename))
            return;

        if (mICCTransform != NULL)
            cmsDeleteTransform(mICCTransform);
        mICCTransform = loadICCTransform(filename);
    }
    else if (paramName == "exposureLUT")
    {
        int nbr;
        if (!readParam(pParam, nbr))
        {
            g_log(NULL, G_LOG_LEVEL_WARNING, "%s - Message wrongly formed for exposureLUT", mClassName.c_str());
            return;
        }

        int type;
        if (!readParam(pParam, type, 2))
        {
            g_log(NULL, G_LOG_LEVEL_WARNING, "%s - Message wrongly formed for exposureLUT", mClassName.c_str());
            return;
        }

        vector< vector<float> > keys;
        for (int i = 0; i < nbr / 2; ++i)
        {
            if (!readParam(pParam, keys[i][0], i * 2 + 3))
                return;
            if (!readParam(pParam, keys[i][1], i * 2 + 4))
                return;
        }

        mExposureLUT.set((LookupTable::interpolation)type, keys);
    }
    else if (paramName == "gainLUT")
    {
        int nbr;
        if (!readParam(pParam, nbr))
        {
            g_log(NULL, G_LOG_LEVEL_WARNING, "%s - Message wrongly formed for gainLUT", mClassName.c_str());
            return;
        }

        int type;
        if (!readParam(pParam, type, 2))
        {
            g_log(NULL, G_LOG_LEVEL_WARNING, "%s - Message wrongly formed for gainLUT", mClassName.c_str());
            return;
        }

        vector< vector<float> > keys;
        for (int i = 0; i < nbr / 2; ++i)
        {
            if (!readParam(pParam, keys[i][0], i * 2 + 3))
                return;
            if (!readParam(pParam, keys[i][1], i * 2 + 4))
                return;
        }

        mGainLUT.set((LookupTable::interpolation)type, keys);
    }
    else if (paramName == "autoExposure")
    {
        float v[7];
        for (int i = 0; i < 4; ++i)
            if (!readParam(pParam, v[i], i+1))
            {
                g_log(NULL, G_LOG_LEVEL_WARNING, "%s - Message wrongly formed for autoExposure", mClassName.c_str());
                return;
            }

        mAutoExposureRoi = cv::Rect(v[0], v[1], v[2], v[3]);

        if (readParam(pParam, v[4], 5))
            mAutoExposureTarget = v[4];
        if (readParam(pParam, v[5], 6))
            mAutoExposureThreshold = v[5];
        if (readParam(pParam, v[6], 7))
            mAutoExposureStep = v[6];
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
    else if (paramName == "save")
    {
        float active, period;
        string filename;

        if (!readParam(pParam, active, 1))
            return;
        if (!readParam(pParam, period, 2))
            return;
        if (!readParam(pParam, filename, 3))
            return;

        if (active == 1.f)
        {
            mSaveToFile = true;
            mSavePeriod = (int)period;
            mBaseFilename = filename;
        }
        else
        {
            mSaveToFile = false;
        }
    }
}

/************/
atom::Message Source_2D::getBaseParameter(atom::Message pParam) const
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
float Source_2D::getEV()
{
    // Update the exposure time in case it changes automatically
    atom::Message msg;
    msg.push_back(atom::StringValue::create("exposureTime"));
    setParameter(msg);

    return log2(mAperture*mAperture*(1/mExposureTime)*100/mISO)-mGain/6.f;
}

/************/
void Source_2D::applyMask(cv::Mat& pImg)
{
    // If not done yet, the mask is converted to float and resized accordingly to pImg
    if (pImg.rows != mMask.rows || pImg.cols != mMask.cols)
    {
        cv::Mat buffer;
        cv::resize(mMask, buffer, cv::Size(pImg.cols, pImg.rows), 0, 0, cv::INTER_NEAREST);
        mMask = buffer;
    }
    if (mMask.depth() != pImg.type())
    {
        cv::Mat buffer = cv::Mat::zeros(mMask.rows, mMask.cols, pImg.type());
        mMask.convertTo(buffer, pImg.type());
        mMask = buffer;
        mMask /= 255;
    }

    cv::multiply(mMask, pImg, pImg);
}

/************/
void Source_2D::filterNoise(cv::Mat& pImg)
{
    // We apply a simple median filter of size 1px to reduce noise
    cv::medianBlur(pImg, pImg, 3);
}

/************/
void Source_2D::scale(cv::Mat& pImg)
{
    cv::Mat output;
    cv::resize(pImg, output, cv::Size(), mScale, mScale, cv::INTER_LINEAR);
    pImg = output;
}

/************/
void Source_2D::rotate(cv::Mat& pImg)
{
    cv::Point2f center = cv::Point2f((float)pImg.cols / 2.f, (float)pImg.rows / 2.f);
    cv::Mat rotMat = cv::getRotationMatrix2D(center, mRotation, 1.0);
    cv::Mat rotatedMat;
    cv::warpAffine(pImg, rotatedMat, rotMat, cv::Size(pImg.cols, pImg.rows), cv::INTER_LINEAR);
    pImg = rotatedMat;
}

/************/
void Source_2D::correctVignetting(cv::Mat& pImg)
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
                    mVignettingMat.at<cv::Vec3f>(y, x)[c] = correction;
            }
        }

        mRecomputeVignettingMat = false;
    }

    cv::multiply(pImg, mVignettingMat, pImg, 1.0, pImg.type());
}

/************/
void Source_2D::correctDistortion(cv::Mat& pImg)
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
void Source_2D::correctFisheye(cv::Mat& pImg)
{
    if (mRecomputeFisheyeMat == true || mFisheyeMat.size() != pImg.size())
    {
        mFisheyeMat = cv::Mat::zeros(mHeight, mWidth, CV_32FC2);
        float inFocal = mOpticalDesc.fisheye[0];
        float outFocal = mOpticalDesc.fisheye[1];

        cv::Point2f center;
        center.x = (float)mWidth / 2.f;
        center.y = (float)mHeight / 2.f;

        // See http://wiki.panotools.org/Fisheye_Projection for more information
        float radius = std::min(center.x, center.y);

        for (int x = 0; x < (int)mWidth; ++x)
        {
            for (int y = 0; y < (int)mHeight; ++y)
            {
                float dstRadius = sqrtf(pow((float)x - center.x, 2.f) + pow((float)y - center.y, 2.f));
                cv::Vec2f dir;
                dir[0] = ((float)x - center.x) / dstRadius;
                dir[1] = ((float)y - center.y) / dstRadius;

                float angle = atan(dstRadius / outFocal);
                float srcRadius = inFocal * angle;

                mFisheyeMat.at<cv::Vec2f>(y, x)[0] = center.x + srcRadius*dir[0];
                mFisheyeMat.at<cv::Vec2f>(y, x)[1] = center.y + srcRadius*dir[1];
            }
        }

        mRecomputeFisheyeMat = false;
    }

    cv::Mat resultMat;
    cv::remap(pImg, resultMat, mFisheyeMat, cv::Mat(), cv::INTER_LINEAR);

    pImg = resultMat;
}

/************/
cmsHTRANSFORM Source_2D::loadICCTransform(std::string pFile)
{
    cmsHTRANSFORM transform = NULL;
    cmsHPROFILE inProfile, outProfile;
    
    // Load the specified ICC profile
    inProfile = cmsOpenProfileFromFile(pFile.c_str(), "r");
    if (inProfile == NULL)
    {
        g_log(NULL, G_LOG_LEVEL_WARNING, "%s - Error while loading ICC profile %s", mClassName.c_str(), pFile.c_str());
        return transform;
    }

    outProfile = cmsCreate_sRGBProfile();
    transform = cmsCreateTransform(inProfile, TYPE_BGR_8, outProfile, TYPE_BGR_8, INTENT_PERCEPTUAL, 0);

    cmsCloseProfile(inProfile);
    cmsCloseProfile(outProfile);

    g_log(NULL, G_LOG_LEVEL_INFO, "%s - ICC profile %s correctly loaded", mClassName.c_str(), pFile.c_str());
    return transform;
}

/*************/
void Source_2D::applyAutoExposure(cv::Mat& pImg)
{
    if (pImg.channels() != 3)
        return;

    // TODO: Allow to chose the colorspace for luminance computation
    // Compute the luminance in the ROI
    cv::Rect roi = mAutoExposureRoi;
    roi.x = min(roi.x, pImg.cols - 1);
    roi.y = min(roi.y, pImg.rows - 1);
    roi.width = min(roi.width, pImg.cols - 1 - roi.x);
    roi.height = min(roi.height, pImg.rows - 1 - roi.y);

    cv::Mat buffer;
    pImg.convertTo(buffer, CV_32FC3);
    buffer /= 255.f;
    cv::pow(buffer, 1/mGamma, buffer); // Conversion from sRGB to RGB

    float luminance = 0.f;
    float pixelNumber = 0.f;
    for (int x = roi.x; x < roi.x + roi.width && x < buffer.cols; ++x)
        for (int y = roi.y; y < roi.y + roi.height && x < buffer.rows; ++y)
        {
            float r, g, b;
            r = buffer.at<cv::Vec3f>(y, x)[0];
            g = buffer.at<cv::Vec3f>(y, x)[1];
            b = buffer.at<cv::Vec3f>(y, x)[2];

            luminance += 0.2126f * r + 0.7152f * g + 0.0722f * b;
            pixelNumber += 1.f;
        }

    if (pixelNumber == 0)
        return;

    luminance = pow(luminance / pixelNumber, mGamma) * 255.f;
    // If we don't need to update exposure ...
    if (abs(luminance - mAutoExposureTarget) < mAutoExposureThreshold)
        return;

    // We set the exposure "mAutoExposureStep" higher.
    float exposure = mExposureTime * (1.f + mAutoExposureStep * (mAutoExposureTarget - luminance) / abs(luminance - mAutoExposureTarget)); 
    if (exposure == 0.f) // We don't want to be stuck at the lowest value
        return;

    atom::Message message;
    message.push_back(atom::StringValue::create("exposureTime"));
    message.push_back(atom::FloatValue::create(exposure));
    setParameter(message);

    g_log(NULL, G_LOG_LEVEL_DEBUG, "%s %i %i: exposureTime  %f", mName.c_str(), mSubsourceNbr, mId, exposure);

    // Lastly, we log-broadcast the changes
    g_log(LOG_BROADCAST, G_LOG_LEVEL_INFO, "exposureTime %s %i %i %f", mName.c_str(), mSubsourceNbr, mId, exposure);
}

/*************/
void Source_2D::createHdri(cv::Mat& pImg)
{
    // TODO: make this work even if cameras dont send the right exposure value

    static int ldriCount = -1;
    // If we just started HDRI capture, we need to set the exposure to the start value
    atom::Message message;
    message.push_back(atom::StringValue::create("exposureTime"));
    if (ldriCount == -1)
    {
        message.push_back(atom::FloatValue::create(mHdriStartExposure));
        setParameter(message);
        ldriCount++;
        return;
    }

    // Add current frame to the HdriBuilder
    mHdriBuilder.addLDR(&pImg, getEV());
    ldriCount++;

    // Change the exposure time for the next frame
    if (ldriCount < mHdriSteps)
    {
        message.push_back(atom::FloatValue::create(mExposureParam*pow(2.0, mHdriStepSize)));
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
void Source_2D::saveToFile(cv::Mat& pImg)
{
    if (mSavePhase == 0)
    {
        char buffer[16];
        sprintf(buffer, "%05i", mSaveIndex);
        string filename = mBaseFilename + string(buffer);
        if (pImg.depth() == CV_8U || pImg.depth() == CV_16U)
        {
            filename += string(".png");
            cv::imwrite(filename, pImg);
        }

        mSaveIndex++;
        mSavePhase++;
    }
    else
    {
        mSavePhase = (mSavePhase + 1) % mSavePeriod;
    }
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
cv::Mat MatBuffer::get() const
{
    unsigned int loc = _head;
    return _mats[loc];
}
