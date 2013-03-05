#include "detector.h"

std::string Detector::mClassName = "Detector";
std::string Detector::mDocumentation = "N/A";

/*************/
Detector::Detector()
{
    mName = mClassName;
    mSourceNbr = 1;

    mVerbose = true;

    mOutputBuffer = cv::Mat::zeros(0, 0, CV_8U);
    // By default, the mask is all white (all pixels are used)
    mSourceMask = cv::Mat::ones(1, 1, CV_8U);
}

/*****************/
Detector::Detector(int pParam)
{
    Detector();
}

/*****************/
void Detector::setMask(cv::Mat pMask)
{
    mSourceMask = pMask.clone();
    mMask = pMask.clone();
}

/*****************/
atom::Message Detector::getParameter(atom::Message pParam)
{
    atom::Message message;

    if (pParam.size() < 1)
        return message;
        
    std::string param;
    try
    {
        param = atom::toString(pParam[0]);
    }
    catch (...)
    {
        return message;
    }

    message.push_back(pParam[0]);
    if (param == "name")
        message.push_back(atom::StringValue::create(mClassName.c_str()));
    else if (param == "osc path")
        message.push_back(atom::StringValue::create(mOscPath.c_str()));

    return message;
}

/*****************/
void Detector::setBaseParameter(atom::Message pParam)
{
    atom::Message message;

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

    if (paramName == "verbose")
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
        mVerbose = value;
    }
}

/*****************/
void Detector::addSource(shared_ptr<Source> source)
{
    mSources.push_back(source);
}

/*****************/
cv::Mat Detector::getMask(cv::Mat pCapture, int pInterpolation)
{
    if (pCapture.rows != mMask.rows || pCapture.cols != mMask.cols)
        cv::resize(mSourceMask, mMask, pCapture.size(), 0, 0, pInterpolation);

    return mMask;
}

/*****************/
cv::Mat getLeastSumForLevel(cv::Mat pConfig, cv::Mat* pDistances, int pLevel, cv::Mat pAttributed, float &pSum, int pShift)
{
    // If we lost one or more blobs, we will need to shift the remaining blobs to test all
    // the possible combinations
    int lLevelRemaining = pConfig.rows - (pLevel + 1);
    int lMaxShift = std::max(0, std::min(pConfig.rows - pDistances->rows - pShift, lLevelRemaining));

    float lMinSum = std::numeric_limits<float>::max();
    float lCurrentSum;
    cv::Mat lAttributed;
    cv::Mat lConfig, lCurrentConfig;

    // We try without shifting anything
    for(int i = 0; i < pAttributed.rows + lMaxShift; ++i)
    {

        // If we do not shift
        if(i < pAttributed.rows)
        {
            if(pAttributed.at<uchar>(i) == 0)
            {    
                lAttributed = pAttributed.clone();
                lCurrentConfig = pConfig.clone();

                lAttributed.at<uchar>(i) = 255;
                lCurrentSum = pSum + pDistances->at<float>(i, pLevel);
                lCurrentConfig.at<uchar>(pLevel) = i;

                if(lLevelRemaining > 0)
                    lCurrentConfig = getLeastSumForLevel(lCurrentConfig, pDistances, pLevel + 1, lAttributed, lCurrentSum, pShift);

                if(lCurrentSum < lMinSum)
                {
                    lMinSum = lCurrentSum;
                    lConfig = lCurrentConfig;
                }
            }
        }
        // if we shift, don't attribute this keypoint to any blob
        else if(i >= pAttributed.rows)
        {
            lAttributed = pAttributed.clone();
            lCurrentConfig = pConfig.clone();
            lCurrentSum = pSum;

            if(lLevelRemaining > 0)
                lCurrentConfig = getLeastSumForLevel(lCurrentConfig, pDistances, pLevel + 1, lAttributed, lCurrentSum, pShift + 1);
            
            if(lCurrentSum < lMinSum)
            {
                lMinSum = lCurrentSum;
                lConfig = lCurrentConfig;
            }
        }
    }

    if(lMinSum < std::numeric_limits<float>::max())
    {
        pSum = lMinSum;
        return lConfig;
    }
    else
    {
        return pConfig;
    }
}

/*****************/
cv::Mat getLeastSumConfiguration(cv::Mat* pDistances)
{
    float lMinSum = 0.f;
    cv::Mat lConfiguration = cv::Mat::ones(pDistances->cols, 1, CV_8U)*255;
    cv::Mat lAttributed = cv::Mat::zeros(pDistances->rows, 1, CV_8U);

    lConfiguration = getLeastSumForLevel(lConfiguration, pDistances, 0, lAttributed, lMinSum, 0);

    return lConfiguration;
}
