#include "blob_2D.h"

using namespace std;

/*************/
Blob2D::Blob2D()
{
    mProperties.position.x = 0.0;
    mProperties.position.y = 0.0;
    mProperties.size = 0.0;
    mProperties.speed.x = 0.0;
    mProperties.speed.y = 0.0;

    // We are filtering a 2 variables state, and
    // we have a measure for all of them
    mFilter.init(4, 2, 0);

    mFilter.transitionMatrix = *(cv::Mat_<float>(4, 4) << 1,0,1,0, 0,1,0,1, 0,0,1,0, 0,0,0,1);
    setIdentity(mFilter.measurementMatrix);
    setIdentity(mFilter.processNoiseCov, cv::Scalar::all(1e-6));
    setIdentity(mFilter.measurementNoiseCov, cv::Scalar::all(1e-4));
    setIdentity(mFilter.errorCovPost, cv::Scalar::all(1));
}

/*************/
void Blob2D::setParameter(const char* pParam, float pValue)
{
    if (strcmp(pParam, "processNoiseCov") == 0)
    {
        if(pValue > 0.0)
            setIdentity(mFilter.processNoiseCov, cv::Scalar::all(pValue));
    }
    else if (strcmp(pParam, "measurementNoiseCov") == 0)
    {
        if(pValue > 0.0)
            setIdentity(mFilter.measurementNoiseCov, cv::Scalar::all(pValue));
    }
    else
        std::cout << __FILE__ << " - " << __FUNCTION__ << " - Wrong parameters." << std::endl;
}

/*************/
void Blob2D::init(properties pNewBlob)
{
    mFilter.statePre.at<float>(0) = pNewBlob.position.x;
    mFilter.statePre.at<float>(1) = pNewBlob.position.y;
    mFilter.statePre.at<float>(2) = 0.f;
    mFilter.statePre.at<float>(3) = 0.f;

    mProperties = pNewBlob;

    mPrediction.position.x = pNewBlob.position.x;
    mPrediction.position.y = pNewBlob.position.y;
}

/*************/
Blob::properties Blob2D::predict()
{
    cv::Mat lPrediction;
    properties lProperties;

    // If no new measure was set, we update the position with
    // predicted values. This is needed to keep some coherency
    // to the speed value.
    if (updated == false)
    {
        cv::Mat lPrediction = cv::Mat::zeros(2, 1, CV_32F);
        lPrediction.at<float>(0) = mPrediction.position.x;
        lPrediction.at<float>(1) = mPrediction.position.y;

        mFilter.correct(lPrediction);

        mProperties.position.x = mPrediction.position.x;
        mProperties.position.y = mPrediction.position.y;
        mProperties.size = mPrediction.size;
    }

    lPrediction = mFilter.predict();

    mPrediction.position.x = lPrediction.at<float>(0);
    mPrediction.position.y = lPrediction.at<float>(1);
    mPrediction.speed.x = lPrediction.at<float>(2);
    mPrediction.speed.x = lPrediction.at<float>(3);

    updated = false;
    
    return mPrediction;
}

/*************/
void Blob2D::setNewMeasures(properties pNewBlob)
{
    mProperties.size = pNewBlob.size;

    cv::Mat lMeasures = cv::Mat::zeros(2, 1, CV_32F);
    lMeasures.at<float>(0) = pNewBlob.position.x;
    lMeasures.at<float>(1) = pNewBlob.position.y;

    cv::Mat lEstimation = mFilter.correct(lMeasures);
    mProperties.position.x = lEstimation.at<float>(0);
    mProperties.position.y = lEstimation.at<float>(1);
    mProperties.speed.x = lEstimation.at<float>(2);
    mProperties.speed.y = lEstimation.at<float>(3);

    updated = true;
}

/*************/
float Blob2D::getDistanceFromPrediction(properties pBlob)
{
    float distance = pow(pBlob.position.x - mPrediction.position.x, 2.0)
        + pow(pBlob.position.y - mPrediction.position.y, 2.0);

    return distance;
}
