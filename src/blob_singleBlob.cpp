#include "blob_singleBlob.h"

/*************/
SingleBlob::SingleBlob()
{
    mProperties.position.x = 0.0;
    mProperties.position.y = 0.0;
    mProperties.size = 0.0;
    mProperties.speed.x = 0.0;
    mProperties.speed.y = 0.0;

    // We are filtering a 3 variables state, and
    // we have a measure for all of them
    mFilter.init(5, 3, 0);

    mFilter.transitionMatrix = *(cv::Mat_<float>(5, 5) << 1,0,0,1,0, 0,1,0,0,1, 0,0,1,0,0, 0,0,0,1,0, 0,0,0,0,1);
    setIdentity(mFilter.measurementMatrix);
    setIdentity(mFilter.processNoiseCov, cv::Scalar::all(1e-3));
    setIdentity(mFilter.measurementNoiseCov, cv::Scalar::all(1e-3));
    setIdentity(mFilter.errorCovPost, cv::Scalar::all(1));
}

/*************/
void SingleBlob::init(properties pNewBlob)
{
    mFilter.statePre.at<float>(0) = pNewBlob.position.x;
    mFilter.statePre.at<float>(1) = pNewBlob.position.y;
    mFilter.statePre.at<float>(2) = pNewBlob.size;
    mFilter.statePre.at<float>(3) = 0.f;
    mFilter.statePre.at<float>(4) = 0.f;

    mProperties = pNewBlob;
}

/*************/
Blob::properties SingleBlob::predict()
{
    cv::Mat lPrediction;
    properties lProperties;

    lPrediction = mFilter.predict();

    mPrediction.position.x = lPrediction.at<float>(0);
    mPrediction.position.y = lPrediction.at<float>(1);
    mPrediction.size = lPrediction.at<float>(2);
    mPrediction.speed.x = lPrediction.at<float>(3);
    mPrediction.speed.x = lPrediction.at<float>(4);

    updated = false;
    
    return mPrediction;
}

/*************/
void SingleBlob::setNewMeasures(properties pNewBlob)
{
    cv::Mat lMeasures = cv::Mat::zeros(3, 1, CV_32F);
    lMeasures.at<float>(0) = pNewBlob.position.x;
    lMeasures.at<float>(1) = pNewBlob.position.y;
    lMeasures.at<float>(2) = pNewBlob.size;

    cv::Mat lEstimation = mFilter.correct(lMeasures);
    mProperties.position.x = lEstimation.at<float>(0);
    mProperties.position.y = lEstimation.at<float>(1);
    mProperties.size = lEstimation.at<float>(2);
    mProperties.speed.x = lEstimation.at<float>(3);
    mProperties.speed.y = lEstimation.at<float>(4);

    updated = true;
}

/*************/
float SingleBlob::getDistanceFromPrediction(properties pBlob)
{
    float distance = pow(pBlob.position.x - mPrediction.position.x, 2.0)
        + pow(pBlob.position.y - mPrediction.position.y, 2.0)
        + pow(pBlob.size - mPrediction.size, 2.0);

    return distance;
}
