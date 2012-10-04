#include "blob.h"

/*************/
Blob::Blob()
{
    static int lIdCounter = 0;
    lIdCounter++;
    mId = lIdCounter;

    updated = false;

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
    setIdentity(mFilter.processNoiseCov, cv::Scalar::all(1e-5));
    setIdentity(mFilter.measurementNoiseCov, cv::Scalar::all(1e-5));
    setIdentity(mFilter.errorCovPost, cv::Scalar::all(1));
}

/*************/
Blob::~Blob()
{
}

/*************/
void Blob::init(properties pNewBlob)
{
    mFilter.statePre.at<float>(0) = pNewBlob.position.x;
    mFilter.statePre.at<float>(1) = pNewBlob.position.y;
    mFilter.statePre.at<float>(2) = pNewBlob.size;
    mFilter.statePre.at<float>(3) = 0.f;
    mFilter.statePre.at<float>(4) = 0.f;

    mProperties = pNewBlob;
}

/*************/
Blob::properties Blob::predict()
{
    cv::Mat lPrediction;
    properties lProperties;

    lPrediction = mFilter.predict();

    lProperties.position.x = lPrediction.at<float>(0);
    lProperties.position.y = lPrediction.at<float>(1);
    lProperties.size = lPrediction.at<float>(2);
    lProperties.speed.x = lPrediction.at<float>(3);
    lProperties.speed.x = lPrediction.at<float>(4);

    updated = false;
    
    return lProperties;
}

/*************/
void Blob::setNewMeasures(properties pNewBlob)
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
Blob::properties Blob::getBlob()
{
    return mProperties;
}

/*************/
bool Blob::isUpdated()
{
    return updated;
}
