#include "detector.h"

/*************/
Detector::Detector()
{
    mOutputBuffer = cv::Mat::zeros(0, 0, CV_8U);
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
