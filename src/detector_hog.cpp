#include "detector_hog.h"

#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>

using namespace std;
using namespace chrono;

/*****************/
// Class for parallel detection
/*****************/
class Parallel_Detect : public cv::ParallelLoopBody
{
    public:
        Parallel_Detect(const vector<cv::Point>* points, vector<cv::Point>* samples, const int size,
            const Descriptor_Hog* descriptor, const CvSVM* svm):
            _points(points), _samples(samples), _size(size), _descriptor(descriptor), _svm(svm)
        {
            mMutex.reset(new mutex());
        }

        void operator()(const cv::Range& r) const
        {
            vector<float> description;
            cv::Mat descriptionMat;
            for (int idx = r.start; idx < r.end; ++idx)
            {
                const cv::Point point = (*_points)[idx];
                description = _descriptor->getDescriptor(point);
                if (description.size() == 0)
                    continue;
                descriptionMat = cv::Mat(1, description.size(), CV_32FC1, &description[0]);

                if (_svm->predict(descriptionMat) == 1.f)
                {
                    lock_guard<mutex> lock(*mMutex.get());
                    _samples->push_back(point);
                }
            }
        }

    private:
        const vector<cv::Point>* _points;
        vector<cv::Point>* _samples;
        const int _size;
        const Descriptor_Hog* _descriptor;
        const CvSVM* _svm;

        shared_ptr<mutex> mMutex;
};

/*************/
// Definition of class Detector_Hog
/*************/
std::string Detector_Hog::mClassName = "Detector_Hog";
std::string Detector_Hog::mDocumentation = "N/A";
unsigned int Detector_Hog::mSourceNbr = 1;

/*************/
Detector_Hog::Detector_Hog()
{
    make();
}

/*************/
Detector_Hog::Detector_Hog(int pParam)
{
    make();
}

/*************/
void Detector_Hog::make()
{
    mOutputBuffer = cv::Mat::zeros(480, 640, CV_8UC3);

    mName = mClassName;
    mOscPath = "/blobserver/hog";

    mFilterSize = 3;
    mFilterDilateCoeff = 3;

    mRoiSize = cv::Point_<int>(64, 128);
    mBlockSize = cv::Point_<int>(2, 2);
    mCellSize = cv::Point_<int>(16, 16);
    mBins = 9;
    mSigma = 0.f;
    updateDescriptorParams();

    mIsModelLoaded = false;
    mMaxTimePerFrame = 1e5;
    mMaxThreads = 4;

    mBlobMergeDistance = 64.f;
}

/*************/
atom::Message Detector_Hog::detect(const vector<cv::Mat> pCaptures)
{
    unsigned long long timeStart = duration_cast<microseconds>(high_resolution_clock::now().time_since_epoch()).count();

    if (pCaptures.size() == 0 || !mIsModelLoaded)
        return mLastMessage;

    // For simplicity...
    cv::Mat input = pCaptures[0];

    // We get windows of interest, using BG subtraction
    // and previous blobs positions
    mBgSubtractor(input, mBgSubtractorBuffer);
    // Erode and dilate to suppress noise
    cv::Mat lEroded;
    cv::erode(mBgSubtractorBuffer, lEroded, cv::Mat(), cv::Point(-1, -1), mFilterSize);
    cv::dilate(lEroded, mBgSubtractorBuffer, cv::Mat(), cv::Point(-1, -1), mFilterSize * mFilterDilateCoeff);
    cv::threshold(mBgSubtractorBuffer, mBgSubtractorBuffer, 250, 255, cv::THRESH_BINARY);

    // The result is translated so that the window will be correctly place for the given pixel
    {
        cv::Mat transMat = cv::Mat::zeros(2, 3, CV_32F);
        transMat.at<float>(0, 0) = 1.f;
        transMat.at<float>(1, 1) = 1.f;
        transMat.at<float>(0, 2) = -(float)mRoiSize.width * 0.33f;
        transMat.at<float>(1, 2) = -(float)mRoiSize.height * 0.33f;
        cv::Mat translatedMat;
        cv::warpAffine(mBgSubtractorBuffer, translatedMat, transMat, cv::Size(mBgSubtractorBuffer.cols, mBgSubtractorBuffer.rows), cv::INTER_LINEAR);
        mBgSubtractorBuffer = translatedMat;
    }

    // We draw rectangles to handle previously detected blobs
    for_each (mBlobs.begin(), mBlobs.end(), [&] (Blob2D blob)
    {
        Blob::properties props = blob.getBlob();
        cv::Rect rect(props.position.x - props.size/2, props.position.y - props.size/2, props.size, props.size);
        cv::rectangle(mBgSubtractorBuffer, rect, 255, CV_FILLED);
    } );

    // The result is resized according to cell size
    cv::Size outputSize;
    outputSize.width = mBgSubtractorBuffer.cols / mCellSize.width;
    outputSize.height = mBgSubtractorBuffer.rows / mCellSize.height;
    cv::Mat resizedBuffer;
    cv::resize(mBgSubtractorBuffer, resizedBuffer, outputSize, 0, 0, cv::INTER_NEAREST);

    // We feed the image to the descriptor
    mDescriptor.setImage(input);

    // We fill the vector of all positions to test
    if (mSvmValidPositions.capacity() != outputSize.width * outputSize.height)
        mSvmValidPositions.reserve(outputSize.width * outputSize.height);

    int validPositions = 0;
    for (int x = 0; x < resizedBuffer.cols; ++x)
        for (int y = 0; y < resizedBuffer.rows; ++y)
        {
            if (resizedBuffer.at<uchar>(y, x) < 255)
                continue;

            vector<cv::Point>::iterator it = mSvmValidPositions.begin() + validPositions;
            *it = cv::Point(x, y);
            validPositions++;
        }
    int totalSamples = validPositions;

    // We go randomly through this list
    vector<cv::Point> samples;
    vector<float> description;
    cv::Mat descriptionMat;

    unsigned long long timePresent = duration_cast<microseconds>(high_resolution_clock::now().time_since_epoch()).count();
    while (validPositions && timePresent - timeStart < mMaxTimePerFrame)
    {
        vector<cv::Point> points;
        int nbrPoints = min(mMaxThreads, validPositions);
        for (int i = 0; i < nbrPoints; ++i)
        {
            unsigned int random = mRng();
            unsigned int position = random % validPositions;
            vector<cv::Point>::iterator it = mSvmValidPositions.begin() + position;
            cv::Point point = *it;
            vector<cv::Point>::iterator lastIt = mSvmValidPositions.begin() + validPositions - 1;
            swap(*lastIt, *it);

            point.x *= mCellSize.width;
            point.y *= mCellSize.height;

            validPositions--;
            points.push_back(point);
        }

        cv::parallel_for_(cv::Range(0, nbrPoints), Parallel_Detect(&points, &samples, mRoiSize.width, &mDescriptor, &mSvm));

        timePresent = duration_cast<microseconds>(high_resolution_clock::now().time_since_epoch()).count();
    }

    // A single object can be detected by multiple windows.
    // We need to merge them
    for (int i = 0; i < samples.size(); ++i)
    {
        float meanFactor = 1.f;
        for (int j = i + 1; j < samples.size();)
        {
            float distance = sqrtf(pow(samples[i].x - samples[j].x, 2.f) + pow(samples[i].y - samples[j].y, 2.f));
            if (distance < mBlobMergeDistance)
            {
                meanFactor++;
                samples[i].x = (int)((float)samples[i].x * (meanFactor - 1.f)/meanFactor + (float)samples[j].x * 1.f / meanFactor);
                samples[i].y = (int)((float)samples[i].y * (meanFactor - 1.f)/meanFactor + (float)samples[j].y * 1.f / meanFactor);

                vector<cv::Point>::iterator it = samples.begin() + j;
                samples.erase(it);
            }
            else
                j++;
        }
    }

    // We create the properties which will be converted to blobs
    vector<Blob::properties> properties;
    for (int i = 0; i < samples.size(); ++i)
    {
        Blob::properties propertie;
        propertie.position.x = samples[i].x;
        propertie.position.y = samples[i].y;
        propertie.size = mRoiSize.width;
        propertie.speed.x = 0.f;
        propertie.speed.y = 0.f;

        properties.push_back(propertie);
    }

    // We want to track them
    trackBlobs<Blob2D>(properties, mBlobs, 30);

    cv::Mat resultMat = cv::Mat::zeros(input.rows, input.cols, CV_8UC3);
    for_each (mBlobs.begin(), mBlobs.end(), [&] (Blob2D blob)
    {
        Blob::properties props = blob.getBlob();
        cv::Rect rect(props.position.x, props.position.y, mRoiSize.width, mRoiSize.height);
        cv::rectangle(resultMat, rect, cv::Scalar(1, 1, 1), CV_FILLED);
    } );

    // The result is shown
    cv::multiply(input, resultMat, resultMat);

    if (mVerbose)
        cout << "Detector_Hog - Evaluated ratio = " << 1.f - (float)validPositions / (float)totalSamples << endl;

    // Constructing the message
    mLastMessage.clear();
    mLastMessage.push_back(atom::IntValue::create((int)mBlobs.size()));
    mLastMessage.push_back(atom::IntValue::create(6));

    for(int i = 0; i < mBlobs.size(); ++i)
    {
        int lX, lY, lSize, ldX, ldY, lId;
        Blob::properties properties = mBlobs[i].getBlob();
        lX = (int)(properties.position.x);
        lY = (int)(properties.position.y);
        lSize = (int)(properties.size);
        ldX = (int)(properties.speed.x);
        ldY = (int)(properties.speed.y);
        lId = (int)mBlobs[i].getId();

        // Print the blob number on the blob
        if (mVerbose)
        {
            char lNbrStr[8];
            sprintf(lNbrStr, "%i", lId);
            cv::putText(resultMat, lNbrStr, cv::Point(lX, lY), cv::FONT_HERSHEY_COMPLEX, 0.66, cv::Scalar(128.0, 128.0, 128.0, 128.0));
        }

        // Add this blob to the message
        mLastMessage.push_back(atom::IntValue::create(lX));
        mLastMessage.push_back(atom::IntValue::create(lY));
        mLastMessage.push_back(atom::IntValue::create(lSize));
        mLastMessage.push_back(atom::IntValue::create(ldX));
        mLastMessage.push_back(atom::IntValue::create(ldY));
        mLastMessage.push_back(atom::IntValue::create(lId));
    }

    mOutputBuffer = resultMat.clone();

    return mLastMessage;
}

/*************/
void Detector_Hog::setParameter(atom::Message pMessage)
{
    atom::Message::const_iterator iter = pMessage.begin();

    std::string cmd;
    try
    {
        cmd = toString(pMessage[0]);
    }
    catch (atom::BadTypeTagError error)
    {
        return;
    }

    if (cmd == "modelFilename")
    {
        if (pMessage.size() < 2)
            return;

        string filename;
        try
        {
            filename = atom::toString(pMessage[1]);
        }
        catch (atom::BadTypeTagError error)
        {
            return;
        }

        cout << "Attemping to load SVM model from file " << filename << endl;
        mSvm.load(filename.c_str());
        mIsModelLoaded = true;
    }
    else if (cmd == "maxTimePerFrame")
    {
        if (pMessage.size() < 2)
            return;

        int duration;
        try
        {
            duration = atom::toInt(pMessage[1]);
        }
        catch (atom::BadTypeTagError error)
        {
            return;
        }

        mMaxTimePerFrame = max(33000, duration);
    }
    else if (cmd == "maxThreads")
    {
        if (pMessage.size() < 2)
            return;

        int nbr;
        try
        {
            nbr = atom::toInt(pMessage[1]);
        }
        catch (atom::BadTypeTagError error)
        {
            return;
        }

        mMaxThreads = max(1, nbr);
    }
    else if (cmd == "mergeDistance")
    {
        if (pMessage.size() < 2)
            return;

        float distance;
        try
        {
            distance = atom::toFloat(pMessage[1]);
        }
        catch (atom::BadTypeTagError error)
        {
            return;
        }

        mBlobMergeDistance = max(16.f, distance);
    }
    else if (cmd == "filterSize")
    {
        if (pMessage.size() < 2)
            return;

        int filterSize;
        try
        {
            filterSize = atom::toFloat(pMessage[1]);
        }
        catch (atom::BadTypeTagError error)
        {
            return;
        }

        mFilterSize = max(1, filterSize);
    }
    else if (cmd == "roiSize")
    {
        if (pMessage.size() < 2)
            return;

        string roiStr;
        try
        {
            roiStr = atom::toString(pMessage[1]);
        }
        catch (atom::BadTypeTagError error)
        {
            return;
        }

        cv::Size_<int> roiSize;
        sscanf(roiStr.c_str(), "size_%ix%i", &(roiSize.width), &(roiSize.height));
        if (roiSize.width != 0 && roiSize.height != 0)
            mRoiSize = roiSize;
        updateDescriptorParams();
    }
    else if (cmd == "blockSize")
    {
        if (pMessage.size() < 2)
            return;

        string blockStr;
        try
        {
            blockStr = atom::toString(pMessage[1]);
        }
        catch (atom::BadTypeTagError error)
        {
            return;
        }

        cv::Size_<int> blockSize;
        sscanf(blockStr.c_str(), "size_%ix%i", &(blockSize.width), &(blockSize.height));
        if (blockSize.width != 0 && blockSize.height != 0)
            mBlockSize = blockSize;
        updateDescriptorParams();
    }
    else if (cmd == "cellSize")
    {
        if (pMessage.size() < 2)
            return;

        string cellStr;
        try
        {
            cellStr = atom::toString(pMessage[1]);
        }
        catch (atom::BadTypeTagError error)
        {
            return;
        }

        cv::Size_<int> cellSize;
        sscanf(cellStr.c_str(), "size_%ix%i", &(cellSize.width), &(cellSize.height));
        if (cellSize.width != 0 && cellSize.height != 0)
            mCellSize = cellSize;
        updateDescriptorParams();
    }
    else if (cmd == "bins")
    {
        if (pMessage.size() < 2)
            return;

        float bins;
        try
        {
            bins = atom::toFloat(pMessage[1]);
        }
        catch (atom::BadTypeTagError error)
        {
            return;
        }

        mBins = max(2.f, bins);
        updateDescriptorParams();
    }
    else
        setBaseParameter(pMessage);
}

/*************/
void Detector_Hog::updateDescriptorParams()
{
    mDescriptor.setHogParams(mRoiSize, mBlockSize, mCellSize, mBins, false, Descriptor_Hog::L2_NORM, mSigma);
}
