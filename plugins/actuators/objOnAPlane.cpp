#include "objOnAPlane.h"

using namespace std;

/*****************/
// Class for parallel remap
/*****************/
class Parallel_Remap : public cv::ParallelLoopBody
{
    public:
        Parallel_Remap(const vector<cv::Mat>* imgs, vector<cv::Mat>* maps, vector<cv::Mat>* output, bool verbose, cv::Size size):
            _imgs(imgs), _maps(maps), _outputs(output), _verbose(verbose), _size(size) {}

        void operator()(const cv::Range& r) const
        {
            const cv::Mat* img = &(*_imgs)[r.start];
            cv::Mat* map = &(*_maps)[r.start + 1];
            cv::Mat* output = &(*_outputs)[r.start];
            for (int idx = r.start; idx < r.end; ++idx, ++img, ++map, ++output)
            {
                cv::Mat buffer;
                cv::remap(*img, buffer, *map, cv::Mat(), cv::INTER_AREA);
                
                cv::resize(buffer, *output, _size, 0, 0, cv::INTER_LINEAR);
                cv::cvtColor(*output, *output, CV_BGR2HSV);
            }
        }

    private:
        const vector<cv::Mat>* _imgs;
        vector<cv::Mat>* _maps;
        vector<cv::Mat>* _outputs;
        bool _verbose;
        cv::Size _size;
};

/*****************/
// Class for parallel comparison
/*****************/
class Parallel_Compare : public cv::ParallelLoopBody
{
    public:
        Parallel_Compare(cv::Mat* master, cv::Mat* capture, cv::Mat* result, float level):
            _master(master), _capture(capture), _result(result), _level(level) {}

        void operator()(const cv::Range& r) const
        {
            cv::Vec3b* master = &(_master->at<cv::Vec3b>(r.start, 0));
            cv::Vec3b* capture = &(_capture->at<cv::Vec3b>(r.start, 0));
            uchar* result = &(_result->at<uchar>(r.start, 0));
            for (int y = r.start; y != r.end; ++y, master += _master->cols, capture += _capture->cols, result += _result->cols)
            {
                for (int x = 0; x < _master->cols; ++x)
                {
                    cv::Vec3b mst = *(master + x);
                    cv::Vec3b cpt = *(capture + x);
                    float dist = sqrtf(pow((float)mst[0]-(float)cpt[0], 2.f)
                        + pow((float)mst[1]-(float)cpt[1], 2.f)
                        + pow((float)mst[2]-(float)cpt[2], 2.f));
                    if (dist > _level)
                        *(result + x) = 255;
                }
            }
        }

    private:
        cv::Mat* _master;
        cv::Mat* _capture;
        cv::Mat* _result;
        float _level;
};

/*****************/
// Definition of Actuator_ObjOnAPlane
/*****************/
std::string Actuator_ObjOnAPlane::mClassName = "Actuator_ObjOnAPlane";
std::string Actuator_ObjOnAPlane::mDocumentation = "N/A";
unsigned int Actuator_ObjOnAPlane::mSourceNbr = 0;

/*****************/
Actuator_ObjOnAPlane::Actuator_ObjOnAPlane()
{
    make();
}

/*****************/
Actuator_ObjOnAPlane::Actuator_ObjOnAPlane(int pParam)
{
    make();
}

/*****************/
void Actuator_ObjOnAPlane::make()
{
    mName = mClassName;
    mOscPath = "objOnAPlane";

    mMaxTrackedBlobs = 16;
    mDetectionLevel = 10.0;
    mFilterSize = 3;
    mMinArea = 32;

    mProcessNoiseCov = 1e-5;
    mMeasurementNoiseCov = 1e-5;
}

/*****************/
atom::Message Actuator_ObjOnAPlane::detect(const vector< Capture_Ptr > pCaptures)
{
    vector<cv::Mat> captures = captureToMat(pCaptures);

    // If the spaces definition changed
    if (mMapsUpdated == false)
        updateMaps(captures);

    // Conversion of captures from their own space to a common space
    // Also, resizes them all to the same size, and converts them to HSV
    std::vector<cv::Mat> correctedCaptures;
    correctedCaptures.resize(captures.size());
    cv::parallel_for_(cv::Range(0, captures.size()), Parallel_Remap(&captures, &mMaps, &correctedCaptures, mVerbose, captures[0].size()));
    if (mVerbose)
    {
        for (int i = 0; i < correctedCaptures.size(); ++i)
        {
            char str[16];
            sprintf(str, "capture %i", i);
            cv::imshow(str, correctedCaptures[i]);
        }
    }

    // Compare the first capture to all the others
    cv::Mat master = correctedCaptures[0];
    cv::Mat detected = cv::Mat::zeros(master.rows, master.cols, CV_8U);

    for_each (correctedCaptures.begin(), correctedCaptures.end(), [&] (cv::Mat capture)
    {
        cv::parallel_for_(cv::Range(0, master.rows), Parallel_Compare(&master, &capture, &detected, mDetectionLevel));
    } );
    
    // Convert the detection to real space
    cv::Mat realDetected;
    cv::remap(detected, realDetected, mMaps[0], cv::Mat(), cv::INTER_LINEAR);

    // Erode and dilate to suppress noise
    cv::Mat lEroded;
    cv::erode(realDetected, lEroded, cv::Mat(), cv::Point(-1, -1), mFilterSize);
    cv::dilate(lEroded, realDetected, cv::Mat(), cv::Point(-1, -1), mFilterSize*2);

    // Apply the mask
    cv::Mat lMask = getMask(realDetected, cv::INTER_NEAREST);
    cv::parallel_for_(cv::Range(0, realDetected.rows), Parallel_Mask<uchar>(&realDetected, &lMask));

    // Detect blobs
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(realDetected, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_TC89_L1);

    std::vector<Blob::properties> lProperties;
    for(int i = 0; i < std::min((int)(contours.size()), mMaxTrackedBlobs); ++i)
    {
        cv::Rect rect = cv::boundingRect(contours[i]);

        Blob::properties properties;
        properties.position.x = rect.x + rect.width / 2;
        properties.position.y = rect.y + rect.height / 2;
        properties.size = cv::contourArea(contours[i], false);
        properties.speed.x = 0.f;
        properties.speed.y = 0.f;

        if (properties.size > mMinArea)
        {
            cv::drawContours(realDetected, contours, i, cv::Scalar(255, 0, 0), 2);
            lProperties.push_back(properties);
        }
    }

    // We want to track them
    trackBlobs<Blob2D>(lProperties, mBlobs);

    // Make sure their covariances are correctly set
    std::vector<Blob2D>::iterator lIt = mBlobs.begin();
    for(; lIt != mBlobs.end(); ++lIt)
    {
        lIt->setParameter("processNoiseCov", mProcessNoiseCov);
        lIt->setParameter("measurementNoiseCov", mMeasurementNoiseCov);
    }

    // And we send and print them
    // Include the number and size of each blob in the message
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
            cv::putText(realDetected, lNbrStr, cv::Point(lX, lY), cv::FONT_HERSHEY_COMPLEX, 0.66, cv::Scalar(128.0, 128.0, 128.0, 128.0));
        }

        // Add this blob to the message
        mLastMessage.push_back(atom::IntValue::create(lId));
        mLastMessage.push_back(atom::IntValue::create(lX));
        mLastMessage.push_back(atom::IntValue::create(lY));
        mLastMessage.push_back(atom::IntValue::create(lSize));
        mLastMessage.push_back(atom::IntValue::create(ldX));
        mLastMessage.push_back(atom::IntValue::create(ldY));
    }

    // Save the result in a buffer
    mOutputBuffer = realDetected;

    return mLastMessage;
}

/*****************/
void Actuator_ObjOnAPlane::setParameter(atom::Message pMessage)
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

    // Add a new space to the vector
    if (cmd == "addSpace")
    {
        int size = 0;
        if (mSpaces.size() != 0)
            size = mSpaces[0].size();

        if (pMessage.size() == size + 1 || size == 0)
        {
            std::vector<cv::Vec2f> space;
            for (int i = 1; i < pMessage.size(); i+=2)
            {
                cv::Vec2f point;
                try
                {
                    point[0] = toFloat(pMessage[i]);
                    point[1] = toFloat(pMessage[i+1]);
                }
                catch (atom::BadTypeTagError error)
                {
                    return;
                }
                space.push_back(point);
            }
            mSpaces.push_back(space);
            mMapsUpdated = false;
        }
    }
    // Replace all the current spaces at once
    else if (cmd == "spaces")
    {
        int number;
        if (!readParam(pMessage, number))
            return;

        int size = (pMessage.size()-2) / number;
        // If all spaces specified have not the same size
        // or size is an odd number (we need 2D points)
        // or not enough points are given for each space (3 minimum)
        if (size*number < pMessage.size()-2 || size % 2 != 0  || size < 6)
            return;

        std::vector<std::vector<cv::Vec2f>> tempSpaces;
        for (int i = 0; i < number; ++i)
        {
            std::vector<cv::Vec2f> space;
            for (int j = 0; j < size; j+=2)
            {
                cv::Vec2f point;
                try
                {
                    point[0] = toFloat(pMessage[i*size+2+j]);
                    point[1] = toFloat(pMessage[i*size+2+j+1]);
                }
                catch (atom::BadTypeTagError error)
                {
                    return;
                }

                space.push_back(point);
            }
            tempSpaces.push_back(space);
        }
        
        // Everything went fine, we can replace the old spaces
        mSpaces.clear();
        for (int i = 0; i < tempSpaces.size(); ++i)
            mSpaces.push_back(tempSpaces[i]);

        mMapsUpdated = false;
    }
    else if (cmd == "clearSpaces")
    {
        mSpaces.clear();
        mMapsUpdated = false;
    }
    else if (cmd == "detectionLevel")
    {
        float value;
        if (readParam(pMessage, value))
            mDetectionLevel = std::max(0.f, value);
    }
    else if (cmd == "processNoiseCov")
    {
        float value;
        if (readParam(pMessage, value))
            mProcessNoiseCov = std::max(0.f, value);
    }
    else if (cmd == "measurementNoiseCov")
    {
        float value;
        if (readParam(pMessage, value))
            mMeasurementNoiseCov = std::max(0.f, value);
    }
    else if (cmd == "filterSize")
    {
        float value;
        if (readParam(pMessage, value))
            mFilterSize = std::max(0.f, value);
    }
    else if (cmd == "minBlobArea")
    {
        float value;
        if (readParam(pMessage, value))
            mMinArea = std::max(0.f, value);
    }
    else if (cmd == "maxTrackedBlobs")
    {
        float value;
        if (readParam(pMessage, value))
            mMaxTrackedBlobs = std::max(0.f, value);
    }
    else
        setBaseParameter(pMessage);
}

/*****************/
void Actuator_ObjOnAPlane::updateMaps(std::vector<cv::Mat> pCaptures)
{
    mMaps.clear();

    // Creating map to convert from uniform space to the real one
    {
        std::vector<cv::Vec2f> space = mSpaces[0];

        cv::Vec2f baseX, baseY, origin;
        int size = space.size();

        baseX[0] = space[1][0] - space[0][0];
        baseX[1] = space[1][1] - space[0][1];
        baseX /= cv::norm(baseX);
        
        // The second vector has to be ortho
        baseY[0] = -baseX[1];
        baseY[1] = baseX[0];

        // Temporary origin
        origin = space[0];
        
        // We have an ortho base, we can use the dot product
        cv::Vec2f max = 0.f;
        std::vector<cv::Vec2f>::iterator iterPoint;
        for (iterPoint = space.begin()+1; iterPoint != space.end(); ++iterPoint)
        {
            cv::Vec2f point = *iterPoint;
            cv::Vec2f vector = point - origin;

            cv::Vec2f coords;
            coords[0] = vector.dot(baseX);
            coords[1] = vector.dot(baseY);

            if (coords[0] < 0.f)
            {
                origin[0] += coords[0];
                coords[0] = 0.f;
            }
            if (coords[1] < 0.f)
            {
                origin[1] += coords[1];
                coords[1] = 0.f;
            }
        }

        std::vector<cv::Vec2f> newCoords;
        newCoords.push_back(origin);
        for (iterPoint = space.begin()+1; iterPoint != space.end(); ++iterPoint)
        {
            cv::Vec2f point = *iterPoint;
            cv::Vec2f vector = point - origin;

            cv::Vec2f coords;
            coords[0] = vector.dot(baseX);
            coords[1] = vector.dot(baseY);

            max[0] = std::max(max[0], coords[0]);
            max[1] = std::max(max[1], coords[1]);

            newCoords.push_back(coords);
        }

        // Create the map
        float ratio = (float)(max[1])/(float)(max[0]);
        cv::Mat map = cv::Mat::zeros(pCaptures[0].rows, pCaptures[0].cols, CV_32FC2); 

        // We now need to projection from the uniform space to real space
        // Calculates the tranformation matrix
        cv::Point2f inPoints[4];
        cv::Point2f outPoints[4];
        for (int i = 0; i < 4; ++i)
        {
            outPoints[i].x = newCoords[i][0] / max[0] * (float)map.cols;
            outPoints[i].y = newCoords[i][1] / max[1] * (float)map.cols*ratio;
        }

        inPoints[0] = cv::Point2f(0.f, 0.f);
        inPoints[1] = cv::Point2f((float)(pCaptures[0].cols), 0.f);
        inPoints[2] = cv::Point2f((float)(pCaptures[0].cols), (float)(pCaptures[0].rows));
        inPoints[3] = cv::Point2f(0.f, (float)(pCaptures[0].rows));

        cv::Mat transformMat = cv::getPerspectiveTransform(inPoints, outPoints);

        // Create the map
        cv::Mat tmpMap = map.clone(); 

        for (int x = 0; x < tmpMap.cols; ++x)
        {
            for (int y = 0; y < tmpMap.rows; ++y)
            {
                cv::Vec2f pos;
                pos[0] = ((float)x / (float)tmpMap.cols) * (float)(pCaptures[0].cols);
                pos[1] = ((float)y / (float)tmpMap.rows) * (float)(pCaptures[0].rows);
                tmpMap.at<cv::Vec2f>(y, x) = pos;
            }
        }

        cv::warpPerspective(tmpMap, map, transformMat, tmpMap.size(), cv::INTER_LINEAR);

        mMaps.push_back(map);
    }

    // Creating maps for the captures
    std::vector<std::vector<cv::Vec2f>>::const_iterator iterSpace;
    std::vector<cv::Mat>::const_iterator iterCapture;
    for (iterSpace = mSpaces.begin()+1, iterCapture = pCaptures.begin();
        iterSpace != mSpaces.end() && iterCapture != pCaptures.end();
        ++iterSpace, ++iterCapture)
    {
        std::vector<cv::Vec2f> space = *iterSpace;
        cv::Mat capture = *iterCapture;

        // Calculates the tranformation matrix
        cv::Point2f inPoints[4];
        cv::Point2f outPoints[4];
        for (int i = 0; i < 4; ++i)
            inPoints[i] = space[i];

        outPoints[0] = cv::Point2f(0.f, 0.f);
        outPoints[1] = cv::Point2f((float)(capture.cols), 0.f);
        outPoints[2] = cv::Point2f((float)(capture.cols), (float)(capture.rows));
        outPoints[3] = cv::Point2f(0.f, (float)(capture.rows));

        cv::Mat transformMat = cv::getPerspectiveTransform(inPoints, outPoints);

        // Create the map
        cv::Mat tmpMap = cv::Mat::zeros(capture.rows, capture.cols, CV_32FC2); 
        cv::Mat map = cv::Mat::zeros(capture.rows, capture.cols, CV_32FC2); 

        for (int x = 0; x < tmpMap.cols; ++x)
        {
            for (int y = 0; y < tmpMap.rows; ++y)
            {
                tmpMap.at<cv::Vec2f>(y, x) = cv::Vec2f(x, y);
            }
        }

        cv::warpPerspective(tmpMap, map, transformMat, tmpMap.size(), cv::INTER_LINEAR);

        mMaps.push_back(map);
    }

    mMapsUpdated = true;
}
