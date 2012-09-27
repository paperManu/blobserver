#include <iostream>
#include <limits>
#include "glib.h"
#include "opencv2/opencv.hpp"
#include "lo/lo.h"

static gboolean gHide = FALSE;
static gboolean gVerbose = FALSE;

static gint gCamNbr = 0;

static gint gFilterSize = 3;

static GString* gIpAddress = NULL;
static GString* gIpPort = NULL;
static gboolean gTcp = FALSE;

static gboolean gOutliers = FALSE;
static gboolean gLight = FALSE;

static GOptionEntry gEntries[] =
{
    {"hide", 0, 0, G_OPTION_ARG_NONE, &gHide, "Hides the camera output", NULL},
    {"cam", 'c', 0, G_OPTION_ARG_INT, &gCamNbr, "Selects which camera to use", NULL},
    {"filter", 'f', 0, G_OPTION_ARG_INT, &gFilterSize, "Specifies the size of the filtering kernel to use", NULL},
    {"ip", 'i', 0, G_OPTION_ARG_STRING_ARRAY, &gIpAddress, "Specifies the ip address to send messages to", NULL}, 
    {"port", 'p', 0, G_OPTION_ARG_STRING_ARRAY, &gIpPort, "Specifies the port to send messages to", NULL},
    {"tcp", 't', 0, G_OPTION_ARG_NONE, &gTcp, "Use TCP instead of UDP for message transmission", NULL},
    {"verbose", 'v', 0, G_OPTION_ARG_NONE, &gVerbose, "If set, outputs values to the std::out", NULL},
    {"outliers", 0, 0, G_OPTION_ARG_NONE, &gOutliers, "Detects the outliers (luminance wise) and outputs their mean position", NULL},
    {"light", 0, 0, G_OPTION_ARG_NONE, &gLight, "Detects light zone, extract them as blobs and outputs their position and size", NULL},
    {NULL}
};

/********************/
int factorial(int n)
{
    int lValue = 1;
    for(int i=n; i>0; i--)
        lValue *= i;
    return lValue;
}

/************************************/
// Blob class, with tracking features
class Blob
{
    public:
        Blob();
        ~Blob();

        void init(cv::KeyPoint pNewBlob);
        cv::KeyPoint predict();
        void setNewMeasures(cv::KeyPoint pNewBlob);
        
        cv::KeyPoint getBlob();
        bool isUpdated();

    private:
        bool updated;
        cv::KeyPoint mBlob;
        cv::KalmanFilter mFilter;
};

/*************/
Blob::Blob()
{
    updated = false;

    mBlob.pt.x = 0.0;
    mBlob.pt.y = 0.0;
    mBlob.size = 0.0;

    // We are filtering a 3 variables state, and
    // we have a measure for all of them
    mFilter.init(3, 3);

    mFilter.transitionMatrix = *(cv::Mat_<float>(3, 3) << 1,0,0, 0,1,0, 0,0,1);
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
void Blob::init(cv::KeyPoint pNewBlob)
{
    mFilter.statePre.at<float>(0) = pNewBlob.pt.x;
    mFilter.statePre.at<float>(1) = pNewBlob.pt.y;
    mFilter.statePre.at<float>(2) = pNewBlob.size;
}

/*************/
cv::KeyPoint Blob::predict()
{
    cv::Mat lPrediction;
    cv::KeyPoint lKeyPoint;

    lPrediction = mFilter.predict();
    lKeyPoint.pt.x = lPrediction.at<float>(0);
    lKeyPoint.pt.y = lPrediction.at<float>(1);
    lKeyPoint.size = lPrediction.at<float>(2);

    updated = false;
    
    return lKeyPoint;
}

/*************/
void Blob::setNewMeasures(cv::KeyPoint pNewBlob)
{
    cv::Mat lMeasures = cv::Mat::zeros(3, 1, CV_32F);
    lMeasures.at<float>(0) = pNewBlob.pt.x;
    lMeasures.at<float>(1) = pNewBlob.pt.y;
    lMeasures.at<float>(2) = pNewBlob.size;

    cv::Mat lEstimation = mFilter.correct(lMeasures);
    mBlob.pt.x = lEstimation.at<float>(0);
    mBlob.pt.y = lEstimation.at<float>(1);
    mBlob.size = lEstimation.at<float>(2);

    updated = true;
}

/*************/
cv::KeyPoint Blob::getBlob()
{
    return mBlob;
}

/*************/
bool Blob::isUpdated()
{
    return updated;
}

/*****************************/
// Definition of the app class
class App
{
    public:
        // Constructor and destructor
        App();
        ~App();

        // Arguments parser
        int parseArgs(int argc, char **argv);

        // Initialization, depending on arguments
        int init();

        // Main loop
        int loop();

    private:
        // Attributes
        // liblo related
        lo_address mOscAddress;

        // opencv related
        cv::VideoCapture mCamera;
        cv::Mat mCameraBuffer;

        cv::SimpleBlobDetector* mLightBlobDetector; // OpenCV object which detects the blobs in an image
        std::vector<Blob> mLightBlobs; // Vector of detected and tracked blobs

        // Methods
        // Various filter and detection modes availables
        
        // Detects outliers based on the mean and std dev, and
        // outputs their mean position and total size
        cv::Mat detectMeanOutliers();

        // Detects light spots, and outputs each one of their position
        // and size
        cv::Mat detectLightSpots();
       
        // This function returns the configuration (element from x linked to element from y)
        // which gives the lowest sum, according the the pDistances. Returns a matrix with the
        // same dimensions as pDistances, filled with 0 and 255
        cv::Mat getLeastSumConfiguration(cv::Mat* pDistances);
        // This function is called by the previous one, and should not be called by itself
        cv::Mat getLeastSumForLevel(cv::Mat pConfig, cv::Mat* pDistances, int pLevel, cv::Mat pAttributed, float &pSum);
};


/*****************/
App::App()
    :mLightBlobDetector(NULL)
{
}


/*****************/
App::~App()
{
}

/*****************/
int App::parseArgs(int argc, char** argv)
{
    GError *error = NULL;
    GOptionContext* context;

    context = g_option_context_new("- blobserver, sends blobs through OSC");
    g_option_context_add_main_entries(context, gEntries, NULL);

    if(!g_option_context_parse(context, &argc, &argv, &error))
    {
        std::cout << "Error while parsing options: " << error->message << std::endl;
        return 1;
    }

    return 0;
}

/*****************/
int App::init()
{
    // Initialize camera
    mCamera.open(gCamNbr);
    if(!mCamera.isOpened())
    {
        std::cout << "Error while opening camera number " << gCamNbr << ". Exiting." << std::endl;
        return 1;
    }
    // Get a first frame to initialize the buffer
    mCamera.read(mCameraBuffer);

    // Initialize OSC
    if(gIpAddress != NULL)
    {
        std::cout << "IP specified: " << gIpAddress->str << std::endl;
    }
    else
    {
        gIpAddress = g_string_new("127.0.0.1");
        std::cout << "No IP specified, using localhost" << std::endl;
    }

    if(gIpPort != NULL)
    {
        std::cout << "Using port number " << gIpPort->str << std::endl;
    }
    else
    {
        gIpPort = g_string_new("9000");
        std::cout << "No port specified, using 9000" << std::endl;
    }

    if(gTcp)
        mOscAddress = lo_address_new_with_proto(LO_TCP, gIpAddress->str, gIpPort->str);
    else
        mOscAddress = lo_address_new_with_proto(LO_UDP, gIpAddress->str, gIpPort->str);

    // Initialize a few other things
    // Set mLightDetector to indeed detect light
    cv::SimpleBlobDetector::Params lParams;
    lParams.filterByColor = true;
    lParams.blobColor = 255;
    mLightBlobDetector = new cv::SimpleBlobDetector(lParams);
    
    return 0;
}

/*****************/
int App::loop()
{
    bool lShowCamera = !gHide;
    int lSourceNumber = 0;

    bool loop = true;
    while(loop)
    {
        int lTotalBuffers = 1; // starting at 1, because there is at least the camera buffer
        std::vector<cv::Mat> lBuffers;
        std::vector<std::string> lBufferNames;

        // Frame capture
        mCamera.read(mCameraBuffer);
        // cv::Mat are not copied when not specified, so the bandwidth usage
        // of the following operation is minimal
        lBuffers.push_back(mCameraBuffer);
        lBufferNames.push_back(std::string("camera"));

        // If the frame seems valid
        if(mCameraBuffer.size[0] > 0 && mCameraBuffer.size[0] > 0)
        {
            // Camera capture
            if(lShowCamera)
                cv::imshow("blobserver", mCameraBuffer);

            // Informations extraction
            if(gOutliers)
            {
                lBuffers.push_back(detectMeanOutliers());
                lBufferNames.push_back(std::string("mean outliers"));
                lTotalBuffers += 1;
            }

            if(gLight)
            {
                lBuffers.push_back(detectLightSpots());
                lBufferNames.push_back(std::string("light blobs"));
                lTotalBuffers += 1;
            }

            if(lShowCamera)
            {
                cv::putText(lBuffers[lSourceNumber], lBufferNames[lSourceNumber].c_str(), cv::Point(10, 30),
                    cv::FONT_HERSHEY_COMPLEX, 1.0, cv::Scalar::all(255.0));
                cv::imshow("blobserver", lBuffers[lSourceNumber]);
            }
        }

        char lKey = cv::waitKey(5);
        if(lKey == 'q')
            loop = false;
        if(lKey == 'w')
        {
            lSourceNumber = (lSourceNumber+1)%lTotalBuffers;
            std::cout << "Buffer displayed: " << lBufferNames[lSourceNumber] << std::endl;
        }
    }

    return 0;
}

/*****************/
cv::Mat App::detectMeanOutliers()
{
    cv::Mat lMean, lStdDev;
    cv::Mat lOutlier, lEroded, lFiltered;

    // Eliminate the outliers : calculate the mean and std dev
    lOutlier = cv::Mat::zeros(mCameraBuffer.size[0], mCameraBuffer.size[1], CV_8U);
    lEroded = lOutlier.clone();
    lFiltered = lOutlier.clone();
    cv::cvtColor(mCameraBuffer, lOutlier, CV_RGB2GRAY);

    cv::meanStdDev(mCameraBuffer, lMean, lStdDev);
    cv::absdiff(lOutlier, lMean.at<double>(0), lOutlier);

    // Detect pixels far from the mean (> 2*stddev)
    cv::threshold(lOutlier, lOutlier, 2*lStdDev.at<double>(0), 255, cv::THRESH_BINARY);

    // Erode and dilate to suppress noise
    cv::erode(lOutlier, lEroded, cv::Mat(), cv::Point(-1, -1), gFilterSize);
    cv::dilate(lEroded, lFiltered, cv::Mat(), cv::Point(-1, -1), gFilterSize);

    // Calculate the barycenter of the outliers
    int lNumber = 0;
    int lX=0, lY=0;

    for(int x=0; x<lFiltered.size[1]; x++)
        for(int y=0; y<lFiltered.size[0]; y++)
        {
            if(lFiltered.at<uchar>(y, x) == 255)
            {
                lX += x;
                lY += y;
                lNumber++;
            }
        }

    if(lNumber > 0)
    {
        lX /= lNumber;
        lY /= lNumber;
    }
    else
    {
        lX = lFiltered.size[1] / 2;
        lY = lFiltered.size[0] / 2;
    }

    if(gVerbose)
    {
        std::cout << "--- Outliers detection:" << std::endl;
        std::cout << "x: " << lX << " - y: " << lY << " - size: " << lNumber << std::endl;
    }

    // Send the result
    lo_send(mOscAddress, "/blobserver/meanOutliers/", "iii", lX, lY, lNumber);

    return lFiltered;
}

/*****************/
cv::Mat App::detectLightSpots()
{
    cv::Mat lMean, lStdDev;
    cv::Mat lOutlier, lLight;
    cv::Mat lEroded;
    std::vector<cv::KeyPoint> lKeyPoints;

    // Eliminate the outliers : calculate the mean and std dev
    lOutlier = cv::Mat::zeros(mCameraBuffer.size[0], mCameraBuffer.size[1], CV_8U);
    lLight = lOutlier.clone();
    cv::cvtColor(mCameraBuffer, lOutlier, CV_RGB2GRAY);

    cv::meanStdDev(mCameraBuffer, lMean, lStdDev);
    cv::absdiff(lOutlier, lMean.at<double>(0), lOutlier);

    // Detect pixels which values are superior to the mean
    cv::threshold(lOutlier, lLight, lMean.at<double>(0), 255, cv::THRESH_BINARY);
    // Detect pixels far from the mean (> 2*stddev)
    cv::threshold(lOutlier, lOutlier, 2*lStdDev.at<double>(0), 255, cv::THRESH_BINARY);
    // Combinaison of both previous conditions
    cv::bitwise_and(lOutlier, lLight, lLight);
    // Erode and dilate to suppress noise
    cv::erode(lLight, lEroded, cv::Mat(), cv::Point(-1, -1), gFilterSize);
    cv::dilate(lEroded, lLight, cv::Mat(), cv::Point(-1, -1), gFilterSize);

    // Now we have to detect blobs
    mLightBlobDetector->detect(lLight, lKeyPoints);

    // We want to track them
    // First we update all the previous blobs we detected,
    // and keep their predicted new position
    for(int i=0; i<mLightBlobs.size(); ++i)
        mLightBlobs[i].predict();
    
    // Then we compare all these prediction with real measures and
    // associate them together
    cv::Mat lConfiguration;
    std::cout << "--- Number of blobs: " << mLightBlobs.size() << std::endl;
    std::cout << "--- Number of key points: " << lKeyPoints.size() << std::endl;
    if(mLightBlobs.size() != 0)
    {
        cv::Mat lTrackMat = cv::Mat::zeros(lKeyPoints.size(), mLightBlobs.size(), CV_32F);

        // Compute the squared distance between all new blobs, and all tracked ones
        for(int i=0; i<lKeyPoints.size(); ++i)
        {
            for(int j=0; j<mLightBlobs.size(); ++j)
            {
                cv::KeyPoint keyPoint = lKeyPoints[i];
                cv::KeyPoint blob = mLightBlobs[i].getBlob();

                float lDistance = pow(keyPoint.pt.x - blob.pt.x, 2.0)
                    + pow(keyPoint.pt.y - blob.pt.y, 2.0)
                    + pow(keyPoint.size - blob.size, 2.0);

                lTrackMat.at<float>(i, j) = lDistance;
            }
        }

        // We associate each tracked blobs with the fittest blob, using a least square approach
        lConfiguration = getLeastSumConfiguration(&lTrackMat);
    }

    cv::Mat lAttributedKeypoints = cv::Mat::zeros(lKeyPoints.size(), 1, CV_8U);
    std::vector<Blob>::iterator lBlob = mLightBlobs.begin();
    for(int i=0; i<lConfiguration.rows; ++i)
    {
        int lIndex = lConfiguration.at<uchar>(i);
        // We update the blobs which we were able to track
        if(lIndex < 255)
        {
            lBlob->setNewMeasures(lKeyPoints[lIndex]);
            lBlob++;
            lAttributedKeypoints.at<uchar>(i) = 255;
        }
    }
    // We delete the blobs we couldn't track
    //for(lBlob = mLightBlobs.begin(); lBlob != mLightBlobs.end(); lBlob++)
    for(int i=0; i<mLightBlobs.size(); i++)
    {
        if(!mLightBlobs[i].isUpdated())
        {
            mLightBlobs.erase(mLightBlobs.begin()+i);
            i--;
        }
    }
    // And we create new blobs for the new objects detected
    for(int i=0; i<lAttributedKeypoints.rows; ++i)
    {
        int lIndex = lAttributedKeypoints.at<uchar>(i);
        if(lIndex == 0)
        {
            Blob lNewBlob;
            lNewBlob.init(lKeyPoints[lIndex]);
            mLightBlobs.push_back(lNewBlob);
        }
    }

    if(gVerbose)
        std::cout << "--- Light blobs detection:" << std::endl;

    // And we send and print them
    for(int i=0; i<lKeyPoints.size(); ++i)
    {
        int lX, lY, lSize;
        lX = (int)(lKeyPoints[i].pt.x);
        lY = (int)(lKeyPoints[i].pt.y);
        lSize = (int)(lKeyPoints[i].size);

        if(gVerbose)
            std::cout << "Blob #" << i << " - x=" << lX << " - y=" << lY << " - size=" << lSize << std::endl;

        // Send the result through OSC
        lo_send(mOscAddress, "/blobserver/lightSpots/", "iiii", i, lX, lY, lSize);
    }

    // DEBUG
    for(int i=0; i<mLightBlobs.size(); ++i)
    {
        int lX, lY, lS;
        cv::KeyPoint keyPoint = mLightBlobs[i].getBlob();
        lX = (int)(keyPoint.pt.x);
        lY = (int)(keyPoint.pt.y);
        lS = (int)(keyPoint.size);
        
        std::cout << "Blob #" << i << " - x=" << lX << " - y=" << lY << " - size=" << lS << std::endl;
    }

    return lLight;
}

/*****************/
cv::Mat App::getLeastSumConfiguration(cv::Mat* pDistances)
{
    float lMinSum = 0.f;
    cv::Mat lConfiguration = cv::Mat::ones(mLightBlobs.size(), 1, CV_8U)*255;
    cv::Mat lAttributed = cv::Mat::zeros(pDistances->rows, 1, CV_8U);

    lConfiguration = getLeastSumForLevel(lConfiguration, pDistances, 0, lAttributed, lMinSum);

    return lConfiguration;
}

/*****************/
cv::Mat App::getLeastSumForLevel(cv::Mat pConfig, cv::Mat* pDistances, int pLevel, cv::Mat pAttributed, float &pSum)
{
    float lMinSum = std::numeric_limits<float>::max();
    cv::Mat lAttributed;
    cv::Mat lConfig;

    for(int i=0; i<pAttributed.rows; i++)
    {
        if(pAttributed.at<uchar>(i, 0) == 0)
        {
            lAttributed = pAttributed.clone();
            lAttributed.at<uchar>(i, 0) = 255;
            
            float lCurrentSum = pSum + pDistances->at<float>(i, pLevel);
            
            cv::Mat lCurrentConfig = pConfig.clone();
            lCurrentConfig.at<uchar>(pLevel) = i;

            if(pLevel < pDistances->cols-1)
                lCurrentConfig = getLeastSumForLevel(pConfig, pDistances, pLevel+1, lAttributed, lCurrentSum);

            if(lCurrentSum < lMinSum)
            {
                lMinSum = lCurrentSum;
                lConfig = lCurrentConfig;
            }
        }
    }

    pSum = lMinSum;
    return lConfig;
}

/*****************/
int main(int argc, char** argv)
{
    App theApp;
    int ret;

    ret = theApp.parseArgs(argc, argv);
    if(ret != 0)
        return ret;

    ret = theApp.init();
    if(ret != 0)
        return ret;

    ret = theApp.loop();
    return ret;
}
