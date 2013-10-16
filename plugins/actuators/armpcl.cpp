#include "armpcl.h"

using namespace std;

#if HAVE_PCL

#include <pcl/common/pca.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>

std::string Actuator_ArmPcl::mClassName = "Actuator_ArmPcl";
std::string Actuator_ArmPcl::mDocumentation = "N/A";
unsigned int Actuator_ArmPcl::mSourceNbr = 1;

/*************/
Actuator_ArmPcl::Actuator_ArmPcl()
{
    make();
}

/*************/
Actuator_ArmPcl::Actuator_ArmPcl(int pParam)
{
    make();
}

/*************/
void Actuator_ArmPcl::make()
{
    mName = mClassName;
    mOscPath = "armPcl";

    mFrameNumber = 0;

    mOutputType = 0;

    mNeighboursNbr = 200;
    mMaxDistanceFromMean = 0.7f;
    mMainAxis = 0;

    mMaxManhattanDistance = 0.1;
    mMinCloudSize = 50;
}

/*************/
atom::Message Actuator_ArmPcl::detect(vector<Capture_Ptr> pCaptures)
{
    vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > pointclouds;
    for_each (pCaptures.begin(), pCaptures.end(), [&] (Capture_Ptr capture)
    {
        Capture_3D_PclRgba_Ptr pcl = dynamic_pointer_cast<Capture_3D_PclRgba>(capture);
        if (pcl.get() != NULL)
            pointclouds.push_back(pcl->get());
    });

    if (pointclouds.size() == 0)
    {
        mLastMessage.clear();
        return mLastMessage;
    }

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl = pointclouds[0];

    if (pcl->points.size() < 3)
    {
        mLastMessage.clear();
        return mLastMessage;
    }

    pcl::PCA<pcl::PointXYZRGBA> pca;
    pca.setInputCloud(pcl);

    Eigen::Vector4f mean = pca.getMean();
    Eigen::Matrix3f eigenVectors = pca.getEigenVectors();
    Eigen::Vector3f eigenValues = pca.getEigenValues();

    pcl::PointCloud<pcl::PointXYZRGBA> projection;
    pca.project(*pcl.get(), projection);

    // We check which axis is the main one from the eigenvectors.
    int mainAxis = mMainAxis;
    if (eigenVectors(0, 1) > eigenVectors(0, 0) && eigenVectors(0, 1) > eigenVectors(0, 2) && mainAxis == -1)
        mainAxis = 1;
    else if (eigenVectors(0, 2) > eigenVectors(0, 0) && eigenVectors(0, 2) > eigenVectors(0, 1) && mainAxis == -1)
        mainAxis = 2;
    else if (mainAxis == -1)
        mainAxis = 0;
    
    float maxDistance = 0.f, dist;
    int maxIndex = 0;
    for (int i = 0; i < pcl->points.size(); ++i)
    {
        pcl::PointXYZRGBA point = pcl->at(i);
        if (mainAxis == 0)
            dist = sqrtf(pow(point.y - mean(1), 2.0) + pow(point.z - mean(2), 2.0));
        else if (mainAxis == 1)
            dist = sqrtf(pow(point.z - mean(2), 2.0) + pow(point.x - mean(0), 2.0));
        else
            dist = sqrtf(pow(point.x - mean(0), 2.0) + pow(point.y - mean(1), 2.0));

        if (dist > maxDistance && dist < mMaxDistanceFromMean)
        {
            maxIndex = i;
            maxDistance = dist;
        }
    }

    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>());
    tree->setInputCloud(pcl);
    vector<int> indices(maxIndex);
    vector<float> squaredDistances(maxIndex);
    tree->nearestKSearch(pcl->at(maxIndex), mNeighboursNbr, indices, squaredDistances);

    // We get rid of points which are too far away from the first one
    pcl::CropBox<pcl::PointXYZRGBA> boxFilter;
    Eigen::Vector4f minPoint, maxPoint;
    minPoint(0) = pcl->at(maxIndex).x - mMaxManhattanDistance;
    minPoint(1) = pcl->at(maxIndex).y - mMaxManhattanDistance;
    minPoint(2) = pcl->at(maxIndex).z - mMaxManhattanDistance;
    maxPoint(0) = pcl->at(maxIndex).x + mMaxManhattanDistance;
    maxPoint(1) = pcl->at(maxIndex).y + mMaxManhattanDistance;
    maxPoint(2) = pcl->at(maxIndex).z + mMaxManhattanDistance;
    boxFilter.setMin(minPoint);
    boxFilter.setMax(maxPoint);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr armCloud(new pcl::PointCloud<pcl::PointXYZRGBA>(*pcl.get(), indices));
    boxFilter.setInputCloud(armCloud);
    pcl::PointCloud<pcl::PointXYZRGBA> armCloudFiltered;
    boxFilter.filter(armCloudFiltered);

    // If the resulting cloud is too small, that means the farthest point was some "noise"
    if (armCloudFiltered.points.size() < mMinCloudSize)
        return mLastMessage;

    pcl::PointXYZ meanPoint;
    meanPoint.x = meanPoint.y = meanPoint.z = 0.f;
    for (int i = 0; i < armCloudFiltered.points.size(); ++i)
    {
        meanPoint.x += armCloudFiltered.at(i).x - mean(0);
        meanPoint.y += armCloudFiltered.at(i).y - mean(1);
        meanPoint.z += armCloudFiltered.at(i).z - mean(2);
    }
    meanPoint.x /= armCloudFiltered.points.size();
    meanPoint.y /= armCloudFiltered.points.size();
    meanPoint.z /= armCloudFiltered.points.size();

    // Draw the directions in a cv::Mat
    if (mOutputType == 0)
    {
        cv::Mat output = cv::Mat::zeros(mOutputBuffer.size(), CV_8UC3);
        cv::Point2f center(mOutputBuffer.cols / 2, mOutputBuffer.rows / 2);
        cv::Point2f direction1(mOutputBuffer.cols / 2 + meanPoint.x * 128, mOutputBuffer.rows / 2 + meanPoint.y * 128);
        cv::Point2f direction2(mOutputBuffer.cols / 2 + meanPoint.y * 128, mOutputBuffer.rows / 2 + meanPoint.z * 128);
        cv::Point2f direction3(mOutputBuffer.cols / 2 + meanPoint.z * 128, mOutputBuffer.rows / 2 + meanPoint.x * 128);
        cv::line(output, center, direction1, cv::Scalar(255, 0, 0), 2);
        cv::line(output, center, direction2, cv::Scalar(0, 255, 0), 2);
        cv::line(output, center, direction3, cv::Scalar(0, 0, 255), 2);
        mOutputBuffer = output;
    }

    // Also, output a cloud which contains the mean and the arm
    if (mOutputType == 1)
    {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr arm(new pcl::PointCloud<pcl::PointXYZRGBA>());
        for (int i = 0; i < indices.size(); ++i)
            arm->points.push_back(pcl->at(indices[i]));
        pcl::PointXYZRGBA centerPoint;
        centerPoint.x = mean(0);
        centerPoint.y = mean(1);
        centerPoint.z = mean(2);
        centerPoint.r = 255;
        centerPoint.g = centerPoint.b = 0;
        arm->points.push_back(centerPoint);

        Capture_3D_PclRgba_Ptr capture(new Capture_3D_PclRgba(arm));
        mCapture = capture;
    }

    // Lastly, create the message
    mLastMessage.clear();
    mLastMessage.push_back(atom::IntValue::create(1));
    mLastMessage.push_back(atom::IntValue::create(4));
    mLastMessage.push_back(atom::IntValue::create(0));
    mLastMessage.push_back(atom::FloatValue::create(meanPoint.x));
    mLastMessage.push_back(atom::FloatValue::create(meanPoint.y));
    mLastMessage.push_back(atom::FloatValue::create(meanPoint.z));

    return mLastMessage;
}

/*************/
void Actuator_ArmPcl::setParameter(atom::Message pMessage)
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

    if (cmd == "outputType")
    {
        float type;
        if (readParam(pMessage, type))
            mOutputType = min(1, max(0, (int)type));
    }
    if (cmd == "neighboursNbr")
    {
        float size;
        if (readParam(pMessage, size))
            mNeighboursNbr = max(1, (int)size);
    }
    if (cmd == "maxDistanceFromMean")
    {
        float dist;
        if (readParam(pMessage, dist))
            mMaxDistanceFromMean = max(0.1f, dist);
    }
    if (cmd == "mainAxis")
    {
        float axis;
        if (readParam(pMessage, axis))
            mMainAxis = min(2, max(-1, (int)axis));
    }
    if (cmd == "maxManhattanDistance")
    {
        float dist;
        if (readParam(pMessage, dist))
            mMaxManhattanDistance = max(0.f, dist);
    }
    if (cmd == "minCloudSize")
    {
        float nbr;
        if (readParam(pMessage, nbr))
            mMinCloudSize = max(0, (int)nbr);
    }

    setBaseParameter(pMessage);
}

/*************/
vector<Capture_Ptr> Actuator_ArmPcl::getOutput() const
{
    vector<Capture_Ptr> outputVec;

    if (mOutputType == 0)
        outputVec.push_back(Capture_2D_Mat_Ptr(new Capture_2D_Mat(mOutputBuffer.clone())));
    else if (mOutputType == 1)
        outputVec.push_back(mCapture);

    return outputVec;
}

#endif //HAVE_PCL
