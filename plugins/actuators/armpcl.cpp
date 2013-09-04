#include "armpcl.h"

using namespace std;

#if HAVE_PCL

#include <pcl/common/pca.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>

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

    mNeighboursNbr = 100;
    mMinClusterSize = 50;
    mMaxClusterSize = 25000;
    mClusterTolerance = 0.03;
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
    int mainAxis = 0;
    if (eigenVectors(0, 1) > eigenVectors(0, 0) && eigenVectors(0, 1) > eigenVectors(0, 2))
        mainAxis = 1;
    else if (eigenVectors(0, 2) > eigenVectors(0, 0) && eigenVectors(0, 2) > eigenVectors(0, 1))
        mainAxis = 2;
    
    float maxDistance = 0.f, dist;
    int maxIndex;
    for (int i = 0; i < pcl->size(); ++i)
    {
        pcl::PointXYZRGBA point = pcl->at(i);
        if (mainAxis == 0)
            dist = sqrtf(pow(point.y - mean(1), 2.0) + pow(point.z - mean(2), 2.0));
        else if (mainAxis == 1)
            dist = sqrtf(pow(point.x - mean(0), 2.0) + pow(point.z - mean(2), 2.0));
        else
            dist = sqrtf(pow(point.x - mean(0), 2.0) + pow(point.y - mean(1), 2.0));

        if (dist > maxDistance)
        {
            maxIndex = i;
            maxDistance = dist;
        }
    }

    pcl::search::KdTree<pcl::PointXYZRGBA> tree;
    tree.setInputCloud(pcl);
    vector<int> indices(maxIndex);
    vector<float> squaredDistances(maxIndex);
    tree.nearestKSearch(pcl->at(maxIndex), mNeighboursNbr, indices, squaredDistances);

    pcl::PointXYZ meanPoint;
    meanPoint.x = meanPoint.y = meanPoint.z = 0.f;
    for (int i = 0; i < indices.size(); ++i)
    {
        meanPoint.x += pcl->at(indices[i]).x - mean(0);
        meanPoint.y += pcl->at(indices[i]).y - mean(1);
        meanPoint.z += pcl->at(indices[i]).z - mean(2);
    }
    meanPoint.x /= indices.size();
    meanPoint.y /= indices.size();
    meanPoint.z /= indices.size();

    cv::Mat output = cv::Mat::zeros(mOutputBuffer.size(), CV_8UC3);
    cv::Point2f center(mOutputBuffer.cols / 2, mOutputBuffer.rows / 2);
    cv::Point2f direction(mOutputBuffer.cols / 2 + meanPoint.y * 128, mOutputBuffer.rows / 2 + meanPoint.z * 128);
    cv::line(output, center, direction, cv::Scalar(255), 3);
    mOutputBuffer = output;

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

    if (cmd == "minClusterSize")
    {
        float size;
        if (readParam(pMessage, size))
            mMinClusterSize = max(1, (int)size);
    }
    if (cmd == "maxClusterSize")
    {
        float size;
        if (readParam(pMessage, size))
            mMaxClusterSize = max(1, (int)size);
    }
    if (cmd == "clusterTolerance")
    {
        float tol;
        if (readParam(pMessage, tol))
            mClusterTolerance = max((float)1e-3, tol);
    }

    setBaseParameter(pMessage);
}

#endif //HAVE_PCL
