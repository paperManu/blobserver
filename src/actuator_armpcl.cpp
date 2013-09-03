#include "actuator_armpcl.h"

using namespace std;

#if HAVE_PCL

#include <pcl/common/pca.h>
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
    mOscPath = "clusterPcl";

    mFrameNumber = 0;

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

    cv::Mat output = cv::Mat::zeros(mOutputBuffer.size(), CV_8UC3);
    cv::Point2f center(mOutputBuffer.cols / 2, mOutputBuffer.rows / 2);
    cv::Point2f direction(mOutputBuffer.cols / 2 + eigenVectors(2, 1) * 64, mOutputBuffer.rows / 2 + eigenVectors(2, 2) * 64);
    cv::line(output, center, direction, cv::Scalar(255), 3);
    mOutputBuffer = output;
    
    float meanYProjected = 0.f;
    for_each (projection.begin(), projection.end(), [&] (pcl::PointXYZRGBA& point)
    {
        meanYProjected += point.y;
    });
    //meanYProjected /= (float)projection.size();

    mLastMessage.clear();
    mLastMessage.push_back(atom::IntValue::create(4));
    mLastMessage.push_back(atom::IntValue::create(5));
    mLastMessage.push_back(atom::FloatValue::create(meanYProjected));
    mLastMessage.push_back(atom::FloatValue::create(mean(0)));
    mLastMessage.push_back(atom::FloatValue::create(mean(1)));
    mLastMessage.push_back(atom::FloatValue::create(mean(2)));
    mLastMessage.push_back(atom::FloatValue::create(mean(3)));
    for (int i = 0; i < 3; ++i)
    {
        mLastMessage.push_back(atom::IntValue::create(i));
        mLastMessage.push_back(atom::FloatValue::create(eigenVectors(i, 0)));
        mLastMessage.push_back(atom::FloatValue::create(eigenVectors(i, 1)));
        mLastMessage.push_back(atom::FloatValue::create(eigenVectors(i, 2)));
        mLastMessage.push_back(atom::FloatValue::create(eigenValues(i)));
    }

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
