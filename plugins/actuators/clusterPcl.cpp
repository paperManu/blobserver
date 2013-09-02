#include "clusterPcl.h"

using namespace std;

#if HAVE_PCL

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>

std::string Actuator_ClusterPcl::mClassName = "Actuator_ClusterPcl";
std::string Actuator_ClusterPcl::mDocumentation = "N/A";
unsigned int Actuator_ClusterPcl::mSourceNbr = 1;

/*************/
Actuator_ClusterPcl::Actuator_ClusterPcl()
{
    make();
}

/*************/
Actuator_ClusterPcl::Actuator_ClusterPcl(int pParam)
{
    make();
}

/*************/
void Actuator_ClusterPcl::make()
{
    mName = mClassName;
    mOscPath = "clusterPcl";

    mFrameNumber = 0;

    mMinClusterSize = 50;
    mMaxClusterSize = 25000;
    mClusterTolerance = 0.03;
}

/*************/
atom::Message Actuator_ClusterPcl::detect(vector<Capture_Ptr> pCaptures)
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

    if (pcl->points.size() == 0)
    {
        mLastMessage.clear();
        return mLastMessage;
    }

    vector<pcl::PointXYZ> clusterPositions;

    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>());
    tree->setInputCloud(pcl);

    vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
    ec.setClusterTolerance(mClusterTolerance);
    ec.setMinClusterSize(mMinClusterSize);
    ec.setMaxClusterSize(mMaxClusterSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(pcl);
    ec.extract(cluster_indices);

    for_each (cluster_indices.begin(), cluster_indices.end(), [&] (pcl::PointIndices cluster)
    {
        pcl::PointXYZ position;
        position.x = 0.f;
        position.y = 0.f;
        position.z = 0.f;

        for_each (cluster.indices.begin(), cluster.indices.end(), [&] (int index)
        {
            position.x += pcl->points[index].x;
            position.y += pcl->points[index].y;
            position.z += pcl->points[index].z;
        });

        position.x /= (float)cluster.indices.size();
        position.y /= (float)cluster.indices.size();
        position.z /= (float)cluster.indices.size();

        clusterPositions.push_back(position);
    });

    mLastMessage.clear();
    mLastMessage.push_back(atom::IntValue::create(clusterPositions.size()));
    mLastMessage.push_back(atom::IntValue::create(4));
    for (int i = 0; i < clusterPositions.size(); i++)
    {
        mLastMessage.push_back(atom::FloatValue::create(clusterPositions[i].x));
        mLastMessage.push_back(atom::FloatValue::create(clusterPositions[i].y));
        mLastMessage.push_back(atom::FloatValue::create(clusterPositions[i].z));
        mLastMessage.push_back(atom::IntValue::create(i));
    }

    return mLastMessage;
}

/*************/
void Actuator_ClusterPcl::setParameter(atom::Message pMessage)
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
