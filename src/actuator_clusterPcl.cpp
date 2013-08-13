#include "actuator_clusterPcl.h"

using namespace std;

#if HAVE_PCL

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
    mOscPath = "/blobserver/clusterPcl";

    mFrameNumber = 0;
}

/*************/
atom::Message Actuator_ClusterPcl::detect(vector<Capture_Ptr> pCaptures)
{
    vector< pcl::PointCloud<pcl::PointXYZRGBA> > pointclouds;
    for_each (pCaptures.begin(), pCaptures.end(), [&] (Capture_Ptr capture)
    {
        Capture_3D_PclRgba_Ptr pcl = dynamic_pointer_cast<Capture_3D_PclRgba>(capture);
        if (pcl.get() != NULL)
            pointclouds.push_back(pcl.get());
    });

    return mLastMessage;
}

/*************/
void Actuator_ClusterPcl::setParameter(atom::Message pMessage)
{
    setBaseParameter(pMessage);
}

#endif //HAVE_PCL
