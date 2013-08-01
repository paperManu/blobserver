#include "source_3d_shmdata.h"

using namespace std;

#if HAVE_PCL && HAVE_SHMDATA

std::string Source_3D_Shmdata::mClassName = "Source_3D_Shmdata";
std::string Source_3D_Shmdata::mDocumentation = "N/A";

/*************/
Source_3D_Shmdata::Source_3D_Shmdata()
{
    make(0);
}

/*************/
Source_3D_Shmdata::Source_3D_Shmdata(int pParam)
{
    make(pParam);
}

/*************/
void Source_3D_Shmdata::make(int pParam)
{
    mName = mClassName;
    mSubsourceNbr = pParam;
}

/*************/
Source_3D_Shmdata::~Source_3D_Shmdata()
{
}

/*************/
bool Source_3D_Shmdata::connect()
{
    return true;
}

/*************/
bool Source_3D_Shmdata::disconnect()
{
    return true;
}

/*************/
bool Source_3D_Shmdata::grabFrame()
{
    return true;
}

/*************/
Capture_Ptr Source_3D_Shmdata::retrieveFrame()
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloud;
    unsigned long long timestamp = mShm->getCloud(pointCloud);

    Capture_3D_PclRgba_Ptr capture(new Capture_3D_PclRgba(pointCloud));
    return capture;
}

/*************/
void Source_3D_Shmdata::setParameter(atom::Message pParam)
{
    string paramName;
    float paramValue;

    try
    {
        paramName = atom::toString(pParam[0]);
    }
    catch (atom::BadTypeTagError exception)
    {
        return;
    }

    if (paramName == "location")
    {
        string location;
        if (!readParam(pParam, location))
            return;

        mShm.reset(new ShmPointCloud<pcl::PointXYZRGBA>(location.c_str(), false));
    }
    else if (paramName == "cameraNumber")
    {
        if (readParam(pParam, paramValue))
            mSubsourceNbr = (unsigned int)paramValue;
    }
}

/*************/
atom::Message Source_3D_Shmdata::getParameter(atom::Message pParam) const
{
    atom::Message msg;

    if (pParam.size() < 1)
        return msg;

    string paramName;
    try
    {
        paramName = atom::toString(pParam[0]);
    }
    catch (atom::BadTypeTagError exception)
    {
        return msg;
    }

    msg.push_back(pParam[0]);
    if (paramName == "framerate")
        msg.push_back(atom::IntValue::create(mFramerate));
    else if (paramName == "subsourcenbr")
        msg.push_back(atom::IntValue::create(mSubsourceNbr));

    return msg;
}

#endif
