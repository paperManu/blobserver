#include "base_objects.h"

#include <algorithm>
#include <chrono>
#include <limits>

using namespace std;

/*************/
// LookupTable
LookupTable::LookupTable()
{
    mIsSet = false;

    mStart[0] = numeric_limits<float>::max();
    mEnd[0] = numeric_limits<float>::min();

    mOutOfRange = true;
}

/*************/
LookupTable::LookupTable(interpolation inter, vector< vector<float> > keys)
{
    set(inter, keys);
}

/*************/
void LookupTable::set(interpolation inter, vector< vector<float> > keys)
{
    mInterpolation = inter; 
    mKeys = keys;
    sort(mKeys.begin(), mKeys.end(), [] (vector<float> a, vector<float> b)
    {
        return a[0] < b[0];
    });

    mStart[0] = (*(mKeys.begin()))[0];
    mStart[1] = (*(mKeys.begin()))[1];
    mEnd[0] = (*(mKeys.end() - 1))[0];
    mEnd[1] = (*(mKeys.end() - 1))[1];

    mOutOfRange = true;
    mIsSet = true;
}

/*************/
float LookupTable::operator[](const float& value)
{
    mOutOfRange = true;

    if (mInterpolation == interpolation::linear)
    {
        if (value < mStart[0])
            return value;
        if (value > mEnd[0])
            return value;

        mOutOfRange = false;

        vector<float> a;
        a.push_back(value);
        auto upper = upper_bound(mKeys.begin(), mKeys.end(), a, [] (vector<float> a, vector<float> b)
        {
            return a[0] < b[0];
        });
        auto lower = upper - 1;

        float ratio = (a[0] - (*lower)[0]) / ((*upper)[0] - (*lower)[0]);
        float result = (*lower)[1] + ratio * ((*upper)[1] - (*lower)[1]);
        return result;
    }
}

/*************/
// ShmAuto
ShmAuto::ShmAuto(const char* filename)
{
    mFilename = filename;
}

/*************/
void ShmAuto::setCapture(Capture_Ptr& capture, const unsigned long long timestamp)
{
    if (dynamic_pointer_cast<Capture_2D_Mat>(capture).get() != NULL)
    {
        if (dynamic_pointer_cast<ShmImage>(mShm).get() == NULL)
            mShm.reset(new ShmImage(mFilename.c_str()));
        mShm->setCapture(capture);
    }
#if HAVE_PCL
    else if (dynamic_pointer_cast<Capture_3D_PclRgba>(capture).get() != NULL)
    {
        if (dynamic_pointer_cast<ShmPcl>(mShm).get() == NULL)
            mShm.reset(new ShmPcl(mFilename.c_str()));
        mShm->setCapture(capture);
    }
#endif // HAVE_PCL
}

#if HAVE_SHMDATA
/*************/
// ShmImage
ShmImage::ShmImage(const char* filename):
    _writer(NULL),
    _startTime(0)
{
    _filename = std::string(filename);
    _width = 0;
    _height = 0;
}

/*************/
ShmImage::~ShmImage()
{
    if (_writer != NULL)
        shmdata_any_writer_close(_writer);
}

/*************/
void ShmImage::setCapture(Capture_Ptr& capture, const unsigned long long timestamp)
{
    Capture_2D_Mat_Ptr capture2D = dynamic_pointer_cast<Capture_2D_Mat>(capture);
    if (capture2D.get() == NULL)
    {
        g_log(NULL, G_LOG_LEVEL_WARNING, "ShmImage: Wrong type of Capture received");
        return;
    }
    cv::Mat image = capture2D->get();

    if (_width != image.cols || _height != image.rows || _type != image.type())
        if (!init(image.cols, image.rows, image.type()))
            return;

    // Get the current timestamp
    unsigned long long currentTime;
    if (timestamp == 0)
    {
        auto now = chrono::high_resolution_clock::now();
        currentTime = chrono::duration_cast<chrono::milliseconds>(now.time_since_epoch()).count();
    }
    else
    {
        currentTime = timestamp;
    }

    if (_startTime == 0)
      _startTime = currentTime;

    shmdata_any_writer_push_data(_writer, (void*)(image.data), _width*_height*_bpp/8, (currentTime - _startTime) * 1e6, NULL, NULL);
}

/*************/
bool ShmImage::init(const unsigned int width, const unsigned int height, int type)
{
    if (_writer != NULL)
        shmdata_any_writer_close(_writer);

    _writer = shmdata_any_writer_init();

    std::string dataType;
    char buffer[256] = "";
    if (type == CV_8UC3)
    {
        sprintf(buffer, "video/x-raw-rgb,bpp=%i,endianness=4321,depth=%i,red_mask=255,green_mask=65280,blue_mask=16711680,width=%i,height=%i,framerate=30/1", 24, 24, width, height);
        _bpp = 24;
    }
    else if (type == CV_8UC4)
    {
        sprintf(buffer, "video/x-raw-rgb,bpp=%i,endianness=4321,depth=%i,red_mask=255,green_mask=65280,blue_mask=16711680,width=%i,height=%i,framerate=30/1", 32, 32, width, height);
        _bpp = 32;
    }
    else if (type == CV_8U)
    {
        sprintf(buffer, "video/x-raw-gray,bpp=%i,endianness=1234,depth=%i,width=%i,height=%i,framerate=30/1", 8, 8, width, height);
        _bpp = 8;
    }
    else if (type == CV_16U)
    {
        sprintf(buffer, "video/x-raw-gray,bpp=%i,endianness=1234,depth=%i,width=%i,height=%i,framerate=30/1", 16, 16, width, height);
        _bpp = 16;
    }
    else
    {
        return false;
    }

    dataType = std::string(buffer);
    shmdata_any_writer_set_data_type(_writer, dataType.c_str());

    if (!shmdata_any_writer_set_path(_writer, _filename.c_str()))
    {
            _width = 0;
            _height = 0;
            _bpp = 0;
            std::cout << "**** The file " << _filename.c_str() << " exists, therefore a shmdata cannot be operated with this path." << std::endl;
            shmdata_any_writer_close(_writer);
            return true;
    }

    _width = width;
    _height = height;
    _type = type;

    shmdata_any_writer_start(_writer);

    return true;
}
#endif // HAVE_SHMDATA

#if HAVE_PCL
/*************/
ShmPcl::ShmPcl(const char* filename)
{
    _writer.reset(new ShmPointCloud<pcl::PointXYZRGBA>(filename, true));
}

/*************/
void ShmPcl::setCapture(Capture_Ptr& capture, const unsigned long long timestamp)
{
    Capture_3D_PclRgba_Ptr capture3d = dynamic_pointer_cast<Capture_3D_PclRgba>(capture);
    _writer->setCloud(capture3d->get(), false, timestamp);
}
#endif //HAVE_PCL
