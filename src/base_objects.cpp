#include "base_objects.h"

using namespace std;

#if HAVE_SHMDATA
/*************/
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
void ShmImage::setImage(cv::Mat& image, const unsigned long long timestamp)
{
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
