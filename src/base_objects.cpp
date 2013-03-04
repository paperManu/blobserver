#include "base_objects.h"

/*************/
ShmImage::ShmImage(const char* filename, const image_type type)
    : _writer(NULL)
{
    _filename = std::string(filename);
    _type = type;
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
void ShmImage::setImage(const unsigned char* image, const unsigned int width, const unsigned int height, const unsigned int bpp, const unsigned long long timestamp)
{
    if (_width != width || _height != height)
        init(width, height, bpp);

    shmdata_any_writer_push_data(_writer, (void*)image, width*height*bpp/8, 0, NULL, NULL);
}

/*************/
void ShmImage::init(const unsigned int width, const unsigned int height, const unsigned int bpp)
{
    if (_writer != NULL)
        shmdata_any_writer_close(_writer);

    _writer = shmdata_any_writer_init();

    std::string dataType;
    char buffer[256] = "";
    if (_type == rgb)
    {
        sprintf(buffer, "video/x-raw-rgb,bpp=%i,endianness=4321,depth=%i,red_mask=16711680,green_mask=65280,blue_mask=255,width=%i,height=%i,framerate=30/1", bpp, bpp, width, height);
    }
    else if (_type == depth)
    {
        sprintf(buffer, "video/x-raw-gray,bpp=%i,endianness=1234,depth=%i,width=%i,height=%i,framerate=30/1", bpp, bpp, width, height);
    }

    dataType = std::string(buffer);
    shmdata_any_writer_set_data_type(_writer, dataType.c_str());

    if (!shmdata_any_writer_set_path(_writer, _filename.c_str()))
    {
            _width = 0;
            _height = 0;
            std::cout << "**** The file " << _filename.c_str() << " exists, therefore a shmdata cannot be operated with this path." << std::endl;
            shmdata_any_writer_close(_writer);
    }

    _width = width;
    _height = height;

    shmdata_any_writer_start(_writer);
}
