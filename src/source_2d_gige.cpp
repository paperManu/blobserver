#include "source_2d_gige.h"

#if HAVE_ARAVIS

using namespace std;

string Source_2D_Gige::mClassName = "Source_2D_Gige";
string Source_2D_Gige::mDocumentation = "N/A";

/*************/
Source_2D_Gige::Source_2D_Gige()
{
    make(0);
}

/*************/
Source_2D_Gige::Source_2D_Gige(int pParam)
{
    make(pParam);
}

/*************/
void Source_2D_Gige::make(int pParam)
{
    mName = mClassName;
    mSubsourceNbr = pParam;

    mCamera = NULL;
    mStream = NULL;

    mInvertRGB = false;
}

/*************/
Source_2D_Gige::~Source_2D_Gige()
{
    disconnect();
}

/*************/
bool Source_2D_Gige::connect()
{
    arv_g_type_init();

    if (mSubsourceNbr == 0)
        mCamera = arv_camera_new(NULL);
    else
        mCamera = arv_camera_new(to_string(mSubsourceNbr).c_str());

    if (mCamera == NULL)
    {
        g_log(NULL, G_LOG_LEVEL_WARNING, "%s - Unable to open subsource %i", mClassName.c_str(), mSubsourceNbr);
        return false;
    }

    // Get current camera parameters
    gint width, height;
    arv_camera_get_region(mCamera, NULL, NULL, &width, &height);
    mWidth = (unsigned int)width;
    mHeight = (unsigned int)height;

    mFramerate = arv_camera_get_frame_rate(mCamera);
    mExposureTime = arv_camera_get_exposure_time(mCamera);
    arv_camera_set_pixel_format(mCamera, ARV_PIXEL_FORMAT_RGB_8_PACKED);

    const char* id = arv_camera_get_device_id(mCamera);
    mId = stoi(string(id));

    // Create the stream object
    allocateStream();

    // Start the acquisition
    arv_camera_set_acquisition_mode(mCamera, ARV_ACQUISITION_MODE_CONTINUOUS);
    arv_camera_start_acquisition(mCamera);

    return true;
}

/*************/
void Source_2D_Gige::allocateStream()
{
    // TODO: find why the following lines output an error...
    //if (mStream != NULL && G_IS_OBJECT(mStream))
    //    g_object_unref(mStream);

    int payload = arv_camera_get_payload(mCamera);
    mStream = arv_camera_create_stream(mCamera, streamCb, this);
    for (int i = 0; i < 50; ++i)
        arv_stream_push_buffer(mStream, arv_buffer_new(payload, NULL));

    if (ARV_PIXEL_FORMAT_BIT_PER_PIXEL(arv_camera_get_pixel_format(mCamera)) == 8)
        mChannels = 1;
    else
        mChannels = 3;
}

/*************/
bool Source_2D_Gige::disconnect()
{
    if (mCamera != NULL)
    {
        arv_camera_stop_acquisition(mCamera);
        g_object_unref(mCamera);
    }

    if (mStream != NULL && G_IS_OBJECT(mStream))
        g_object_unref(mStream);

    return true;
}

/*************/
cv::Mat Source_2D_Gige::retrieveRawFrame()
{
    // If in-camera autoexposure is on, this needs to be done at each frame
    mExposureTime = arv_camera_get_exposure_time(mCamera);
    
    return mBuffer.get().clone();
}

/*************/
void Source_2D_Gige::setParameter(atom::Message pParam)
{
    string paramName;
    float value;

    try
    {
        paramName = atom::toString(pParam[0]);
    }
    catch (atom::BadTypeTagError exception)
    {
        return;
    }

    // Next parameters are all numbers
    if (!readParam(pParam, value))
    {
        return;
    }
    else if (paramName == "raw")
    {
        if (value == 1.f && mChannels == 3)
        {
            arv_camera_stop_acquisition(mCamera);
            arv_camera_set_pixel_format(mCamera, ARV_PIXEL_FORMAT_BAYER_BG_8);
            allocateStream();

            if (ARV_PIXEL_FORMAT_BIT_PER_PIXEL(arv_camera_get_pixel_format(mCamera)) == 8)
                mBayer = true;
            else
                mBayer = false;
            arv_camera_start_acquisition(mCamera);
        }
        else if (value == 0.f && mChannels == 1)
        {
            arv_camera_stop_acquisition(mCamera);
            arv_camera_set_pixel_format(mCamera, ARV_PIXEL_FORMAT_RGB_8_PACKED);
            allocateStream();

            if (ARV_PIXEL_FORMAT_BIT_PER_PIXEL(arv_camera_get_pixel_format(mCamera)) == 8)
                mBayer = true;
            else
                mBayer = false;
            arv_camera_start_acquisition(mCamera);
        }
    }
    else if (paramName == "width")
    {
        int min, max;
        arv_camera_get_width_bounds(mCamera, &min, &max);
        if (value < min || value > max)
            return;

        arv_camera_stop_acquisition(mCamera);
        int x, y, width, height;
        arv_camera_get_region(mCamera, &x, &y, &width, &height);
        arv_camera_set_region(mCamera, x, y, value, height);
        arv_camera_get_region(mCamera, &x, &y, &width, &height);
        mWidth = width;
        mHeight = height;

        allocateStream();
        arv_camera_start_acquisition(mCamera);
    }
    else if (paramName == "height")
    {
        int min, max;
        arv_camera_get_height_bounds(mCamera, &min, &max);
        if (value < min || value > max)
            return;

        arv_camera_stop_acquisition(mCamera);
        int x, y, width, height;
        arv_camera_get_region(mCamera, &x, &y, &width, &height);
        arv_camera_set_region(mCamera, x, y, width, value);
        arv_camera_get_region(mCamera, &x, &y, &width, &height);
        mWidth = width;
        mHeight = height;

        allocateStream();
        arv_camera_start_acquisition(mCamera);
    }
    else if (paramName == "offsetX")
    {
        arv_camera_stop_acquisition(mCamera);
        int x, y, width, height;
        arv_camera_get_region(mCamera, &x, &y, &width, &height);
        arv_camera_set_region(mCamera, value, y, width, height);
        arv_camera_get_region(mCamera, &x, &y, &width, &height);
        mWidth = width;
        mHeight = height;

        allocateStream();
        arv_camera_start_acquisition(mCamera);
    }
    else if (paramName == "offsetY")
    {
        arv_camera_stop_acquisition(mCamera);
        int x, y, width, height;
        arv_camera_get_region(mCamera, &x, &y, &width, &height);
        arv_camera_set_region(mCamera, x, value, width, height);
        arv_camera_get_region(mCamera, &x, &y, &width, &height);
        mWidth = width;
        mHeight = height;

        allocateStream();
        arv_camera_start_acquisition(mCamera);
    }
    else if (paramName == "binning")
    {
        if (value < 1.f)
            return;
        arv_camera_stop_acquisition(mCamera);
        arv_camera_set_binning(mCamera, value, value);
        allocateStream();
        arv_camera_start_acquisition(mCamera);
    }
    else if (paramName == "invertRGB")
    {
        if (value == 1.f)
            mInvertRGB = true;
        else
            mInvertRGB = false;
    }
    else if (paramName == "framerate")
    {
        double min, max;
        arv_camera_get_frame_rate_bounds(mCamera, &min, &max);
        if (value < min || value > max)
        {
            g_log(NULL, G_LOG_LEVEL_WARNING, "%s - Framerate value out of bounds: [%f, %f]", mClassName.c_str(), min, max);
            return;
        }

        arv_camera_set_frame_rate(mCamera, value);
        mFramerate = arv_camera_get_frame_rate(mCamera);
    }
    else if (paramName == "exposureTime")
    {
        double min, max;
        arv_camera_get_exposure_time_bounds(mCamera, &min, &max);
        if (value < min || value > max)
        {
            g_log(NULL, G_LOG_LEVEL_WARNING, "%s - Exposure value out of bounds: [%f, %f]", mClassName.c_str(), min, max);
            return;
        }

        arv_camera_set_exposure_time_auto(mCamera, ARV_AUTO_OFF);
        arv_camera_set_exposure_time(mCamera, value);
        mExposureTime = arv_camera_get_exposure_time(mCamera);
        mExposureParam = value;
    }
    else if (paramName == "gain")
    {
        // If a LUT is set, the gain is set in dB, otherwise it is
        // a value with no specific scale (camera dependant)
        if (mGainLUT.isSet())
        {
            float lutValue = mGainLUT[value];
            if (mGainLUT.isOutOfRange())
            {
                g_log(NULL, G_LOG_LEVEL_WARNING, "%s - Exposure value out of bounds", mClassName.c_str());
                return;
            }

            double min, max;
            arv_camera_get_gain_bounds(mCamera, &min, &max);
            if (lutValue < min || lutValue > max)
            {
                g_log(NULL, G_LOG_LEVEL_WARNING, "%s - Gain value out of bounds: [%f, %f]", mClassName.c_str(), min, max);
                return;
            }

            arv_camera_set_gain_auto(mCamera, ARV_AUTO_OFF);
            arv_camera_set_gain(mCamera, lutValue);
            lutValue = arv_camera_get_gain(mCamera);
            mGain = mGainLUT.inverse(lutValue);
        }
        else
        {
            double min, max;
            arv_camera_get_gain_bounds(mCamera, &min, &max);
            if (value < min || value > max)
            {
                g_log(NULL, G_LOG_LEVEL_WARNING, "%s - Gain value out of bounds: [%f, %f]", mClassName.c_str(), min, max);
                return;
            }

            arv_camera_set_gain_auto(mCamera, ARV_AUTO_OFF);
            arv_camera_set_gain(mCamera, value);
            mGain = arv_camera_get_gain(mCamera);
        }
    }
    else
        setBaseParameter(pParam);
}

/*************/
atom::Message Source_2D_Gige::getParameter(atom::Message pParam) const
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
    if (paramName == "width")
        msg.push_back(atom::IntValue::create(mWidth));
    else if (paramName == "height")
        msg.push_back(atom::IntValue::create(mWidth));
    else if (paramName == "framerate")
        msg.push_back(atom::IntValue::create(mFramerate));
    else if (paramName == "exposureTime")
        msg.push_back(atom::FloatValue::create(mExposureTime));
    else if (paramName == "subsourcenbr")
        msg.push_back(atom::IntValue::create(mSubsourceNbr));
    else
        msg = getBaseParameter(pParam);

    return msg;
}

/*************/
atom::Message Source_2D_Gige::getSubsources() const
{
    atom::Message message;

    arv_update_device_list();
    unsigned int nbrDevices = arv_get_n_devices();

    for (unsigned int i = 0; i < nbrDevices; ++i)
    {
        const char* id = arv_get_device_id(i);
        message.push_back(atom::IntValue::create(i));
        message.push_back(atom::StringValue::create(id));
    }

    return message;
}

/*************/
void Source_2D_Gige::streamCb(void* user_data, ArvStreamCallbackType type, ArvBuffer* buffer)
{
    Source_2D_Gige* source = (Source_2D_Gige*)user_data;

    if (buffer != NULL)
    {
        if (buffer->status == ARV_BUFFER_STATUS_SUCCESS)
        {
            cv::Mat img;
            if (source->mChannels == 3)
                img = cv::Mat::zeros(source->mHeight, source->mWidth, CV_8UC3);
            else
                img = cv::Mat::zeros(source->mHeight, source->mWidth, CV_8U);

            memcpy(img.data, buffer->data, buffer->width * buffer->height * ARV_PIXEL_FORMAT_BIT_PER_PIXEL(buffer->pixel_format) / 8);

            if (source->mInvertRGB && source->mChannels == 3)
            {
                cv::Mat inverted = cv::Mat::zeros(img.size(), img.type());
                cv::cvtColor(img, inverted, CV_BGR2RGB);
                img = inverted;
            }
            else if (source->mBayer && source->mChannels == 1)
            {
                cv::Mat bayer = cv::Mat::zeros(img.size(), CV_8UC3);
                switch (arv_camera_get_pixel_format(source->mCamera))
                {
                case ARV_PIXEL_FORMAT_BAYER_BG_8:
                    cvtColor(img, bayer, CV_BayerBG2RGB);
                    break;
                case ARV_PIXEL_FORMAT_BAYER_GB_8:
                    cvtColor(img, bayer, CV_BayerGB2RGB);
                    break;
                case ARV_PIXEL_FORMAT_BAYER_RG_8:
                    cvtColor(img, bayer, CV_BayerRG2RGB);
                    break;
                case ARV_PIXEL_FORMAT_BAYER_GR_8:
                    cvtColor(img, bayer, CV_BayerGR2RGB);
                    break;
                }
                img = bayer;
            }
            source->mBuffer = img;
            source->mUpdated = true;
        }
        else
        {
            string msg;
            switch (buffer->status)
            {
            case ARV_BUFFER_STATUS_SUCCESS:
                msg = "the buffer is cleared";
                break;
            case ARV_BUFFER_STATUS_TIMEOUT:
                msg = "timeout was reached before all packets are received";
                break;
            case ARV_BUFFER_STATUS_MISSING_PACKETS:
                msg = "stream has missing packets";
                break;
            case ARV_BUFFER_STATUS_WRONG_PACKET_ID:
                msg = "stream has packet with wrong id";
                break;
            case ARV_BUFFER_STATUS_SIZE_MISMATCH:
                msg = "the received image didn't fit in the buffer data space";
                break;
            case ARV_BUFFER_STATUS_FILLING:
                msg = "the image is currently being filled";
                break;
            case ARV_BUFFER_STATUS_ABORTED:
                msg = "the filling was aborted before completion";
                break;
            }
            g_log(NULL, G_LOG_LEVEL_DEBUG, "%s - Error in the received packet: %s", source->mClassName.c_str(), msg.c_str());
        }
        arv_stream_push_buffer(source->mStream, buffer);
    }
    else
    {
        switch (type)
        {
        case ARV_STREAM_CALLBACK_TYPE_INIT:
            g_log(NULL, G_LOG_LEVEL_DEBUG, "%s - Stream callback: thread initialization", source->mClassName.c_str());
            break;
        case ARV_STREAM_CALLBACK_TYPE_EXIT:
            g_log(NULL, G_LOG_LEVEL_DEBUG, "%s - Stream callback: thread end", source->mClassName.c_str());
            break;
        //case ARV_STREAM_CALLBACK_TYPE_START_BUFFER:
        //    g_log(NULL, G_LOG_LEVEL_DEBUG, "%s - Stream callback: buffer filling start", source->mClassName.c_str());
        //    break;
        //case ARV_STREAM_CALLBACK_TYPE_BUFFER_DONE:
        //    g_log(NULL, G_LOG_LEVEL_DEBUG, "%s - Stream callback: buffer filled", source->mClassName.c_str());
        //    break;
        }
    }
}

#endif // HAVE_ARAVIS
