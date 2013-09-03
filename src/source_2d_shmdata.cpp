#include "source_2d_shmdata.h"

using namespace std;

#if HAVE_SHMDATA
#include <regex>

string Source_2D_Shmdata::mClassName = "Source_2D_Shmdata";
string Source_2D_Shmdata::mDocumentation = "N/A";

/*************/
Source_2D_Shmdata::Source_2D_Shmdata()
{
    make(0);
}

/*************/
Source_2D_Shmdata::Source_2D_Shmdata(int pParam)
{
    make(pParam);
}

/*************/
void Source_2D_Shmdata::make(int pParam)
{
    mReader = NULL;

    mName = mClassName;
    mSubsourceNbr = pParam;
    mId = pParam;
}

/*************/
Source_2D_Shmdata::~Source_2D_Shmdata()
{
    disconnect();
}

/*************/
bool Source_2D_Shmdata::connect()
{
    return true;    
}

/*************/
bool Source_2D_Shmdata::disconnect()
{
    if (mReader != NULL)
        shmdata_any_reader_close(mReader);

    return true;
}

/*************/
bool Source_2D_Shmdata::grabFrame()
{
    return true;
}

/*************/
cv::Mat Source_2D_Shmdata::retrieveRawFrame()
{
    lock_guard<mutex> lock(mMutex);
    return mBuffer.get();
}

/*************/
void Source_2D_Shmdata::setParameter(atom::Message pParam)
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

        if (mReader != NULL)
            shmdata_any_reader_close(mReader);

        mReader = shmdata_any_reader_init();
        shmdata_any_reader_run_gmainloop(mReader, SHMDATA_FALSE);
        shmdata_any_reader_set_on_data_handler(mReader, Source_2D_Shmdata::onData, this);
        //shmdata_any_reader_set_debug(mReader, SHMDATA_TRUE);
        shmdata_any_reader_start(mReader, location.c_str());

        g_log(NULL, G_LOG_LEVEL_INFO, "%s: Connected to shmdata %s", mClassName.c_str(), location.c_str());
    }
    else if (paramName == "cameraNumber")
    {
        if (readParam(pParam, paramValue))
            mSubsourceNbr = (unsigned int)paramValue;
    }
    else
        setBaseParameter(pParam);
}

/*************/
atom::Message Source_2D_Shmdata::getParameter(atom::Message pParam) const
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
    else if (paramName == "subsourcenbr")
        msg.push_back(atom::IntValue::create(mSubsourceNbr));
    else
        msg = getBaseParameter(pParam);

    return msg;
}

/*************/
atom::Message Source_2D_Shmdata::getSubsources() const
{
    atom::Message message;

    return message;
}

/*************/
void Source_2D_Shmdata::onData(shmdata_any_reader_t* reader, void* shmbuf, void* data, int data_size, unsigned long long timestamp,
    const char* type_description, void* user_data)
{
    Source_2D_Shmdata* context = static_cast<Source_2D_Shmdata*>(user_data);
    lock_guard<mutex> lock(context->mMutex);

    string dataType(type_description);
    regex regRgb, regGray, regYUV, regBpp, regWidth, regHeight, regRed, regBlue, regFormatYUV;
    try
    {
        // GCC 4.6 does not support the full regular expression. Some work around is needed,
        // this is why this may seem complicated for nothing ...
        regRgb = regex("(video/x-raw-rgb)(.*)", regex_constants::extended);
        regGray = regex("(video/x-raw-gray)(.*)", regex_constants::extended);
        regYUV = regex("(video/x-raw-yuv)(.*)", regex_constants::extended);
        regFormatYUV = regex("(.*format=\\(fourcc\\))(.*)", regex_constants::extended);
        regBpp = regex("(.*bpp=\\(int\\))(.*)", regex_constants::extended);
        regWidth = regex("(.*width=\\(int\\))(.*)", regex_constants::extended);
        regHeight = regex("(.*height=\\(int\\))(.*)", regex_constants::extended);
        regRed = regex("(.*red_mask=\\(int\\))(.*)", regex_constants::extended);
        regBlue = regex("(.*blue_mask=\\(int\\))(.*)", regex_constants::extended);
    }
    catch (const regex_error& e)
    {
        g_log(NULL, G_LOG_LEVEL_WARNING, "%s - Regex error code: %i", mClassName.c_str(), e.code());
        return;
    }

    if (regex_match(dataType, regRgb) || regex_match(dataType, regGray) || regex_match(dataType, regYUV))
    {
        int bpp, width, height, red, green, blue, channels;
        bool isGray = false;
        bool isYUV = false;
        bool is420 = false;

        smatch match;
        string substr, format;

        if (regex_match(dataType, match, regBpp))
        {
            ssub_match subMatch = match[2];
            substr = subMatch.str();
            sscanf(substr.c_str(), ")%i", &bpp);
        }
        if (regex_match(dataType, match, regWidth))
        {
            ssub_match subMatch = match[2];
            substr = subMatch.str();
            sscanf(substr.c_str(), ")%i", &width);
        }
        if (regex_match(dataType, match, regHeight))
        {
            ssub_match subMatch = match[2];
            substr = subMatch.str();
            sscanf(substr.c_str(), ")%i", &height);
        }
        if (regex_match(dataType, match, regRed))
        {
            ssub_match subMatch = match[2];
            substr = subMatch.str();
            sscanf(substr.c_str(), ")%i", &red);
        }
        else
        {
            if (regex_match(dataType, regYUV))
                isYUV = true;
            else
                isGray = true;
        }
        if (regex_match(dataType, match, regBlue))
        {
            ssub_match subMatch = match[2];
            substr = subMatch.str();
            sscanf(substr.c_str(), ")%i", &blue);
        }

        if (isGray)
            channels = 1;
        else if (bpp == 24)
            channels = 3;
        else if (bpp == 32)
            channels = 4;
        else if (isYUV)
        {
            bpp = 16;
            channels = 3;

            if (regex_match(dataType, match, regFormatYUV))
            {
                char format[16];
                ssub_match subMatch = match[2];
                substr = subMatch.str();
                sscanf(substr.c_str(), ")%s", format);
                if (strstr(format, (char*)"I420") != NULL)
                    is420 = true;
            }
        }

        if (width == 0 || height == 0 || bpp == 0)
            return;

        cv::Mat buffer;
        context->mWidth = width;
        context->mHeight = height;
        context->mChannels = channels;

        if (channels == 1 && bpp == 8)
            buffer = cv::Mat::zeros(height, width, CV_8U);
        else if (channels == 1 && bpp == 16)
            buffer = cv::Mat::zeros(height, width, CV_16U);
        else if (channels == 2)
            buffer = cv::Mat::zeros(height, width, CV_8UC2);
        else if (channels == 3)
            buffer = cv::Mat::zeros(height, width, CV_8UC3);
        else if (channels == 4)
            buffer = cv::Mat::zeros(height, width, CV_8UC4);

        if (isYUV && !is420)
        {
            buffer = cv::Mat::zeros(height, width, CV_8UC2);
            memcpy((char*)(buffer.data), (const char*)data, width*height*bpp/8);
        }
        else if (isYUV && is420)
        {
            cv::Mat buffer_Y = cv::Mat::zeros(height, width, CV_8U);
            cv::Mat buffer_U = cv::Mat::zeros(height / 2, width / 2, CV_8U);
            cv::Mat buffer_V = cv::Mat::zeros(height / 2, width / 2, CV_8U);

            memcpy((char*)buffer_Y.data, (const char*)data, width*height);
            memcpy((char*)buffer_U.data, (const char*)data + width*height, width*height / 4);
            memcpy((char*)buffer_V.data, (const char*)data + width*height + width*height / 4, width*height / 4);

            cv::Mat buffer_U_res, buffer_V_res;
            cv::resize(buffer_U, buffer_U_res, cv::Size(0, 0), 2.0, 2.0, CV_INTER_NN);
            cv::resize(buffer_V, buffer_V_res, cv::Size(0, 0), 2.0, 2.0, CV_INTER_NN);

            vector<cv::Mat> buffers;
            buffers.push_back(buffer_Y);
            buffers.push_back(buffer_U_res);
            buffers.push_back(buffer_V_res);

            cv::merge(buffers, buffer);
        }
        else
        {
            memcpy((char*)(buffer.data), (const char*)data, width*height*bpp/8);
        }

        // If present, we dont keep the alpha channel
        if (buffer.channels() > 3)
        {
            cv::Mat temp;
            cv::cvtColor(buffer, temp, CV_RGBA2RGB);
            buffer = temp;
        }

        if (abs(red) > blue && channels >= 3 && !isGray && !isYUV)
            cvtColor(buffer, buffer, CV_BGR2RGB);
        else if (isYUV && !is420)
            cvtColor(buffer, buffer, CV_YUV2BGR_UYVY);
        else if (isYUV && is420)
            cvtColor(buffer, buffer, CV_YCrCb2RGB);

        context->mBuffer = buffer;
        context->mUpdated = true;
    }
    else
    {
        return;
    }

    shmdata_any_reader_free(shmbuf);
}
#endif // HAVE_SHMDATA
