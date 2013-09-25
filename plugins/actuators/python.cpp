#include "python.h"

using namespace std;

std::string Actuator_Python::mClassName = "Actuator_Python";
std::string Actuator_Python::mDocumentation = "N/A";
unsigned int Actuator_Python::mSourceNbr = 1;

/*************/
Actuator_Python::Actuator_Python()
{
    make();
}

/*************/
Actuator_Python::Actuator_Python(int pParam)
{
    make();
}

/*************/
void Actuator_Python::make()
{
    mOutputBuffer = cv::Mat::zeros(480, 640, CV_8UC3);

    mName = mClassName;
    // OSC path for this actuator
    mOscPath = "python";

    mFrameNumber = 0;

    mIsFileLoaded = false;
    mPythonModule = NULL;

    mRawCapture = NULL;
    mRawRows = mRawCols = mRawChannels = 0;

    Py_Initialize();

    mPythonMain = PyImport_AddModule("__main__");
    if (mPythonMain != NULL)
    {
        mPythonGlobal = PyModule_GetDict(mPythonMain);
    }
    if (!mPythonGlobal)
        return;

    if (PyErr_Occurred())
        PyErr_Print();

    mPythonOutput = PyList_New(0);
    PyDict_SetItemString(mPythonGlobal, "blobOutput", mPythonOutput);
}

/*************/
Actuator_Python::~Actuator_Python()
{
    Py_Finalize();
}

/*************/
atom::Message Actuator_Python::detect(vector< Capture_Ptr > pCaptures)
{
    vector<cv::Mat> captures = captureToMat(pCaptures);
    if (captures.size() < 1)
    {
        g_log(NULL, G_LOG_LEVEL_WARNING, "%s: Not enough valid sources to process", mClassName.c_str());
        return mLastMessage;
    }
    cv::Mat capture = captures[0];

    if (mRawCapture == NULL || mRawRows != capture.rows || mRawCols != capture.cols || mRawChannels != capture.channels())
    {
        mRawCapture = PyList_New(capture.rows);
        for (int y = 0; y < capture.rows; ++y)
        {
            PyObject* row = PyList_New(capture.cols);
            for (int x = 0; x < capture.cols; ++x)
            {
                PyObject* channels = PyList_New(capture.channels());
                PyList_SetItem(row, x, channels);
            }
            PyList_SetItem(mRawCapture, y, row);
        }

        mRawRows = capture.rows;
        mRawCols = capture.cols;
        mRawChannels = capture.channels();
    }

    for (int y = 0; y < capture.rows; ++y)
    {
        PyObject* row = PyList_GetItem(mRawCapture, y);
        for (int x = 0; x < capture.cols; ++x)
        {
            PyObject* channels = PyList_GetItem(row, x);
            for (int c = 0; c < capture.channels(); ++c)
            {
                PyObject* value = PyInt_FromLong((long)capture.at<cv::Vec3b>(y, x)[c]);
                PyList_SetItem(channels, c, value);
            }
        }
    }

    PyObject* result = PyObject_CallFunctionObjArgs(mPythonModule, mRawCapture, NULL);
    if (PyErr_Occurred())
        PyErr_Print();

    if (!PyList_Check(result))
    {
        g_log(NULL, G_LOG_LEVEL_WARNING, "%s: Return value of the Python script should be a list", mClassName.c_str());
        return mLastMessage;
    }

    vector<float> values;
    for (int i = 0; i < PyList_Size(result); ++i)
    {
        PyObject* value = PyList_GetItem(result, i);
        if (PyLong_Check(value))
            values.push_back(PyLong_AsLong(value));
        else if (PyInt_Check(value))
            values.push_back(PyInt_AsLong(value));
        else if (PyFloat_Check(value))
            values.push_back(PyFloat_AsDouble(value));
    }

    mLastMessage.clear();
    mLastMessage.push_back(atom::IntValue::create(1));
    mLastMessage.push_back(atom::IntValue::create(values.size()));
    for (int i = 0; i < values.size(); ++i)
        mLastMessage.push_back(atom::FloatValue::create(values[i]));

    return mLastMessage;
}

/*************/
void Actuator_Python::setParameter(atom::Message pMessage)
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

    if (cmd == "file")
    {
        string filename;
        if (!readParam(pMessage, filename))
            return;

        g_log(NULL, G_LOG_LEVEL_INFO, "%s - Attempting to load Python file %s", mClassName.c_str(), filename.c_str());
       
        FILE* file = NULL;
        file = fopen((filename + string(".py")).c_str(), "r");
        if (file == NULL)
        {
            g_log(NULL, G_LOG_LEVEL_WARNING, "%s - Unable to load file %s", mClassName.c_str(), filename.c_str());
            return;
        }

        if (PyRun_SimpleFile(file, filename.c_str()) != 0)
        {
            g_log(NULL, G_LOG_LEVEL_WARNING, "%s - Python failed to load file %s", mClassName.c_str(), filename.c_str());
            if (PyErr_Occurred())
                PyErr_Print();
            return;
        }

        if (mPythonGlobal != NULL)
            mPythonModule = PyDict_GetItemString(mPythonGlobal, filename.c_str());

        if (mPythonModule == NULL)
            g_log(NULL, G_LOG_LEVEL_WARNING, "%s - Error while loading python module %s. Maybe the function %s is not defined?", mClassName.c_str(), filename.c_str(), filename.c_str());

        mOscPath = filename;

        fclose(file);
    }
    setBaseParameter(pMessage);
}
