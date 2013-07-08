#include "detector_stitch.h"

using namespace std;

std::string Detector_Stitch::mClassName = "Detector_Stitch";
std::string Detector_Stitch::mDocumentation = "N/A";
unsigned int Detector_Stitch::mSourceNbr = 1;

/*************/
Detector_Stitch::Detector_Stitch()
{
    make();
}

/*************/
Detector_Stitch::Detector_Stitch(int pParam)
{
    make();
}

/*************/
void Detector_Stitch::make()
{
    mOutputBuffer = cv::Mat::zeros(480, 640, CV_8UC3);

    mName = mClassName;
    // OSC path for this detector
    mOscPath = "/blobserver/stitch";

    mFrameNumber = 0;

    for (int i=0; i<2; i++) 
    {
        source_crop[i] = false;
        for (int j=0; j<4; j++) 
        {
            source_crop_parameters[i][j] = 0;
        }
        source_pos[i][0] = 0;
        source_pos[i][1] = 0;
    }

    sprintf(outputShmFile, "tmp/blobserver_stitch_%i", 0);
}

/*************/
atom::Message Detector_Stitch::detect(vector<cv::Mat> pCaptures)
{
    if (pCaptures.size() == 0)
        return mLastMessage;

    mOutputBuffer = pCaptures[0].clone();

#if HAVE_SHMDATA
        ShmImage outputImg = new ShmImage(outputShmFile);
        outputImg.setImage(mOutputBuffer);
#endif

    mFrameNumber++;
    mLastMessage = atom::createMessage("iii", 1, 1, mFrameNumber);

    return mLastMessage;
}

/*************/
void Detector_Stitch::setParameter(atom::Message pMessage)
{
    std::string cmd;

    try
    {
        cmd = toString(pMessage[0]);
    }
    catch (atom::BadTypeTagError exception)
    {
        return;
    }

    if (cmd == "output")
    {
        string output;
        if (!readParam(pMessage, output))
            return;

        sprintf(outputShmFile, "%s", output.c_str());
        cout << "Detector_Stitch output: " << output << endl;
    }
    else if (cmd == "cam0_crop")
    {
        if (pMessage.size() == 5)
        {
            try
            {
                source_crop_parameters[0][0] = atom::toInt(pMessage[1]);
                source_crop_parameters[0][1] = atom::toInt(pMessage[2]);
                source_crop_parameters[0][2] = atom::toInt(pMessage[3]);
                source_crop_parameters[0][3] = atom::toInt(pMessage[4]);
            }
            catch (atom::BadTypeTagError error)
            {
                return;
            }

            source_crop[0] = true;
        }
        else
            return;
    }
    else if (cmd == "cam1_crop")
    {
        if (pMessage.size() == 5)
        {
            try
            {
                source_crop_parameters[1][0] = atom::toInt(pMessage[1]);
                source_crop_parameters[1][1] = atom::toInt(pMessage[2]);
                source_crop_parameters[1][2] = atom::toInt(pMessage[3]);
                source_crop_parameters[1][3] = atom::toInt(pMessage[4]);
            }
            catch (atom::BadTypeTagError error)
            {
                return;
            }

            source_crop[1] = true;
        }
        else
            return;
    }
    else if (cmd == "cam0_pos")
    {
        if (pMessage.size() == 3)
        {
            try
            {
                source_pos[0][0] = atom::toInt(pMessage[1]);
                source_pos[0][1] = atom::toInt(pMessage[2]);
            }
            catch (atom::BadTypeTagError error)
            {
                return;
            }
        }
        else
            return;
    }
    else if (cmd == "cam1_pos")
    {
        if (pMessage.size() == 3)
        {
            try
            {
                source_pos[1][0] = atom::toInt(pMessage[1]);
                source_pos[1][1] = atom::toInt(pMessage[2]);
            }
            catch (atom::BadTypeTagError error)
            {
                return;
            }
        }
        else
            return;
    }
    else
        setBaseParameter(pMessage);
}
