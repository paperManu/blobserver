/*
 * @detector_stitch.h
 * The Detector_Stitch class.
 */

#ifndef DETECTOR_STITCH_H
#define DETECTOR_STITCH_H

#include "config.h"
#include "detector.h"
 #if HAVE_SHMDATA
#include <mutex>
#include <shmdata/any-data-reader.h>

 /*************/
// Class Detector_Stitch
class Detector_Stitch : public Detector
{
    public:
        Detector_Stitch();
        Detector_Stitch(int pParam);

        static std::string getClassName() {return mClassName;}
        static std::string getDocumentation() {return mDocumentation;}

        atom::Message detect(std::vector<cv::Mat> pCaptures);
        void setParameter(atom::Message pMessage);

    private:
        static std::string mClassName;
        static std::string mDocumentation;
        static unsigned int mSourceNbr;

        unsigned int mFrameNumber;


        void make();
};

#endif // DETECTOR_STITCH_H
