#include "source.h"

std::string Source::mClassName = "Source";
std::string Source::mDocumentation = "N/A";

/*************/
Source::Source()
{
    mName = mClassName;
    mDocumentation = "N/A";

    mWidth = 0;
    mHeight = 0;
    mChannels = 0;
    mFramerate = 0;
    mSubsourceNbr = 0;
}

/************/
Source::Source(int pParam)
{
    Source();
}
