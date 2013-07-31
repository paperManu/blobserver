#include "source.h"

using namespace std;

std::string Source::mClassName = "Source";
std::string Source::mDocumentation = "N/A";

/*************/
Source::Source():
    mUpdated(false)
{
    mName = mClassName;
    mDocumentation = "N/A";

    mFramerate = 0;

    mSubsourceNbr = 0;
    mId = 0;
}

/************/
Source::Source(int pParam)
{
    Source();
}

/************/
Source::~Source()
{
}
