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

    mSubsourceNbr = string();
    mId = string();
}

/************/
Source::Source(string pParam)
{
    Source();
}

/************/
Source::~Source()
{
}
