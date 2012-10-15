#include "blob.h"

/*************/
Blob::Blob()
{
    static int lIdCounter = 0;
    lIdCounter++;
    mId = lIdCounter;

    updated = false;

}

/*************/
Blob::~Blob()
{
}

/*************/
Blob::properties Blob::getBlob()
{
    return mProperties;
}

/*************/
bool Blob::isUpdated()
{
    return updated;
}
