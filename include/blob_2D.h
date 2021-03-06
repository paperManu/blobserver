/*
 * Copyright (C) 2012 Emmanuel Durand
 *
 * This file is part of blobserver.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * blobserver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with blobserver.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * @blob_2D.h
 * The blob_2D class.
 */

#ifndef BLOB_2D_H
#define BLOB_2D_H

#include "blob.h"

class Blob2D : public Blob
{
    public:
        Blob2D();

        void setParameter(const char* pParam, float pValue);

        void init(properties pNewblob);
        properties predict();
        void setNewMeasures(properties pNewBlob);
        float getDistanceFromPrediction(properties pBlob);

    private:
};

#endif // BLOB_2D_H
