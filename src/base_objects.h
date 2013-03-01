/*
 * Copyright (C) 2013 Emmanuel Durand
 *
 * This file is part of blobserver.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * switcher is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with switcher.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * @base_objects.h
 * A few classes and structs used in multiple files
 */

#ifndef BASE_OBJECTS_H
#define BASE_OBJECTS_H

#include "lo/lo.h"

#include "source.h"
#include "detector.h"

using namespace std;

/*************/
// lo_address in an object
class OscClient
{
    public:
        OscClient(lo_address pAddress) {mAddress = pAddress;}
        ~OscClient() {lo_address_free(mAddress);}

        lo_address get() {return mAddress;}

    private:
        lo_address mAddress;
};

/*************/
// Struct to contain a complete flow, from capture to client
struct Flow
{
    vector<shared_ptr<Source>> sources;
    shared_ptr<Detector> detector;
    shared_ptr<OscClient> client;
    unsigned int id;
    bool run;
};

#endif // BASE_OBJECTS_H
