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
 * @helpers.h
 * Function to help handling of various things
 */

#ifndef HELPERS_H
#define HELPERS_H

#include "atom/message.h"

/*************/
// Function to read a value from a message
template<class T>
bool readParam(const atom::Message pParam, T& pValue)
{
    if (pParam.size() < 2)
        return false;

    auto tag = pParam[1].get()->getTypeTag();
    if (tag == atom::FloatValue::TYPE_TAG)
    {
        float value;
        value = atom::FloatValue::convert(pParam[1])->getFloat();
        reinterpret_cast<float&>(pValue) = value;
    }
    else if (tag == atom::IntValue::TYPE_TAG)
    {
        int value;
        value = atom::IntValue::convert(pParam[1])->getInt();
        reinterpret_cast<int&>(pValue) = value;
    }
    else if (tag == atom::StringValue::TYPE_TAG)
    {
        std::string value;
        value = atom::StringValue::convert(pParam[1])->getString();
        reinterpret_cast<std::string&>(pValue) = value;
    }
    else
        return false;

    return true;
}

#endif // HELPERS_H
