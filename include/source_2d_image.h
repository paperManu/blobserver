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
 * blobserver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with blobserver.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * @source_2d_image.h
 * The Source_2D_Image class.
 */

#ifndef SOURCE_2D_IMAGE_H
#define SOURCE_2D_IMAGE_H

#include "source_2d.h"

class Source_2D_Image : public Source_2D
{
    public:
        Source_2D_Image();
        Source_2D_Image(std::string pParam);
        ~Source_2D_Image();

        static std::string getClassName() {return mClassName;}
        static std::string getDocumentation() {return mDocumentation;}

        atom::Message getSubsources() const; 

        bool connect();
        bool disconnect();
        bool grabFrame();
        cv::Mat retrieveRawFrame();

        void setParameter(atom::Message pParam);
        atom::Message getParameter(atom::Message pParam) const;

    private:
        static std::string mClassName;
        static std::string mDocumentation;

        cv::Mat mImage;

        void make(std::string pParam);
};

#endif // SOURCE_2D_IMAGE_H
