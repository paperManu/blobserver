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
 * @source.h
 * The Source base class.
 */

#ifndef SOURCE_H
#define SOURCE_H

#include <atomic>
#include <mutex>
#include <vector>

#include <glib.h>
#include <atom/message.h>

#include "capture.h"
#include "helpers.h"

/*************/
//! Base Source class, from which all Source classes derive
class Source
{
    public:
            /**
         * \brief Default constructor
         */
        Source();

        /**
         * \brief Constructor allowing to specify a subsource
         * \param pParam Index of the subsource to allocate
         */
        Source(int pParam);

        /**
         * \brief Destructor
         */
        ~Source();

        /**
         * \return Returns the class name of the source
         */
        static std::string getClassName() {return mClassName;}

        /**
         * \return Returns the documentation of the source
         */
        static std::string getDocumentation() {return mDocumentation;}

        /**
         * \brief Allows to get all the availables subsources for a given source
         * \return Returns a message containing all subsources name
         */
        virtual atom::Message getSubsources() const {return atom::Message();}

        // Base methods
        /**
         * \brief Connects to the source, depending on the specified subsource number
         */
        virtual bool connect() {return true;}

        /**
         * \brief Disconnects the source
         */
        virtual bool disconnect() {return true;}

        /**
         * \brief Tells the source to grab a frame, without returning it yet
         */
        virtual bool grabFrame() {}

        /**
         * \brief Returns true if a new grab is available to retrieve
         */
        bool isUpdated() {return mUpdated;}

        /**
         * \brief Retrieves the last frame grabbed by the source with grabFrame()
         */
        virtual Capture_Ptr retrieveFrame() {return Capture_Ptr();}

        /**
         * \brief Sets a parameter
         * \param pParam A message containing the name of the parameter, and its desired value
         */
        virtual void setParameter(atom::Message pParam) {}

        /**
         * \brief Gets the current value for a given parameter
         * \param pParam A message containing the name of the parameter
         * \return Returns a message containing the name of the parameter and its current value
         */
        virtual atom::Message getParameter(atom::Message pParam) const {}

        /**
         * \brief Gets the name of the source
         */
        std::string getName() const {return mName;}

        /**
         * \brief Gets the framerate of the source
         */
        unsigned int getFramerate() const {return mFramerate;}

        /**
         * \brief Gets the subsource number
         */
        unsigned int getSubsourceNbr() const {return mSubsourceNbr;}

    protected:
        bool mUpdated; //!< Flag set to true if a new grab is available
        mutable std::mutex mMutex; //!< Mutex to prevent concurrent read/write of mBuffer

        std::string mName;
        unsigned int mFramerate;

        unsigned int mSubsourceNbr;
        unsigned int mId;

    private:
        static std::string mClassName; //!< Class name, to be set in child class
        static std::string mDocumentation; //!< Class documentation, to be set in child class
};

#endif // SOURCE_H
