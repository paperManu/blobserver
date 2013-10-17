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
 * @actuator.h
 * The actuator base class.
 */

#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <algorithm>

#include <glib.h>
#include "atom/message.h"
#include "opencv2/opencv.hpp"

#include "abstract-factory.h"
#include "capture.h"
#include "helpers.h"
#include "source.h"

/*************/
// Macro for registering Actuator plugins
#define REGISTER_ACTUATOR(act) \
extern "C" \
{ \
    void registerToFactory(factory::AbstractFactory<Actuator, std::string, std::string, int>& factory) \
    { \
        g_log(NULL, G_LOG_LEVEL_DEBUG, "Registering class %s", act::getClassName().c_str()); \
        factory.register_class<act>(act::getClassName(), act::getDocumentation()); \
    } \
}

/*************/
//! Base Actuator class, from which all actuators derive
class Actuator
{
    public:
        /**
         * \brief Constructor
         */
        Actuator();
        Actuator(int pParam);

        /**
         * \brief Destructor
         */
        virtual ~Actuator() {};

        /**
         * \brief Gets the class name of the actuator
         */
        static std::string getClassName() {return mClassName;}
        /**
         * \brief Gets the class documentation of the actuator
         */
        static std::string getDocumentation() {return mDocumentation;}
        /**
         * \brief Get the number of sources this actuator needs
         */
        static unsigned int getSourceNbr() {return mSourceNbr;}

        /**
         * Detects objects in the capture given as a parameter, and returns a message with informations about each blob
         * The two first values in the message are the number of blob, and the size of each blob in the message
         * \param pCaptures A vector containing all captures. Their number should match mSourceNbr.
         */
        virtual atom::Message detect(const std::vector< Capture_Ptr > pCaptures) {return atom::Message();}
        
        /**
         * \brief Returns the message from the last call to detect()
         */
        atom::Message getLastMessage() const {return mLastMessage;}

        /**
         * \brief Sets the mask to use on detection
         */
        void setMask(cv::Mat pMask);
        
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
        atom::Message getParameter(atom::Message pParam) const;
        
        /**
         * \brief Gives a ptr to the actuator, for it to control the source (if needed)
         * \param source A shared_ptr to the source. A weak_ptr is created from it.
         */
        void addSource(std::shared_ptr<Source> source);
        
        /**
         * \brief Gets the name to use in the osc path when sending the message related to this actuator
         */
        std::string getName() const {return mName;}

        /**
         * \brief Gets the full OSC path to use for sending message from this actuator
         */
        std::string getOscPath() const {return mOscPath;}

        /**
         * \brief Gets the resulting image from the actuator.
         */
        virtual std::vector<Capture_Ptr> getOutput() const;

    protected:
        cv::Mat mOutputBuffer; //!< The output buffer, resulting from the detection
        atom::Message mLastMessage; //!< Last message built by detect()
        bool mVerbose;

        std::string mOscPath; //!< OSC path for the actuator, to be set in child class
        std::string mName; // !< Name of the actuator, to be set in child class

        std::vector<std::weak_ptr<Source>> mSources;

        // Methods
        cv::Mat getMask(cv::Mat pCapture, int pInterpolation = CV_INTER_NN);
        void setBaseParameter(const atom::Message pMessage);
        std::vector<cv::Mat> captureToMat(std::vector< Capture_Ptr > pCaptures);

    private:
        static std::string mClassName; //!< Class name, to be set in child class
        static std::string mDocumentation; //!< Class documentation, to be set in child class
        static unsigned int mSourceNbr; //!< Number of sources needed for the actuator, to be set in child class

        cv::Mat mSourceMask, mMask;
};

#endif // ACTUATOR_H
