/*
 * Copyright (C) 2013 Emmanuel Durand
 *
 * This file is part of posturevision. 
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 */

/*
 * @shmpointcloud.h
 * The ShmPointCloud class, and utility classes
 */

#ifndef SHMPOINTCLOUD_H
#define SHMPOINTCLOUD_H

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/compression/octree_pointcloud_compression.h>

#include "config.h"
#include "shmdata/any-data-reader.h"
#include "shmdata/any-data-writer.h"

namespace pio = pcl::io;
namespace poct = pcl::octree;

/*************/
//! NetworkPointCloudCompression class, derived from pcl::io::OctreePointCloudCompression with optimizations for network
template<typename PointT, typename LeafT = poct::OctreeContainerPointIndices,
    typename BranchT = poct::OctreeContainerEmpty,
    typename OctreeT = poct::Octree2BufBase<LeafT, BranchT> >
class NetworkPointCloudCompression : public pio::OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>
{
    typedef typename pio::OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::PointCloudPtr PointCloudPtr;

    public:
        /**
         * \brief Constructor
         * \param compressionProfile_arg Specify here a known compression profile
         * \param showStatistics_arg Output compression statistics
         * \param pointResolution_arg Precision of point coordinates
         * \param octreeResolution_arg Octree resolution at lowest octree level
         * \param doVoxelGridDownDownSampling_arg Voxel grid filtering
         * \param iFrameRate_arg I-frame encoding rate
         * \param doColorEncoding_arg Enable/disable color coding
         * \param colorBitResolution_arg Color bit depth
         */
        NetworkPointCloudCompression(const pio::compression_Profiles_e compressionProfile_arg = pio::MED_RES_ONLINE_COMPRESSION_WITH_COLOR,
                                     const bool showStatistics_arg = false,
                                     const double pointResolution_arg = 0.001,
                                     const double octreeResolution_arg = 0.01,
                                     const bool doVoxelGridDownDownSampling_arg = false,
                                     const unsigned int iFrameRate_arg = 0,
                                     const bool doColorEncoding_arg = true,
                                     const unsigned char colorBitResolution_arg = 6) :
            pio::OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT> (compressionProfile_arg, showStatistics_arg, pointResolution_arg,
                octreeResolution_arg, doVoxelGridDownDownSampling_arg, iFrameRate_arg, doColorEncoding_arg, colorBitResolution_arg),
            validFrame_(PointCloudPtr()),
            prevValidFrame_(0)
        {
        }

        /**
         * \brief Destructor
         */
        ~NetworkPointCloudCompression()
        {
        }

        /**
         * \brief Decode the point cloud contained in the given istream to the given PointCloudPtr
         * \param compressedTreeDataIn_arg Binary input stream containing compressed data
         * \param cloud_arg Reference to the decoded point cloud
         */
        void decodeNetworkPointCloud(std::istream &compressedTreeDataIn_arg, PointCloudPtr &cloud_arg);

    private:
        PointCloudPtr validFrame_;
        bool firstFrame_;

        unsigned int prevValidFrame_;
};

/*************/
template<typename PointT, typename LeafT, typename BranchT, typename OctreeT>
void NetworkPointCloudCompression<PointT, LeafT, BranchT, OctreeT>::decodeNetworkPointCloud(std::istream &compressedTreeDataIn_arg,
    PointCloudPtr &cloud_arg)
{
    typename pcl::PointCloud<PointT>::Ptr tempCloud (new pcl::PointCloud<PointT>());

    try
    {
        this->decodePointCloud(compressedTreeDataIn_arg, tempCloud);
        if (this->frame_ID_ == prevValidFrame_+1 || this->i_frame_ == true)
        {
            validFrame_ = tempCloud;
            prevValidFrame_ = this->frame_ID_;
            firstFrame_ = false;
        }
    }
    catch (...)
    {
        std::cout << "Exception caught." << std::endl;
    }

    if (firstFrame_ == false)
    {
        cloud_arg = validFrame_;
    }
}

/*************/
//! PointCloudBlob class, designed to carry uncompressed cloud over the network
template <typename T>
class PointCloudBlob
{
    public:
        /**
         * \brief Constructor
         */
        PointCloudBlob();

        /**
         * \brief Destructor
         */
        ~PointCloudBlob() {free(_blob);}

        /**
         * \brief Encodes the specified PointCloud to a binary blob
         * \param cloud Reference to the cloud
         * \param size Reference to the size of the blob after being set
         * \param timestamp Timestamp for the cloud.
         * \return A pointer to the resulting binary blob of the cloud
         */
        void* toBlob(typename pcl::PointCloud<T>::Ptr &cloud, int &size, const unsigned long long timestamp = 0);

        /**
         * \brief Decodes the specified binary blob to a PointCloud
         * \param blob Pointer to the binary blob to decode
         * \param size Size of the blob
         * \param timestamp Timestamp will be stored in this pointer
         * \return A shared pointer to a pcl::PointCloud
         */
        typename pcl::PointCloud<T>::Ptr toCloud(const void* blob, const int size, unsigned long long* timestamp = NULL);

    private:
        float* _blob;
        unsigned int _size;
};

/*************/
template <typename T>
PointCloudBlob<T>::PointCloudBlob()
{
    _blob = NULL;
    _size = 0;
}

/*************/
template <typename T>
void* PointCloudBlob<T>::toBlob(typename pcl::PointCloud<T>::Ptr &cloud, int &size, const unsigned long long timestamp)
{
    // Get the size for the blob
    const size_t cloudSize = cloud->size();
    const unsigned int blobSize = cloudSize * (3 + 1); // each point is 3 floats for xyz and one float for the color

    if (blobSize > _size)
    {
        free(_blob);
        _blob = (float*)malloc(blobSize*sizeof(float) + sizeof(unsigned long long));
        _size = blobSize;
    }

    unsigned long long* stampPtr = (unsigned long long*)_blob;
    *stampPtr = timestamp;

    float* blobPtr = (float*)((char*)_blob + sizeof(unsigned long long));
    for (unsigned int i = 0; i < cloudSize; ++i)
    {
        T* point = &(cloud->at(i));
        blobPtr[i*4 + 0] = point->x;
        blobPtr[i*4 + 1] = point->y;
        blobPtr[i*4 + 2] = point->z;
        blobPtr[i*4 + 3] = point->rgb;
    }

    size = blobSize*sizeof(float) + sizeof(unsigned long long);
    return (void*)_blob;
}

/*************/
template <typename T>
typename pcl::PointCloud<T>::Ptr PointCloudBlob<T>::toCloud(const void* blob, const int size, unsigned long long* timestamp)
{
    typename pcl::PointCloud<T>::Ptr cloud(new pcl::PointCloud<T>());

    if (timestamp != NULL)
    {
        unsigned long long* stampPtr = (unsigned long long*)blob;
        *timestamp = *stampPtr;
    }

    // Get the size of the cloud
    const float* newBlob = (float*)((char*)blob + sizeof(unsigned long long));
    const unsigned int cloudSize = (size - sizeof(unsigned long long)) / (4*sizeof(float));

    cloud->reserve(cloudSize);

    for (unsigned int i = 0; i < cloudSize; ++i)
    {
        T point;
        point.x = newBlob[i*4 + 0];
        point.y = newBlob[i*4 + 1];
        point.z = newBlob[i*4 + 2];
        point.rgb = newBlob[i*4 + 3];

        cloud->push_back(point);
    }

    return cloud;
}

/*************/
#define SHMCLOUD_TYPE_BASE          "application/x-pcl"
#define SHMCLOUD_TYPE_COMPRESSED    "application/x-pcd"

/*************/
//! ShmPointCloud class, to write point clouds to shmdata
template <typename T>
class ShmPointCloud
{
    public:
        typedef typename pcl::PointCloud<T>::Ptr cloudPtr;

        /**
         * \brief Constructor
         * \param filename File path to the shmdata
         * \param isWriter Set to true to create a writer, false for a reader
         */
        ShmPointCloud(const char* filename, const bool isWriter = false);

        /**
         * \brief Destructor
         */
        ~ShmPointCloud();

        /**
         * \brief Set the compression to apply to the input point clouds. Only meaningful if the ShmPointCloud is set as a reader.
         * \param compressionProfile_arg Specify here a known compression profile
         * \param showStatistics_arg Output compression statistics
         * \param pointResolution_arg Precision of point coordinates
         * \param octreeResolution_arg Octree resolution at lowest octree level
         * \param doVoxelGridDownDownSampling_arg Voxel grid filtering
         * \param iFrameRate_arg I-frame encoding rate
         * \param doColorEncoding_arg Enable/disable color coding
         * \param colorBitResolution_arg Color bit depth
         */
        void setCompression(const pio::compression_Profiles_e compressionProfile = pio::MED_RES_ONLINE_COMPRESSION_WITH_COLOR,
                            const bool showStatistics = false,
                            const double pointResolution = 0.001,
                            const double octreeResolution = 0.01,
                            const bool doVoxelGridDownDownSampling = false,
                            const unsigned int iFrameRate = 0,
                            const bool doColorEncoding = true,
                            const unsigned char colorBitResolution = 6);

        /**
         * \brief Get the state of the ShmPointCloud
         * \return True if a new point cloud is available
         */
        bool isUpdated() const {return _updated;}

        /**
         * \brief Get the latest point cloud received. Only meaningful if the ShmPointCloud is set as a reader.
         * \param cloud Reference to the cloud where to write
         * \return The timestamp
         */
        unsigned long long getCloud(cloudPtr &cloud) const;

        /**
         * \brief Set a new point cloud to the shmdata. Only meaningful if the ShmPointCloud is set as a writer.
         * \param cloud Reference to the PointCloud
         * \param compress Set to true to compress the point cloud
         * \param timestamp Timestamp of the new cloud
         */
        void setCloud(const cloudPtr &cloud, const bool compress = false, const unsigned long long timestamp = 0);

    private:
        bool _isWriter;

        string _filename;

        shmdata_any_writer_t* _writer;
        shmdata_any_reader_t* _reader;
        cloudPtr _cloud;

        mutable bool _updated;
        unsigned long long _timestamp;
	unsigned long long _startTime;

        char* _dataBuffer;
        unsigned int _dataBufferSize;

        mutable std::mutex _mutex;

        std::shared_ptr<NetworkPointCloudCompression<T>> _networkPointCloudEncoder;
        std::shared_ptr<PointCloudBlob<T>> _blober;

        static void onData(shmdata_any_reader_t* reader, void* shmbuf, void* data, int data_size, unsigned long long timestamp,
            const char* type_description, void* user_data);
};

/*************/
template <typename T>
ShmPointCloud<T>::ShmPointCloud(const char* filename, const bool isWriter) :
    _writer(NULL),
    _reader(NULL),
    _updated(false),
    _startTime(0)
{
    _cloud.reset(new pcl::PointCloud<T>());

    _networkPointCloudEncoder.reset(new NetworkPointCloudCompression<T>());
    _blober.reset(new PointCloudBlob<T>());

    _filename = string(filename);

    _isWriter = isWriter;
    if (_isWriter)
    {
        // Shmdata writer
        _writer = shmdata_any_writer_init();
        shmdata_any_writer_set_data_type(_writer, SHMCLOUD_TYPE_BASE);
        if (!shmdata_any_writer_set_path(_writer, (char*)(filename)))
        {
            cout << "**** The file " << filename << " exists, therefore a shmdata cannot be operated with this path." << endl;
            shmdata_any_writer_close(_writer);
        }
        shmdata_any_writer_start(_writer);
    }
    else
    {
        // Shmdata reader
        _reader = shmdata_any_reader_init();
        shmdata_any_reader_run_gmainloop(_reader, SHMDATA_FALSE);
        shmdata_any_reader_set_on_data_handler(_reader, ShmPointCloud::onData, this);
        //shmdata_any_reader_set_absolute_timestamp(_reader, SHMDATA_ENABLE_ABSOLUTE_TIMESTAMP);
        shmdata_any_reader_start(_reader, (char*)(filename));
    }

    // Buffer
    _dataBufferSize = 1e6; // 1MB is a good start for this buffer
    _dataBuffer = (char*)malloc(_dataBufferSize*sizeof(char));
}

/*************/
template <typename T>
ShmPointCloud<T>::~ShmPointCloud()
{
    if (_reader != NULL)
        shmdata_any_reader_close(_reader);

    if (_writer != NULL)
        shmdata_any_writer_close(_writer);

    free(_dataBuffer);
}

/*************/
template <typename T>
void ShmPointCloud<T>::setCompression(pio::compression_Profiles_e compressionProfile, bool showStatistics,
                                      const double pointResolution, const double octreeResolution,
                                      bool doVoxelGridDownDownSampling, const unsigned int iFrameRate,
                                      bool doColorEncoding, const unsigned char colorBitResolution)
{
    _networkPointCloudEncoder.reset(new NetworkPointCloudCompression<T>(compressionProfile, showStatistics, pointResolution,
                                                                        octreeResolution, doVoxelGridDownDownSampling, iFrameRate,
                                                                        doColorEncoding, colorBitResolution));
}

/*************/
template <typename T>
unsigned long long ShmPointCloud<T>::getCloud(cloudPtr &cloud) const
{
    std::lock_guard<std::mutex> lock(_mutex);
    cloud = _cloud;
    _updated = false;
    return _timestamp;
}

/*************/
template <typename T>
void ShmPointCloud<T>::setCloud(const cloudPtr &cloud, const bool compress, const unsigned long long timestamp)
{
    if (cloud.get() == NULL || cloud->size() == 0 || !_isWriter)
        return;

    cloudPtr lCloud = const_cast<cloudPtr&>(cloud);

    static bool wasCompressed = false;
    if (compress != wasCompressed)
    {
        // We changed transmission mode, we need to restart the writer
        cout << "**** Restarting shmdata writer." << endl;

        shmdata_any_writer_close(_writer);
        _writer = shmdata_any_writer_init();

        if (compress)
            shmdata_any_writer_set_data_type(_writer, SHMCLOUD_TYPE_COMPRESSED);
        else
            shmdata_any_writer_set_data_type(_writer, SHMCLOUD_TYPE_BASE);

        if (!shmdata_any_writer_set_path(_writer, (char*)(_filename.c_str())))
        {
            cout << "**** The file " << _filename << " exists, therefore a shmdata cannot be operated with this path." << endl;
            shmdata_any_writer_close(_writer);
        }
        shmdata_any_writer_start(_writer);

        wasCompressed = compress;
    }

    // Get the current timestamp
    unsigned long long currentTime;
    if (timestamp == 0)
    {
        auto now = std::chrono::high_resolution_clock::now();
        currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    }
    else
    {
        currentTime = timestamp;
    }

    if (_startTime == 0)
      _startTime = currentTime;

    // Compress (or not) and send through shmdata
    // Timestamp is added to the start of the buffer
    if (compress)
    {
        unsigned long long* stampBuffer = (unsigned long long*)_dataBuffer;
        *stampBuffer = currentTime;

        stringstream compressedData;
        // TODO: We should check if the next line is necessary in the future. As of yet, without it the whole history
        // of all clouds is kepts unless a big change happens...
        _networkPointCloudEncoder->deleteTree();
        _networkPointCloudEncoder->encodePointCloud(lCloud, compressedData);
        compressedData.read(reinterpret_cast<char*>(_dataBuffer + sizeof(unsigned long long)), _dataBufferSize);

        unsigned int size = compressedData.gcount() + sizeof(unsigned long long);
        if (size >= _dataBufferSize)
        {
            // It is likely that the buffer is too short, we will resize it
            free(_dataBuffer);
            _dataBufferSize *= 2;
            _dataBuffer = (char*)malloc(_dataBufferSize*sizeof(char));
        }
        else
        {
            shmdata_any_writer_push_data(_writer, 
					 _dataBuffer, 
					 size, 
					 (currentTime - _startTime) * 1e6, //nsec
					 NULL, 
					 NULL);
        }
    }
    else
    {
        int dataSize;
        void* blob =_blober->toBlob(lCloud, dataSize, currentTime);

        shmdata_any_writer_push_data(_writer, 
				     blob, 
				     dataSize, 
				     (currentTime - _startTime) * 1e6, //nsec
				     NULL, 
				     NULL);
    }

    auto now = std::chrono::high_resolution_clock::now();
    currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
}

/*************/
template <typename T>
void ShmPointCloud<T>::onData(shmdata_any_reader_t* reader, void* shmbuf, void* data, int data_size, unsigned long long timestamp,
    const char* type_description, void* user_data)
{
    ShmPointCloud* context = static_cast<ShmPointCloud*>(user_data);
    
    std::lock_guard<std::mutex> lock(context->_mutex);

    cloudPtr cloudOut(new pcl::PointCloud<T>());

    unsigned long long stamp;
    string dataType(type_description);
    if (dataType == string(SHMCLOUD_TYPE_COMPRESSED))
    {
        unsigned long long* stampPtr = (unsigned long long*)data;
        stamp = *stampPtr;

        std::stringstream compressedData;
        compressedData.write((const char*)data + sizeof(unsigned long long), data_size);
        context->_networkPointCloudEncoder->decodeNetworkPointCloud(compressedData, cloudOut);
        context->_cloud = cloudOut;
    }
    else if (dataType == string(SHMCLOUD_TYPE_BASE))
    {
        context->_cloud = context->_blober->toCloud(data, data_size, &stamp);
    }
    else
    {
        return;
    }

    context->_timestamp = stamp;
    context->_updated = true;

    shmdata_any_reader_free(shmbuf);
}

#endif // SHMPOINTCLOUD_H
