#ifndef PCMESHER_H
#define PCMESHER_H

#include <iostream>
#include <stdio.h>

#include <pcl/surface/poisson.h>


#include "pointXYZRGBNormalCam.h"
#include "camera.h"


class PcMesher{

public:

    PcMesher();
    ~PcMesher();

    // Get number of clouds
    unsigned int getNClouds();

    // Get a Ptr to a cloud
    PointCloud<PointXYZRGBNormalCam>::Ptr getPointCloudPtr(unsigned int _index);

    // Outlier filtering
    void removeOutliers(PointCloud<PointXYZRGBNormalCam>::Ptr& _cloud);
    void removeOutliers(unsigned int _index);
    void removeAllOutliers();


    // Estimate the normals of a cloud and store them
    void estimateNormals(const unsigned int _index);
    void estimateAllNormals();

    // Fix the normal orientation using the camera position
    void fixNormal(const unsigned int _index);
    void fixAllNormals();

    // Plane segmentation
    void planeSegmentation();

    // Cilinder segmentation
    void cylinderSegmentation();

    // Surface reconstruction
    void surfaceReconstruction(const unsigned int _index);
    PolygonMesh surfaceReconstruction(PointCloud<PointXYZRGBNormalCam>::Ptr _cloud);
    void allSurfaceReconstruction();

    // Mesh refining
    PolygonMesh deleteWrongVertices(PointCloud<PointXYZRGBNormalCam>::Ptr _cloud, PolygonMesh _inputMesh);
    PolygonMesh decimateMesh(const PolygonMesh& _mesh);

    // Adding every camera to the cloud to see if it works
    void drawCameras();

    // Export a txt file with the camera setup information for the multi-texturing process
    void writeCameraSetupFile(std::string _fileName, const int _width, const int _height);

    // Combine point clouds
    PointCloud<PointXYZRGBNormalCam> combinePointClouds();
    PointCloud<PointXYZRGBNormalCam> combinePointClouds(std::vector<PointCloud<PointXYZRGBNormalCam>::Ptr > _pointclouds);

    // I/O functions
    void bundlerPointReader(PointXYZRGBNormalCam& _point, std::ifstream& _stream);
    void readMesh(std::string _fileName);
    void writeOneMesh(const unsigned int _index, std::string _fileName);
    void writeMesh(std::string _fileName);
    void bundlerReader(std::string _fileName);
    void nvmCameraReader(std::string _fileName);

private:

    std::vector<PointCloud<PointXYZRGBNormalCam>::Ptr > pointClouds_;
    unsigned int nClouds_;

    std::vector<Camera> cameras_;
    unsigned int nCameras_;

    std::vector<unsigned int> cameraOrder_;



};

#endif
