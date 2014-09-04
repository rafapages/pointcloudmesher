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

    // Adding every camera to the cloud to see if it works
    void drawCameras();

    // Combine point clouds
    PointCloud<PointXYZRGBNormalCam> combinePointClouds();
    PointCloud<PointXYZRGBNormalCam> combinePointClouds(std::vector<PointCloud<PointXYZRGBNormalCam>::Ptr > _pointclouds);

    // I/O functions
    void bundlerPointReader(PointXYZRGBNormalCam& _point, std::ifstream& _stream);
    void readMesh(std::string _fileName);
    void writeOneMesh(const unsigned int _index, std::string _fileName);
    void writeMesh(std::string _fileName);
    void bundlerReader(std::string _filename);

private:

    std::vector<PointCloud<PointXYZRGBNormalCam>::Ptr > pointClouds_;
    unsigned int nClouds_;

    std::vector<Camera> cameras_;
    unsigned int nCameras_;



};

#endif
