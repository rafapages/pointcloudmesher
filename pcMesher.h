#ifndef PCMESHER_H
#define PCMESHER_H

#include <iostream>
#include <stdio.h>

#include "pointXYZRGBNormalCam.h"
#include "camera.h"

class PcMesher{

public:

    PcMesher();
    ~PcMesher();

    // Get number of clouds
    unsigned int getNClouds();

    // Estimate the normals of a cloud and store them
    void estimateNormals(const unsigned int _index);
    void estimateAllNormals();

    // Fix the normal orientation using the camera position
    void fixNormal(const unsigned int _index);
    void fixAllNormals();

    // Plane segmentation
    void planeSegmentation();

    // Cilinder segmentation
    void cilinderSegmentation();

    // Surface reconstruction
    void surfaceReconstruction(const unsigned int _index);
    void surfaceReconstruction(PointCloud<PointXYZRGBNormalCam>::Ptr _cloud);
    void allSurfaceReconstruction();

    // Adding every camera to the cloud to see if it works
    void drawCameras();

    // Combine point clouds
    PointCloud<PointXYZRGBNormalCam> combinePointClouds();

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
