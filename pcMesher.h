#ifndef PCMESHER_H
#define PCMESHER_H

#include <iostream>
#include <string>
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

    // Get the vector of point-camera correspondance
    std::vector<std::vector<int> > getCamPerVtx();

    // Get a Ptr to a cloud
    PointCloud<PointXYZRGBNormalCam>::Ptr getPointCloudPtr(unsigned int _index);

    // Remove pointclouds
    void clearPointClouds();

    // Get dimensions of Point Cloud or Mesh
    Eigen::Vector3f getDimensions(const PointCloud<PointXYZRGBNormalCam>::Ptr& _cloud);
    Eigen::Vector3f getDimensions(const unsigned int _index);
    Eigen::Vector3f getDimensions(const PolygonMesh& _mesh);

    // Outlier filtering
    void removeOutliers(PointCloud<PointXYZRGBNormalCam>::Ptr& _cloud);
    void removeOutliers(unsigned int _index);
    void removeAllOutliers();

    // Estimate the normals of a cloud and store them
    void estimateNormals(const unsigned int _index);
    void estimateNormals(const unsigned int _index, const float _radius);
    void estimateAllNormals();
    void estimateAllNormals(const float _radius);

    // Fix the normal orientation using the camera position
    void fixNormal(const unsigned int _index);
    void fixAllNormals();

    // Cluster the point cloud into several sets
    void extractClusters(const unsigned int _index);

    // Plane segmentation
    void segmentPlanes(float _threshold);

    // Cilinder segmentation
    void segmentCylinders();

    // Surface reconstruction
    void surfaceReconstruction(const unsigned int _index);
    PolygonMesh surfaceReconstruction(PointCloud<PointXYZRGBNormalCam>::Ptr _cloud);
    void allSurfaceReconstruction();

    // Mesh refining
    PolygonMesh deleteWrongVertices(PointCloud<PointXYZRGBNormalCam>::Ptr _cloud, PolygonMesh& _inputMesh);
    PolygonMesh deleteWrongVertices2(PointCloud<PointXYZRGBNormalCam>::Ptr _cloud, PolygonMesh _inputMesh);
    PolygonMesh decimateMesh(const PolygonMesh& _mesh, float _reduction);
    PolygonMesh smoothMeshLaplacian(const PolygonMesh& _mesh);

    // Mesh test operations
    bool isMeshOpen(const PolygonMesh& _inputMesh) const;

    // Adding every camera to the cloud to see if their position is correct
    void drawCameras();

    // Export a txt file with the camera setup information for the multi-texturing process
    void writeCameraSetupFile(std::string _fileName, const int _width, const int _height);
    void writeCameraSetupFile(std::string _fileName);
    void getImageDimensions(std::string _imageName, unsigned int& _height, unsigned int& _width);

    // Combine point clouds
    PointCloud<PointXYZRGBNormalCam> combinePointClouds();
    PointCloud<PointXYZRGBNormalCam> combinePointClouds(std::vector<PointCloud<PointXYZRGBNormalCam>::Ptr > _pointclouds);

    // Asign cameras to mesh and export a file with it
    void assignCam2Mesh(const PolygonMesh& _mesh, const PointCloud<PointXYZRGBNormalCam>::Ptr _cloud, const std::string _fileName);

    // I/O functions
    void bundlerPointReader(PointXYZRGBNormalCam& _point, std::ifstream& _stream);
    void readPLYCloud(const std::string& _fileName);
    void writeOneCloud(const unsigned int _index, const std::string _fileName);
    void writeCloud(const std::string _fileName);
    void bundlerReader(const std::string _fileName);
    void readImageList(const std::string _fileName);

    void readPLYMesh(const std::string _fileName, PolygonMesh& _mesh);
    void readOBJMesh(const std::string _fileName, PolygonMesh& _mesh);
    void writeOBJMesh(const std::string _fileName, PolygonMesh& _mesh);

    void exportIndices (PointIndices& _indices, const std::string _fileName);
    void exportCamPerVtx (const std::string _fileName);

    // Other...
    void deleteWrongCameras(const std::string _fileName);

private:

    void removeOutliersFromCamPerVtx(PointIndices& _indices);

    std::vector<PointCloud<PointXYZRGBNormalCam>::Ptr > pointClouds_;
    unsigned int nClouds_;

    std::vector<Camera> cameras_;
    unsigned int nCameras_;

    std::vector<std::string> imageList_;

    std::vector<std::vector<int> > camPerVtx_;

};

#endif
