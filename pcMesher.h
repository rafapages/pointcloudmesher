#ifndef PCMESHER_H
#define PCMESHER_H

#include <iostream>
#include <string>
#include <stdio.h>

#include <pcl/surface/poisson.h>
#include <pcl/geometry/polygon_mesh.h>


#include "pointXYZRGBNormalCam.h"
#include "camera.h"


typedef pcl::geometry::PolygonMesh <pcl::geometry::DefaultMeshTraits <PointXYZRGBNormalCam> > Mesh;


class PcMesher{

public:

    PcMesher();
    ~PcMesher();

    // Get number of clouds
    unsigned int getNClouds() const;

    // Get the vector of point-camera correspondance
    std::vector<std::vector<int> > getCamPerVtx() const;

    // Get a Ptr to a cloud
    PointCloud<PointXYZRGBNormalCam>::Ptr getPointCloudPtr(unsigned int _index);

    // Remove pointclouds
    void clearPointClouds();

    // Get dimensions of Point Cloud or Mesh
    Eigen::Vector3f getDimensions(const PointCloud<PointXYZRGBNormalCam>::Ptr& _cloud);
    Eigen::Vector3f getDimensions(const unsigned int _index);
    Eigen::Vector3f getDimensions(const PolygonMesh& _mesh);

    // Compute point cloud resolution
    double computeResolution(const PointCloud<PointXYZRGBNormalCam>::Ptr& _cloud) const;

    // Outlier filtering
    void removeOutliers(PointCloud<PointXYZRGBNormalCam>::Ptr& _cloud);
    void removeOutliers(unsigned int _index);
    void removeAllOutliers();

    // Downsample a pointcloud
    void downSample(const PointCloud<PointXYZRGBNormalCam>::Ptr& _cloud, PointCloud<PointXYZRGBNormalCam>::Ptr _outCloud);

    // Estimates the plane defined by the cameras
    void getPlaneDefinedByCameras(PointXYZRGBNormalCam& _normal) const;

    // Fit a plane in a point cloud given and return its normal
    void fitPlane(const std::vector<Eigen::Vector3f> _cloud, Eigen::Vector3f& _normal) const;


    // Estimate the normals of a cloud and store them
    void estimateNormals(const unsigned int _index);
    void estimateNormals(const unsigned int _index, const float _radius);
    void estimateAllNormals();
    void estimateAllNormals(const float _radius);

    // Fix the normal orientation using the camera position
    void fixNormal(const unsigned int _index);
    void fixAllNormals();

    // Plane segmentation
    void segmentPlanes(float _threshold);

    // Cilinder segmentation
    void segmentCylinders();

    // Surface reconstruction
    void surfaceReconstruction(const unsigned int _index);
    PolygonMesh surfaceReconstruction(PointCloud<PointXYZRGBNormalCam>::Ptr _cloud);
    void allSurfaceReconstruction();

    // Mesh operations
    void detectLargestComponent(Mesh& _inputMesh) const;
    bool isMeshOpen(const Mesh& _inputMesh) const;
    void openHole(Mesh& _inputMesh, const PointXYZRGBNormalCam& _normal) const;

    // Mesh refining
    void cleanOpenMesh(const PointCloud<PointXYZRGBNormalCam>::Ptr& _cloud, Mesh& _inputMesh) const;
    PolygonMesh deleteWrongVertices(PointCloud<PointXYZRGBNormalCam>::Ptr _cloud, PolygonMesh& _inputMesh);   
    PolygonMesh decimateMesh(const PolygonMesh& _mesh, float _reduction);
    PolygonMesh smoothMeshLaplacian(const PolygonMesh& _mesh);


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
    bool bundlerPointReader(PointXYZRGBNormalCam& _point, std::ifstream& _stream);
    void readPLYCloud(const std::string& _fileName);
    void writeOneCloud(const unsigned int _index, const std::string& _fileName);
    void writeCloud(const std::string& _fileName);
    void bundlerReader(const std::string& _fileName);
    void bundlerReadOnlyCameraInfo(const std::string& _fileName);
    void readImageList(const std::string& _fileName);

    void readPLYMesh(const std::string& _fileName, PolygonMesh& _mesh);
    void readOBJMesh(const std::string& _fileName, PolygonMesh& _mesh);
    void writeOBJMesh(const std::string& _fileName, PolygonMesh& _mesh);

    void exportIndices (PointIndices& _indices, const std::string& _fileName);
    void exportCamPerVtx (const std::string& _fileName);

    // Other...
    void deleteWrongCameras(const std::string& _fileName);

private:

    bool isPointCloseToPointCloud(const PointXYZRGBNormalCam& _point, const PointCloud<PointXYZRGBNormalCam>::Ptr _cloud, float _radius) const;

    void removeOutliersFromCamPerVtx(PointIndices& _indices);

    std::vector<PointCloud<PointXYZRGBNormalCam>::Ptr > pointClouds_;
    unsigned int nClouds_;

    std::vector<Camera> cameras_;
    unsigned int nCameras_;

    std::vector<std::string> imageList_;

    std::vector<std::vector<int> > camPerVtx_;

};

#endif
