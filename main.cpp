#include "pcMesher.h"

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>


int main (int argc, char *argv[]){

    if (argc != 3){
        std::cerr << "Wrong number of input paremeters" << std::endl;
        std::cerr << "Usage: " << argv[0] << " <bundlerfile.out> <image-list.txt>" << std::endl;
        return 0;
    }

    PcMesher cloud;

    // Reading input parameteres: blunder file and image list
    cloud.bundlerReader(argv[1]);
    cloud.readImageList(argv[2]);
    cloud.writeCameraSetupFile("newversioncameras.txt");

    cloud.writeMesh("input.ply");

    cloud.removeAllOutliers();

    // We first estimate normals to get an initial orientation
    cloud.estimateAllNormals(0.1);
    cloud.writeMesh("sinoutliers.ply");

    // Planes are segmented also using their normal info
    cloud.segmentPlanes();
    // Normals are estimated properly now (and their sense corrected)
    cloud.estimateAllNormals(0.3);
    cloud.fixAllNormals();

    // "Planar" point clouds are collected into a single point cloud
    std::vector<PointCloud<PointXYZRGBNormalCam>::Ptr> pointclouds;
    for (unsigned int i = 1; i < cloud.getNClouds(); i++){
        pointclouds.push_back(cloud.getPointCloudPtr(i));
    }

    PointCloud<PointXYZRGBNormalCam> combinedCloud = cloud.combinePointClouds(pointclouds);
    PointCloud<PointXYZRGBNormalCam>::Ptr combinedCloudPtr = boost::make_shared<PointCloud<PointXYZRGBNormalCam> >(combinedCloud);
    io::savePLYFile("combined_planes.ply", combinedCloud);


    PolygonMesh first_mesh = cloud.surfaceReconstruction(combinedCloudPtr);
    io::savePLYFile("poisson.ply", first_mesh);

    PolygonMesh m = cloud.deleteWrongVertices(combinedCloudPtr, first_mesh);
    PolygonMesh simpleM = cloud.decimateMesh(m);

    io::savePLYFile("poisson_limpio.ply", m);
    io::savePLYFile("poisson_limpio_decimated.ply", simpleM);

    cloud.assignCam2Mesh(m, combinedCloudPtr, "meshcamera.txt");

//    cloud.drawCameras();

    cloud.writeMesh("output.ply");

    return 0;

//    //------------------------------------------------------------------------
//    // Just Poisson
//    //------------------------------------------------------------------------

//    if (argc != 2){
//        std::cerr << "Wrong number of input paremeters" << std::endl;
//        return 0;
//    }

//    PcMesher cloud;

//    // Reading input parameteres: blunder file and image list
//    cloud.readMesh(argv[1]);

//    PolygonMesh first_mesh = cloud.surfaceReconstruction(cloud.getPointCloudPtr(0));
//    io::savePLYFile("poisson_v2.ply", first_mesh);

//    PolygonMesh m = cloud.deleteWrongVertices(cloud.getPointCloudPtr(0), first_mesh);

//    io::savePLYFile("poisson_limpio_v2.ply", m);

//    return 0;

}
