#include "pcMesher.h"

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

#include <boost/make_shared.hpp>

int main (int argc, char *argv[]){

    if (argc != 3){
        std::cerr << "Wrong number of input paremeters" << std::endl;
        std::cerr << "Usage: " << argv[0] << " <bundlerfile.out> <image-list.txt>" << std::endl;
        return 0;
    }

    std::string namein(argv[1]);
    std::string nameout = namein.substr(0, namein.size()-4);

    PcMesher cloud;

    // Reading input parameteres: blunder file and image list
    cloud.bundlerReader(argv[1]);
    cloud.readImageList(argv[2]);
    cloud.writeCameraSetupFile(nameout + "_cameras.txt");
//    cloud.readCloud(argv[1]);

    cloud.writeCloud(nameout + "_input.ply");

    // Statistical outlier removal
    cloud.removeAllOutliers();

    // We first estimate normals to get an initial orientation
    cloud.estimateAllNormals(0.1);
    cloud.writeCloud(nameout + "_sinoutliers.ply");

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
    io::savePLYFile(nameout + "_combined_planes.ply", combinedCloud);


    PolygonMesh first_mesh = cloud.surfaceReconstruction(combinedCloudPtr);
    io::savePLYFile(nameout + "_poisson.ply", first_mesh);

    PolygonMesh m = cloud.deleteWrongVertices(combinedCloudPtr, first_mesh);

    cloud.assignCam2Mesh(m, combinedCloudPtr, nameout + "_meshcamera.txt");
    PolygonMesh ms = cloud.smoothMeshLaplacian(m);
    PolygonMesh simpleM = cloud.decimateMesh(ms);

    io::savePLYFile(nameout + "_poisson_limpio.ply", m);
    io::savePLYFile(nameout + "_poisson_limpio_smooth.ply", ms);
    io::savePLYFile(nameout + "_poisson_limpio_smooth_decimated.ply", simpleM);

    cloud.assignCam2Mesh(ms, combinedCloudPtr, nameout + "_meshcamera.txt");
    cloud.assignCam2Mesh(simpleM, combinedCloudPtr, nameout + "_limpio_decimated_meshcamera.txt");

//    cloud.drawCameras();

    cloud.writeCloud(nameout + "_output.ply");

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
//    cloud.readPLYCloud(argv[1]);

//    PolygonMesh first_mesh = cloud.surfaceReconstruction(cloud.getPointCloudPtr(0));
//    io::savePLYFile("poisson_v2.ply", first_mesh);

//    PolygonMesh m = cloud.deleteWrongVertices(cloud.getPointCloudPtr(0), first_mesh);

//    io::savePLYFile("poisson_limpio_v2.ply", m);

//    return 0;

//    //------------------------------------------------------------------------
//    // Just Poisson smooth
//    //------------------------------------------------------------------------
//    if (argc != 2){
//        std::cerr << "Wrong number of input paremeters" << std::endl;
//        return 0;
//    }

//    PcMesher cloud;
//    PolygonMesh mesh;
//    cloud.readPLYMesh(argv[1], mesh);
//    PolygonMesh smoothMes = cloud.smoothMeshLaplacian(mesh);

//    io::savePLYFile("smooth.ply", smoothMes);

//    return 0;


    //----------------------------------------------------------------------------
    // Just camera per vertex of the mesh
    //----------------------------------------------------------------------------

//    if (argc!=3){
//        std::cerr << "Wrong number of input parameters" << std::endl;
//        std::cerr << "Usage: " << argv[0] << " <bundlerFile.out> <meshFile.{ply,obj}>" << std::endl;
//        return 0;
//    }

//    PcMesher cloud;
//    PolygonMesh mesh;

//    cloud.bundlerReader(argv[1]);
//    cloud.removeAllOutliers();

//    std::string namein(argv[2]);
//    std::string nameout = namein.substr(0, namein.size()-4);
//    nameout += "_camvtx.txt";

//    cloud.readOBJMesh(argv[2], mesh);
//    cloud.assignCam2Mesh(mesh, cloud.getPointCloudPtr(0), nameout);

//    return 0;

//    //---------------------------------------------------------------------------
//    // Y otro...
//    //---------------------------------------------------------------------------

//    PcMesher cloud;
//    PolygonMesh mesh;

//    cloud.readPLYCloud(argv[1]);
////    cloud.readPLYMesh(argv[2], mesh);

//    PolygonMesh first_mesh = cloud.surfaceReconstruction(cloud.getPointCloudPtr(0));
//    io::savePLYFile("test_poisson.ply", first_mesh);
////    PolygonMesh m = cloud.deleteWrongVertices2(cloud.getPointCloudPtr(0), mesh);
//    PolygonMesh m = cloud.deleteWrongVertices2(cloud.getPointCloudPtr(0), first_mesh);

//    PolygonMesh simpleM = cloud.decimateMesh(m);

//    io::savePLYFile("test_poisson_limpio.ply", m);
//    io::savePLYFile("test_poisson_limpio_decimated.ply", simpleM);

//    return 0;

//    //---------------------------------------------------------------------------
//    // Para arreglar la lista de imágenes
//    //---------------------------------------------------------------------------

//    PcMesher cloud;

//    cloud.bundlerReader(argv[1]);
//    cloud.readImageList(argv[2]);
//    cloud.deleteWrongCameras(argv[3]);
//    cloud.writeCameraSetupFile("clean_camera_setup.txt");

//    return 0;

//    //---------------------------------------------------------------------------
//    // Para reconstruir la nube de puntos del skeletonScanner
//    //---------------------------------------------------------------------------

//    if (argc != 2){
//        std::cerr << "Wrong number of input parameters!" << std::endl;
//        std::cerr << "Usage: " << argv[0] << " <inputPointCloud.plu>" << std::endl;
//        return 0;
//    }

//    std::string namein(argv[1]);
//    std::string nameout = namein.substr(0, namein.size()-4);

//    PcMesher cloud;
//    cloud.readPLYCloud(namein);

//    PointCloud<PointXYZRGBNormalCam>::Ptr pc = cloud.getPointCloudPtr(0);
//    std::cerr << "Number of points: " << pc->height * pc->width << std::endl;

////    cloud.removeAllOutliers();
////    cloud.writeCloud(nameout + "_sinoutliers.ply");

//    cloud.estimateAllNormals();
//    cloud.writeCloud(nameout + "_normals.ply");



//    return 0;




}
