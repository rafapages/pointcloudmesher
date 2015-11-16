#include "pcMesher.h"

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

#include <boost/make_shared.hpp>
#include <boost/program_options.hpp>

typedef enum {ALL, POISSON} RunMode;

int main (int argc, char *argv[]){

    RunMode mode = ALL;

    // Parsing execution options
    try {

        namespace po = boost::program_options;

        po::options_description desc("Options");
        desc.add_options()
                ("help,h", "Print this help message")
                ("all,a", "Run the complete system")
                ("poisson,p", "Run just the poisson reconstruction given a point cloud")
                ;

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);

        if (vm.count("help")) {
            std::cerr << desc << std::endl;
            return 0;
        }


        // Either complete system or just poisson will run

        if (vm.count("poisson")) {
            std::cerr << "Only poisson reconstruction will now run." << std::endl;
            mode = POISSON;
        } else {
            std::cerr << "Complete system will now run" << std::endl;
            mode = ALL;
        }

    }

    catch(std::exception& e) {
        std::cerr << "error: " << e.what() << std::endl;
        std::cerr << "Run: " << argv[0] << " -h     if you need help" << std::endl;
        return 1;
    }

    catch(...) {
        std::cerr << "Exception of unknown type!" << std:: endl;
    }


    PcMesher cloud;


    // MODE = ALL

    if (mode == ALL){

        if ((argc != 3) && (argc != 4)){
            std::cerr << "Wrong number of input paremeters for complete mode" << std::endl;
            std::cerr << "Usage: " << argv[0] << " [-a] <bundlerfile.out> <image-list.txt>" << std::endl;
            return 1;
        }


        int index = 1;
        if (argc == 4){
            index = 2;
        }

        std::string namein(argv[index]);
        std::string listname(argv[index+1]);


        std::string nameout = namein.substr(0, namein.size()-4);


        // Reading input parameteres: blunder file and image list
        cloud.bundlerReader(namein);
        cloud.readImageList(listname);
        cloud.writeCameraSetupFile(nameout + "_cameras.txt");

        cloud.writeCloud(nameout + "_input.ply");

        // Statistical outlier removal
        cloud.removeAllOutliers();

        // Estimating dimensions
        Eigen::Vector3f dim = cloud.getDimensions(0);
        const float scale = cbrt(dim(0) * dim(1) * dim(2));
//        std::cerr << "scale: " << scale << std::endl;

        // We first estimate normals to get an initial orientation
//        cloud.estimateAllNormals(0.1);
        cloud.estimateAllNormals(scale*0.01);
        cloud.writeCloud(nameout + "_sinoutliers.ply");

        // Planes are segmented also using their normal info
        cloud.segmentPlanes();
        // Normals are estimated properly now (and their sense corrected)
//        cloud.estimateAllNormals(0.3);
        cloud.estimateAllNormals(scale*0.03);
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

//        cloud.assignCam2Mesh(m, combinedCloudPtr, nameout + "_meshcamera.txt");
        PolygonMesh ms = cloud.smoothMeshLaplacian(m);
        PolygonMesh simpleM = cloud.decimateMesh(ms);

        io::savePLYFile(nameout + "_poisson_limpio.ply", m);
        io::savePLYFile(nameout + "_poisson_limpio_smooth.ply", ms);
        io::savePLYFile(nameout + "_poisson_limpio_smooth_decimated.ply", simpleM);
        cloud.writeOBJMesh(nameout + ".obj", simpleM);

        cloud.assignCam2Mesh(ms, combinedCloudPtr, nameout + "_meshcamera.txt");
        cloud.assignCam2Mesh(simpleM, combinedCloudPtr, nameout + "_limpio_decimated_meshcamera.txt");

        //    cloud.drawCameras();

        cloud.writeCloud(nameout + "_output.ply");
    }


    //------------------------------------------------------------------------
    // Just Poisson
    //------------------------------------------------------------------------

    if (mode == POISSON){

        if (argc != 3){
            std::cerr << "Wrong number of input paremeters" << std::endl;
            std::cerr << "Usage: " << argv[0] << " -p <pointCloud.ply>" << std::endl;
            return 1;
        }


        // Reading input parameteres: blunder file and image list
        cloud.readPLYCloud(argv[2]);

        PolygonMesh first_mesh = cloud.surfaceReconstruction(cloud.getPointCloudPtr(0));
        io::savePLYFile("poisson_v2.ply", first_mesh);

        PolygonMesh m = cloud.deleteWrongVertices(cloud.getPointCloudPtr(0), first_mesh);
        PolygonMesh ms = cloud.smoothMeshLaplacian(m);

        io::savePLYFile("poisson_clean_smooth.ply", ms);
        cloud.writeOBJMesh("poisson_clean_smooth.obj", ms);

    }


    return 0;



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
