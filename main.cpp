#include "pcMesher.h"

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/geometry/mesh_conversion.h>


#include <boost/make_shared.hpp>
#include <boost/program_options.hpp>
#include <boost/program_options/value_semantic.hpp>

typedef enum {ALL, SPLASH, NO_NORMAL, POISSON, FIXLIST, MESH} RunMode;

int main (int argc, char *argv[]){

    RunMode mode = ALL;
    float decimationPrc = 0.0;


    // Parsing execution options
    try {

        namespace po = boost::program_options;

        po::options_description desc("Options");
        desc.add_options()
                ("help,h", "Print this help message")
                ("all,a", "Runs the complete system")
                ("splash,s", "Runs the complete system and setups the data for splash model creation")
                ("normal,n", "Runs the system but loads a point cloud where normals have already been roughly estimated")
                ("poisson,p", "Runs just the poisson reconstruction given a point cloud")
                ("meshtoclean,m", "Loads a mesh and a point cloud and perform operations on the mesh")
                ("fixlist,l", "Fixes the image list given another list with cameras with wrong texture")
                ("decimation", po::value<int>()->default_value(90), "Decimation percentage [0-99%]")
                ;

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);

        if (vm.count("help")) {
            std::cerr << desc << std::endl;
            return 0;
        }


        // Either complete system or just poisson will run
        if (vm.count("normal")){
            std::cerr << "Normals will be loaded from input point cloud" << std::endl;
            mode = NO_NORMAL;
        } else if (vm.count("poisson")){
            std::cerr << "Only poisson reconstruction will now run." << std::endl;
            mode = POISSON;
        } else if (vm.count("meshtoclean")){
            std::cerr << "Mesh will be cleaned, smoothed and decimated" << std::endl;
            mode = MESH;
        } else if (vm.count("fixlist")){
            std::cerr << "Fixing the image list..." << std::endl;
            mode = FIXLIST;
        } else if (vm.count("splash")){
            std::cerr << "Splash model creation system will now run..." << std::endl;
            mode = SPLASH;
        } else {
            std::cerr << "Complete system will now run" << std::endl;
            mode = ALL;
        }

        if (vm.count("decimation")){
            float decim = (float) vm["decimation"].as<int>();
            decimationPrc = 1 - decim * 0.01;
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
    std::string nameout;
    PointXYZRGBNormalCam normal;


    float scale = 0.0;

    //------------------------------------------------------------------------
    // Complete system
    //------------------------------------------------------------------------

    if (mode == ALL){

//        if ((argc != 3) && (argc != 4)){
        if (argc < 3 || argc > 6) {
            std::cerr << "Wrong number of input paremeters for complete mode" << std::endl;
            std::cerr << "Usage: " << argv[0] << " [-a] [--decimation=<value>] <bundlerfile.out> <image-list.txt>" << std::endl;
            return 1;
        }


//        int index = 1;
//        if (argc == 4){
//            index = 2;
//        }

        int index = argc - 2;

        std::string namein(argv[index]);
        std::string listname(argv[index+1]);


        nameout = namein.substr(0, namein.size()-4);


        // Reading input parameteres: blunder file and image list
        cloud.bundlerReader(namein);
        cloud.readImageList(listname);
        cloud.writeCameraSetupFile(nameout + "_cameras.txt");

        // We get the plane defined by the cameras
        cloud.getPlaneDefinedByCameras(normal);

        cloud.writeCloud(nameout + "_input.ply");

        // Statistical outlier removal
        cloud.removeAllOutliers();

        // Estimating dimensions
        Eigen::Vector3f dim = cloud.getDimensions(0);
        scale = cbrt(dim(0) * dim(1) * dim(2));
        std::cerr << "scale: " << scale << std::endl;

        // We first estimate normals to get an initial orientation
//        cloud.estimateAllNormals(scale*0.01);
        cloud.estimateAllNormals(scale*0.005);
        cloud.writeCloud(nameout + "_no_outliers.ply");
    }

    //------------------------------------------------------------------------
    // In this case normals are not calculated because they are preloaded
    //------------------------------------------------------------------------

    if (mode == NO_NORMAL){
//        if (argc != 5){
        if (argc < 5 || argc > 7){
            std::cerr << "Wrong number of input paremeters for no-normal mode" << std::endl;
            std::cerr << "Usage: " << argv[0] << " -n [--decimation=<value>] <bundlerfile.out> <image-list.txt> <pointcloudwithnormals.ply>" << std::endl;
            return 1;
        }

        int index = argc - 3;

        std::string namein(argv[index]);
        std::string listname(argv[index+1]);
        std::string pcname(argv[index+2]);

        nameout = namein.substr(0, namein.size()-4);


        // Reading input parameteres: blunder file and image list
        cloud.bundlerReader(namein); // Bundler file is read but point cloud info, discarded
        cloud.readImageList(listname);
        cloud.writeCameraSetupFile(nameout + "_cameras.txt");
        cloud.clearPointClouds();

        cloud.readPLYCloud(pcname);

        // We get the plane defined by the cameras
        cloud.getPlaneDefinedByCameras(normal);

        // Estimating dimensions
        Eigen::Vector3f dim = cloud.getDimensions(0);
        scale = cbrt(dim(0) * dim(1) * dim(2));
        std::cerr << "scale: " << scale << std::endl;


    }

    if (mode == ALL || mode == NO_NORMAL){


        // Planes are segmented also using their normal info
//        cloud.segmentPlanes(scale*0.005);
        cloud.segmentPlanes(scale*0.001);
        // Normals are estimated properly now (and their sense corrected)
//        cloud.estimateAllNormals(scale*0.01);
        cloud.estimateAllNormals(scale*0.01);
        cloud.fixAllNormals();

        // "Planar" point clouds are collected into a single point cloud
        std::vector<PointCloud<PointXYZRGBNormalCam>::Ptr> pointclouds;      

        if (mode == SPLASH){
            nameout += "-s";
            for (unsigned int i = 1; i < cloud.getNClouds(); i++){
                pointclouds.push_back(cloud.getPointCloudPtr(i));
            }
        } else {
            nameout += "-a";
            for (unsigned int i = 0; i < cloud.getNClouds(); i++){
                pointclouds.push_back(cloud.getPointCloudPtr(i));
            }
        }

        PointCloud<PointXYZRGBNormalCam> combinedCloud = cloud.combinePointClouds(pointclouds);
        PointCloud<PointXYZRGBNormalCam>::Ptr combinedCloudPtr = boost::make_shared<PointCloud<PointXYZRGBNormalCam> >(combinedCloud);
        io::savePLYFile(nameout + "_combined_planes.ply", combinedCloud);


        PolygonMesh first_mesh = cloud.surfaceReconstruction(combinedCloudPtr);
        io::savePLYFile(nameout + "_poisson.ply", first_mesh);

        Mesh polyMesh;
        pcl::geometry::toHalfEdgeMesh(first_mesh, polyMesh); // IMPORTANT!! this is how the conversion is done!

        cloud.detectLargestComponent(polyMesh);

        PointCloud<PointXYZRGBNormalCam>::Ptr sampledPtr (new PointCloud<PointXYZRGBNormalCam>);
        cloud.downSample(cloud.getPointCloudPtr(0), sampledPtr);

	cloud.openHole(polyMesh, normal);
        cloud.cleanOpenMesh(sampledPtr, polyMesh);
	cloud.detectLargestComponent(polyMesh);
	cloud.openHole(polyMesh, normal);
        cloud.cleanOpenMesh(sampledPtr, polyMesh);
	cloud.detectLargestComponent(polyMesh);
        

        PolygonMesh m;
        pcl::geometry::toFaceVertexMesh(polyMesh, m);
//        PolygonMesh m = cloud.deleteWrongVertices(combinedCloudPtr, first_mesh);

//        cloud.assignCam2Mesh(m, combinedCloudPtr, nameout + "_meshcamera.txt");
        PolygonMesh ms = cloud.smoothMeshLaplacian(m);
        PolygonMesh simpleM = cloud.decimateMesh(ms, decimationPrc);

        io::savePLYFile(nameout + "_poisson_clean.ply", m);
        io::savePLYFile(nameout + "_poisson_clean_smooth.ply", ms);
        io::savePLYFile(nameout + "_poisson_clean_smooth_decimated.ply", simpleM);
        cloud.writeOBJMesh(nameout + ".obj", simpleM);

        cloud.assignCam2Mesh(ms, combinedCloudPtr, nameout + "_meshcamera.txt");
        cloud.assignCam2Mesh(simpleM, combinedCloudPtr, nameout + "_clean_decimated_meshcamera.txt");

        //    cloud.drawCameras();

        cloud.writeCloud(nameout + "_output.ply");
    }


    //------------------------------------------------------------------------
    // Just Poisson
    //------------------------------------------------------------------------

    if (mode == POISSON){

//        if (argc != 4){
        if (argc < 4 || argc > 6){
            std::cerr << "Wrong number of input paremeters" << std::endl;
            std::cerr << "Usage: " << argv[0] << " -p [--decimation=<value>] <pointCloud.ply> <bundlermodel.out>" << std::endl;
            return 1;
        }

        // Reading input parameteres: blunder file and image list
        int index = argc - 2;
        cloud.readPLYCloud(argv[index]);
        cloud.bundlerReadOnlyCameraInfo(argv[index+1]);


        cloud.getPlaneDefinedByCameras(normal);

//        // To fix the normals!!
//        cloud.bundlerReader(argv[3]);
//        cloud.fixAllNormals();

        PolygonMesh first_mesh = cloud.surfaceReconstruction(cloud.getPointCloudPtr(0));
        io::savePLYFile("poisson_v2.ply", first_mesh);

        Mesh polyMesh;
        pcl::geometry::toHalfEdgeMesh(first_mesh, polyMesh); // IMPORTANT!! this is how the conversion is done!

        cloud.detectLargestComponent(polyMesh);
        cloud.openHole(polyMesh, normal);


        PointCloud<PointXYZRGBNormalCam>::Ptr sampledPtr (new PointCloud<PointXYZRGBNormalCam>);

        cloud.downSample(cloud.getPointCloudPtr(0), sampledPtr);
        cloud.cleanOpenMesh(sampledPtr, polyMesh);
        cloud.detectLargestComponent(polyMesh);

        PolygonMesh m;
        pcl::geometry::toFaceVertexMesh(polyMesh, m);


//        PolygonMesh m = cloud.deleteWrongVertices(cloud.getPointCloudPtr(0), first_mesh);
        io::savePLYFile("poisson_clean.ply", m);

        PolygonMesh ms = cloud.smoothMeshLaplacian(m);
        PolygonMesh mss = cloud.decimateMesh(ms, decimationPrc);

        io::savePLYFile("poisson_clean_smooth.ply", ms);

        io::savePLYFile("poisson_clean_smooth_decim.ply", mss);
        cloud.writeOBJMesh("poisson_clean_smooth_decim.obj", mss);

    }


    //---------------------------------------------------------------------------
    // This one loads a mesh and cleans, smooths and decimates it
    //---------------------------------------------------------------------------

    if (mode == MESH){


//        if (argc != 5){
        if (argc < 5 || argc > 7){
            std::cerr << "Wrong number of input paremeters for mesh cleaning mode" << std::endl;
            std::cerr << "Usage: " << argv[0] << " -m  [--decimation=<value>] <meshtoclean.ply> <pointcloudreference.ply> <bundlermodel.out>" << std::endl;
            return 1;
        }

        PolygonMesh mesh;

        int index = argc - 3;
        cloud.readPLYMesh(argv[index], mesh);
        cloud.readPLYCloud(argv[index+1]);
        cloud.bundlerReadOnlyCameraInfo(argv[index+2]);


        cloud.getPlaneDefinedByCameras(normal);

        Mesh polyMesh;
        pcl::geometry::toHalfEdgeMesh(mesh, polyMesh); // IMPORTANT!! this is how the conversion is done!

        cloud.detectLargestComponent(polyMesh);
        cloud.openHole(polyMesh, normal);


        PointCloud<PointXYZRGBNormalCam>::Ptr sampledPtr (new PointCloud<PointXYZRGBNormalCam>);

        cloud.downSample(cloud.getPointCloudPtr(0), sampledPtr);
        cloud.cleanOpenMesh(sampledPtr, polyMesh);
        cloud.detectLargestComponent(polyMesh);


        PolygonMesh m;
        pcl::geometry::toFaceVertexMesh(polyMesh, m);

//        PolygonMesh m = cloud.deleteWrongVertices(cloud.getPointCloudPtr(0), mesh);

        io::savePLYFile("mesh_clean.ply", m);

        PolygonMesh ms = cloud.smoothMeshLaplacian(m);
        PolygonMesh mss = cloud.decimateMesh(ms, decimationPrc);

        io::savePLYFile("mesh_clean_smooth.ply", ms);

        io::savePLYFile("mesh_clean_smooth_decim.ply", mss);
        cloud.writeOBJMesh("mesh_clean_smooth_decim.obj", mss);


    }

    //---------------------------------------------------------------------------
    // To fix the image list
    //---------------------------------------------------------------------------

    if (mode == FIXLIST){

        if (argc != 5){
            std::cerr << "Wrong number of input paremeters for fix-list mode" << std::endl;
            std::cerr << "Usage: " << argv[0] << " -l <bundlerfile.out> <image-list.txt> <wrongcameras.txt>" << std::endl;
            return 1;
        }


        PcMesher cloud;

        cloud.bundlerReader(argv[2]);
        cloud.readImageList(argv[3]);
        cloud.deleteWrongCameras(argv[4]);
        cloud.writeCameraSetupFile("clean_camera_setup.txt");

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

}
