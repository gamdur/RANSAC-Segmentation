#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <math.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/console/time.h>
#include <pcl/conversions.h>
#include <pcl/io/ply_io.h>

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;
using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

//convenient structure to handle our pointclouds
struct PCD
{
  pcl::PointCloud<PointT>::Ptr cloud;
  std::string f_name;

  PCD() : cloud (new pcl::PointCloud<pcl::PointXYZ>) {};
};

struct OUTPUT_CLOUD
{
  pcl::PointCloud<PointT>::Ptr cloud;
  pcl::PointIndices::Ptr inliers;
  pcl::ModelCoefficients::Ptr coefficients;

};

//our visualizer
pcl::visualization::PCLVisualizer *p;
int vp_1, vp_2; //its left and right viewports

void showTwoClouds(const PointCloud::Ptr cloud_source1, const PointCloud::Ptr cloud_source2)
// void showCloudsLeft(const PointCloud::Ptr cloud_source1)
{
  p->removePointCloud ("vp1_source");
  p->removePointCloud ("vp1_source2");

  PointCloudColorHandlerCustom<PointT> src_h (cloud_source1, 255, 0, 0);
  PointCloudColorHandlerCustom<PointT> src_h2 (cloud_source2, 0, 0, 255);

  p->addPointCloud (cloud_source1, src_h, "vp1_source", vp_1);
  p->addPointCloud (cloud_source2, src_h2, "vp1_source2", vp_1);

  p-> spin();
}

void showClouds(std::vector<pcl::PointCloud<PointT>::Ptr> clouds)
{
  for (int i=0; i<clouds.size(); i++) {
      string cloud_name = to_string(i);
      p->removePointCloud (cloud_name);
      int min = 0;
      int max = 255;
      int r = min + (rand() % static_cast<int>(max - min + 1));
      int g = min + (rand() % static_cast<int>(max - min + 1));
      int b = min + (rand() % static_cast<int>(max - min + 1));
      PointCloudColorHandlerCustom<PointT> src_h (clouds[i], r, g, b);
      p->addPointCloud (clouds[i], src_h, cloud_name, vp_1);
  }

  p-> spin();
}






void PCLPointToCloud2 (pcl::PointCloud<PointT>& cloud, pcl::PCLPointCloud2& msg) {
    // Ease the user's burden on specifying width/height for unorganized datasets
    if (cloud.width == 0 && cloud.height == 0) {
        msg.width  = static_cast<uint32_t>(cloud.points.size ());
        msg.height = 1;
    } else {
        assert (cloud.points.size () == cloud.width * cloud.height);
        msg.height = cloud.height;
        msg.width  = cloud.width;
    }

    // Fill point cloud binary data (padding and all)
    size_t data_size = sizeof (PointT) * cloud.points.size ();
    msg.data.resize (data_size);
    memcpy (&msg.data[0], &cloud.points[0], data_size);

    // Fill fields metadata
    msg.fields.clear ();
    pcl::for_each_type<typename pcl::traits::fieldList<PointT>::type> (pcl::detail::FieldAdder<PointT>(msg.fields));

    msg.header     = cloud.header;
    msg.point_step = sizeof (PointT);
    msg.row_step   = static_cast<uint32_t> (sizeof (PointT) * msg.width);
    msg.is_dense   = cloud.is_dense;
    /// @todo msg.is_bigendian = ?;
}






void loadData (int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD> > &models)
{
  std::string extension (".pcd");
  // Suppose the first argument is the actual test model
  for (int i = 1; i < argc; i++)
  {
    std::string fname = std::string (argv[i]);
    // Needs to be at least 5: .plot
    if (fname.size () <= extension.size ())
      continue;

    std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);

    //check that the argument is a pcd file
    if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)
    {
      // Load the cloud and saves it into the global list of models
      PCD m;
      m.f_name = argv[i];
      pcl::io::loadPCDFile (argv[i], *m.cloud);
      //remove NAN points from the cloud
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);

      models.push_back (m);
    } else {
        std::cout << "into a .pcd file" << std::endl;
    }
  }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr readCloud(int argc, char** argv) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<PCD, Eigen::aligned_allocator<PCD> > data;
    loadData (argc, argv, data);
    return data[0].cloud;
}

pcl::PointCloud<PointT>::Ptr filterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::PassThrough<PointT> pass;
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, 1.5);
    pass.filter (*cloud_filtered);
    std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;
    return cloud_filtered;
}

pcl::PointCloud<pcl::Normal>::Ptr extractNormals(pcl::PointCloud<PointT>::Ptr cloud_filtered) {
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_filtered);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);
    return cloud_normals;
}

void compute (pcl::PCLPointCloud2::Ptr &input, pcl::PolygonMesh &output,
         int hoppe_or_rbf, float iso_level, int grid_res, float extend_percentage, float off_surface_displacement) {
  pcl::PointCloud<pcl::PointNormal>::Ptr xyz_cloud (new pcl::PointCloud<pcl::PointNormal> ());
  fromPCLPointCloud2 (*input, *xyz_cloud);

  pcl::MarchingCubes<pcl::PointNormal> *mc;
  if (hoppe_or_rbf == 0)
    mc = new pcl::MarchingCubesHoppe<pcl::PointNormal> ();
  else
  {
    mc = new pcl::MarchingCubesRBF<pcl::PointNormal> ();
    (reinterpret_cast<pcl::MarchingCubesRBF<pcl::PointNormal>*> (mc))->setOffSurfaceDisplacement (off_surface_displacement);
  }

  mc->setIsoLevel (iso_level);
  mc->setGridResolution (grid_res, grid_res, grid_res);
  mc->setPercentageExtendGrid (extend_percentage);
  mc->setInputCloud (xyz_cloud);

  pcl::console::TicToc tt;
  tt.tic ();

  cout << "Computing " << endl;
  mc->reconstruct (output);
  delete mc;

  cout << "[done, " << tt.toc () << " ms]" << endl;
}

bool loadCloud2 (const std::string &filename, pcl::PCLPointCloud2 &cloud) {
    pcl::console::TicToc tt;
    cout << "Loading " << filename.c_str () << endl;

    tt.tic ();
    if (pcl::io::loadPLYFile (filename, cloud) < 0)
      return (false);
    int num_points = cloud.width * cloud.height;
    cout << "[done, " << tt.toc () << " ms : " << num_points << " points]" << endl;
    cout << "Available dimensions: " << pcl::getFieldsList (cloud).c_str () << endl;

    return (true);
}


void marchingCubes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1 (new pcl::search::KdTree<pcl::PointXYZ>);
    tree1->setInputCloud (cloud);
    ne.setInputCloud (cloud);
    ne.setSearchMethod (tree1);
    ne.setKSearch (20);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    ne.compute (*normals);

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    concatenateFields(*cloud, *normals, *cloud_with_normals);

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>);
    tree->setInputCloud (cloud_with_normals);

    cout << "begin marching cubes reconstruction" << endl;

    pcl::MarchingCubesRBF<pcl::PointNormal> mc;
    cout << 1 << endl;
    pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);
    cout << 2 << endl;
    mc.setInputCloud (cloud_with_normals);
    cout << 3 << endl;
    mc.setSearchMethod (tree);
    cout << 4 << endl;
    mc.reconstruct (*triangles);

    cout << triangles->polygons.size() << " triangles created" << endl;
}

void clusterComponents (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {


    // // Estimate Normals
    // pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
    // ne.setInputCloud (cloud);
    // ne.compute (*normal_cloud);
    // float* distance_map = ne.getDistanceMap ();
    // boost::shared_ptr<pcl::EdgeAwarePlaneComparator<PointT,pcl::Normal> > eapc = boost::dynamic_pointer_cast<pcl::EdgeAwarePlaneComparator<PointT,pcl::Normal> >(edge_aware_comparator_);
    // eapc->setDistanceMap (distance_map);
    // eapc->setDistanceThreshold (0.01f, false);
    //
    // // Segment Planes
    // double mps_start = pcl::getTime ();
    // std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
    // std::vector<pcl::ModelCoefficients> model_coefficients;
    // std::vector<pcl::PointIndices> inlier_indices;
    pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
    // std::vector<pcl::PointIndices> label_indices;
    // std::vector<pcl::PointIndices> boundary_indices;
    // mps.setInputNormals (normal_cloud);
    // mps.setInputCloud (cloud);
    // mps.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);


    //Segment Objects
    // std::vector<pcl::PointIndices> label_indices;
    pcl::EuclideanClusterComparator<PointT,  pcl::Normal, pcl::Label>::Ptr euclidean_cluster_comparator_ = pcl::EuclideanClusterComparator<PointT,  pcl::Normal, pcl::Label>::Ptr (new pcl::EuclideanClusterComparator<PointT,  pcl::Normal, pcl::Label> ());
    pcl::PointCloud<PointT>::CloudVectorType clusters;

    // boost::shared_ptr<std::set<uint32_t> > plane_labels = boost::make_shared<std::set<uint32_t> > ();
    // for (size_t i = 0; i < label_indices.size (); ++i)
    //   if (label_indices[i].indices.size () > 10000)
    //     plane_labels->insert (i);

    euclidean_cluster_comparator_->setInputCloud (cloud);
    euclidean_cluster_comparator_->setLabels (labels);
    // euclidean_cluster_comparator_->setExcludeLabels (plane_labels);
    euclidean_cluster_comparator_->setDistanceThreshold (0.01f, false);

    pcl::PointCloud<pcl::Label> euclidean_labels;
    std::vector<pcl::PointIndices> euclidean_label_indices;
    pcl::OrganizedConnectedComponentSegmentation<PointT,pcl::Label> euclidean_segmentation (euclidean_cluster_comparator_);
    euclidean_segmentation.setInputCloud (cloud);
    euclidean_segmentation.segment (euclidean_labels, euclidean_label_indices);

    // for (size_t i = 0; i < euclidean_label_indices.size (); i++) {
    //     if (euclidean_label_indices[i].indices.size () > 1000)
    //     {
    //       pcl::PointCloud<PointT> cluster;
    //       pcl::copyPointCloud (*cloud,euclidean_label_indices[i].indices,cluster);
    //       clusters.push_back (cluster);
    //     }
    // }
    //
    // PCL_INFO ("Got %d euclidean clusters!\n", clusters.size ());
}

OUTPUT_CLOUD segmentPlane(pcl::PointCloud<PointT>::Ptr cloud_filtered, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, bool verbal) {
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    pcl::ExtractIndices<PointT> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);

    // Create the segmentation object for the planar model and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight (0.1);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);
    seg.setInputCloud (cloud_filtered);
    seg.setInputNormals (cloud_normals);
    // Obtain the plane inliers and coefficients
    seg.segment (*inliers_plane, *coefficients_plane);
    if (verbal) std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

    // Extract the planar inliers from the input cloud
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers_plane);
    extract.setNegative (false);

    // Write the planar inliers to disk
    pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
    extract.filter (*cloud_plane);
    if (verbal) std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
    std::cerr << endl;

    OUTPUT_CLOUD *output = new OUTPUT_CLOUD;
    output->cloud = cloud_plane;
    output->inliers = inliers_plane;
    output->coefficients = coefficients_plane;
    // {cloud_plane, inliers_plane};
    return *output;
}

OUTPUT_CLOUD segmentCylinder(pcl::PointCloud<PointT>::Ptr cloud_filtered2, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2, bool verbal) {
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    pcl::ExtractIndices<PointT> extract;
    pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.05);
    // seg.setDistanceThreshold (0.04);
    // seg.setRadiusLimits (0, 0.1);
    seg.setRadiusLimits (0, 10.0);
    // seg.setRadiusLimits (0.03, 0.1);
    seg.setInputCloud (cloud_filtered2);
    seg.setInputNormals (cloud_normals2);

    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers_cylinder, *coefficients_cylinder);
    if (verbal) std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

    // Write the cylinder inliers to disk
    extract.setInputCloud (cloud_filtered2);
    extract.setIndices (inliers_cylinder);
    extract.setNegative (false);
    pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
    extract.filter (*cloud_cylinder);
    if (cloud_cylinder->points.empty ()) {
        if (verbal) std::cerr << "Can't find the cylindrical component." << std::endl;
    } else {
    	  if (verbal) std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
    }

    OUTPUT_CLOUD output = {cloud_cylinder, inliers_cylinder, coefficients_cylinder};
    return output;
}

bool isPlane(pcl::PointCloud<PointT>::Ptr cloud) {
    float threshold = 0.2;
    // float threshold = 0.45;

    // for (int i=0; i<clouds.size(); i++) {
        // OUTPUT_CLOUD merged = mergeClouds(clouds[i], output);
    // pcl::PointCloud<PointT>::Ptr merged_cloud = merged.cloud;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals = extractNormals(cloud);

    OUTPUT_CLOUD segmented = segmentPlane(cloud, cloud_normals, false);

    cout << "RATIO CYLINDER: " << (float) segmented.cloud->points.size() / (cloud->points.size()) << endl;

    if (segmented.cloud->points.size() >= threshold * (cloud->points.size())) {
        cout << "IS PLANE" << endl;
        return true;
    }

    // }
    return false;
}

void removeCloud(pcl::PointCloud<PointT>::Ptr cloud_filtered, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, OUTPUT_CLOUD output) {
    pcl::ExtractIndices<PointT> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;

    extract.setNegative (true);
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (output.inliers);
    extract.filter (*cloud_filtered);

    extract_normals.setNegative (true);
    extract_normals.setInputCloud (cloud_normals);
    extract_normals.setIndices (output.inliers);
    extract_normals.filter (*cloud_normals);
}

OUTPUT_CLOUD mergeClouds(OUTPUT_CLOUD cloud1, OUTPUT_CLOUD cloud2) {
    pcl::PointCloud<PointT> cloud3 = *cloud1.cloud;
    cloud3 += *cloud2.cloud;
    OUTPUT_CLOUD result = {cloud3.makeShared(), cloud1.inliers, cloud2.coefficients};
    return result;
}

float norm(pcl::ModelCoefficients::Ptr coefficients1, pcl::ModelCoefficients::Ptr coefficients2) {
    float sqr_sum = 0;
    for (int i=0; i<coefficients1->values.size(); i++) {
        sqr_sum += pow(coefficients1->values[i] - coefficients2->values[i], 2);
    }
    return sqrt(sqr_sum);
}

std::vector<pcl::PointCloud<PointT>::Ptr> mergeCloudArrays(std::vector<OUTPUT_CLOUD> clouds1, std::vector<OUTPUT_CLOUD> clouds2) {
    std::vector<pcl::PointCloud<PointT>::Ptr> *clouds = new std::vector<pcl::PointCloud<PointT>::Ptr>();

    for (int i=0; i<clouds1.size(); i++) {
        clouds->push_back(clouds1[i].cloud);
    }

    for (int i=0; i<clouds2.size(); i++) {
        clouds->push_back(clouds2[i].cloud);
    }

    return *clouds;
}

bool shouldMerge(std::vector<OUTPUT_CLOUD> clouds, OUTPUT_CLOUD output, string shape) {
    float threshold = 0.79;
    for (int i=0; i<clouds.size(); i++) {
        OUTPUT_CLOUD merged = mergeClouds(clouds[i], output);
        pcl::PointCloud<PointT>::Ptr merged_cloud = merged.cloud;
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals = extractNormals(merged_cloud);

        OUTPUT_CLOUD segmented;
        if (shape == "cylinder") {
          segmented = segmentCylinder(merged_cloud, cloud_normals, false);
        } else {
          segmented = segmentPlane(merged_cloud, cloud_normals, false);
        }

        cout << "ratio: " << (float) segmented.cloud->points.size() / (clouds[i].cloud->points.size() + output.cloud->points.size()) << endl;

        if (segmented.cloud->points.size() >= threshold * (clouds[i].cloud->points.size() + output.cloud->points.size())) {
            clouds.erase (clouds.begin()+i);
            clouds.push_back(merged);
            cout << "Merged " << shape << endl;
            return true;
        }

    }
    return false;
}

int main (int argc, char** argv)
{

  cout << "-----------------------------------------------------------------------" << endl;
  cout << "-----------------------------------------------------------------------" << endl;




  // pcl::PCLPointCloud2::Ptr cloud2 (new pcl::PCLPointCloud2);
  // std::string file = "Tango/pillar_short_2.ply";
  // loadCloud2(file, *cloud2);
  // pcl::PolygonMesh output;
  // float iso_level = 0.0f;
  // int hoppe_or_rbf = 1;
  // float extend_percentage = 0.0f;
  // int grid_res = 50;
  // float off_surface_displacement = 0.01f;
  // compute (cloud2, output, hoppe_or_rbf, iso_level, grid_res, extend_percentage, off_surface_displacement);
  // cout << "mc size: " << output.polygons.size() << endl;





  // initialize visualizer
  p = new pcl::visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration example");
  p->createViewPort (0.0, 0, 1.0, 1.0, vp_1);

  // Read cloud
  pcl::PointCloud<PointT>::Ptr cloud = readCloud(argc, argv);

  // Filter cloud
  // pcl::PointCloud<PointT>::Ptr cloud_filtered = filterCloud(cloud);
  pcl::PointCloud<PointT>::Ptr cloud_filtered = cloud;

  // Estimate point normals
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals = extractNormals(cloud_filtered);

  std::vector<OUTPUT_CLOUD> planeClouds;
  std::vector<OUTPUT_CLOUD> cylinderClouds;
  while (true) {
      OUTPUT_CLOUD output1 = segmentPlane(cloud_filtered, cloud_normals, true);
      OUTPUT_CLOUD output2 = segmentCylinder(cloud_filtered, cloud_normals, true);

      cout << "-----------------------------------------------------------------------" << endl;

      if (output1.cloud->points.size() > output2.cloud->points.size()) {
          removeCloud(cloud_filtered, cloud_normals, output1);
          if (!shouldMerge(planeClouds, output1, "plane")) {
              planeClouds.push_back(output1);
          }
      } else {
          removeCloud(cloud_filtered, cloud_normals, output2);
          if (!shouldMerge(cylinderClouds, output2, "cylinder")) {
              if (!isPlane(output2.cloud)) {
                  cylinderClouds.push_back(output2);
              }

              // marchingCubes(output2.cloud);

              // pcl::PolygonMesh output;
              // float iso_level = 0.0f;
              // int hoppe_or_rbf = 1;
              // float extend_percentage = 0.0f;
              // int grid_res = 50;
              // float off_surface_displacement = 0.01f;
              // pcl::PCLPointCloud2::Ptr point_cloud2(new pcl::PCLPointCloud2);
              // PCLPointToCloud2(*output2.cloud, *point_cloud2);
              // compute (point_cloud2, output, hoppe_or_rbf, iso_level, grid_res, extend_percentage, off_surface_displacement);
              // cout << "mc size: " << output.polygons.size() << endl;

              // clusterComponents(output2.cloud);
          }
      }

      int THRESHOLD = 500;
      int size1 = output1.cloud->points.size();
      int size2 = output2.cloud->points.size();
      if (size1 < THRESHOLD && size2 < THRESHOLD) break;

      cout << "cloud size: " << cloud_filtered->points.size() << endl;
  }

  //cout << clouds.size() << endl;

  // visulaize clouds
  // showTwoClouds(clouds[0], clouds[1]);
  std::vector<pcl::PointCloud<PointT>::Ptr> clouds = mergeCloudArrays(planeClouds, cylinderClouds);
  showClouds(clouds);

  return (0);
}
