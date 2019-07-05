#include <pcl/io/ply_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

float coeff_init[11][4];
float x_bounds[10];
int m=0;

double point2planedistance(pcl::PointXYZ pt, pcl::ModelCoefficients::Ptr coefficients){
    double f1 = fabs(coefficients->values[0]*pt.x+coefficients->values[1]*pt.y+coefficients->values[2]*pt.z+coefficients->values[3]);
    double f2 = sqrt(pow(coefficients->values[0],2)+pow(coefficients->values[1],2)+pow(coefficients->values[2],2));
    return f1/f2;
}

class ColorMap{
public:
    ColorMap(double mn, double mx): mn(mn), mx(mx){}
    void setMinMax(double min, double max){ mn = min; mx = max;}
    void setMin(double min){mn = min;}
    void setMax(double max){mx = max;}
    void getColor(double c,uint8_t& R, uint8_t& G, uint8_t& B){
        double normalized = (c - mn)/(mx-mn) * 2 - 1;
        R = (int) (base(normalized - 0.5) * 255);
        G = (int) (base(normalized) * 255);
        B = (int) (base(normalized + 0.5) * 255);
    }
    void getColor(double c, double &rd, double &gd, double &bd){
        uint8_t r;
        uint8_t g;
        uint8_t b;
        getColor(c,r,g,b);
        rd = (double)r/255;
        gd = (double)g/255;
        bd = (double)b/255;
    }
    uint32_t getColor(double c){
        uint8_t r;
        uint8_t g;
        uint8_t b;
        getColor(c,r,g,b);
        return ((uint32_t)r<<16|(uint32_t)g<<8|(uint32_t)b);
    }


private:
    double interpolate(double val, double y0, double x0, double y1, double x1){
        return (val - x0)*(y1-y0)/(x1-x0) + y0;
    }
    double base(double val){
        if (val <= -0.75) return 0;
        else if (val <= -0.25) return interpolate(val,0,-0.75,1,-0.25);
        else if (val <= 0.25) return 1;
        else if (val <= 0.75) return interpolate(val,1.0,0.25,0.0,0.75);
        else return 0;
    }
private:
    double mn,mx;
};

class Color{
private:
    uint8_t r;
    uint8_t g;
    uint8_t b;

public:
    Color(uint8_t R,uint8_t G,uint8_t B):r(R),g(G),b(B){

    }

    void getColor(uint8_t &R,uint8_t &G,uint8_t &B){
        R = r;
        G = g;
        B = b;
    }
    void getColor(double &rd, double &gd, double &bd){
        rd = (double)r/255;
        gd = (double)g/255;
        bd = (double)b/255;
    }
    uint32_t getColor(){
        return ((uint32_t)r<<16|(uint32_t)g<<8|(uint32_t)b);
    }
};

class PlaneFitter
{
public:
  std::string _name;
  _color_pc_with_error= false;
  std::vector<Color> colors;
  PlaneFitter()
  {
    _name="stairs"
    createColors();
    std::cout<<_name<<" : initialised. "<<std::endl;
  }
  void createColors()
  {
          uint8_t r = 0;
          uint8_t g = 0;
          uint8_t b = 0;
          for (int i=0;i<20;i++){
              while (r<70 && g < 70 && b < 70){
                  r = rand()%(255);
                  g = rand()%(255);
                  b = rand()%(255);
              }
              Color c(r,g,b);
              r = 0;
              g = 0;
              b = 0;
              colors.push_back(c);
          }
  }
  // selecting GPU and prining info
  /* int device = 0; */
  /* pcl::gpu::setDevice (device); */
  /* pcl::gpu::printShortCudaDeviceInfo (device); */

  // Read in the cloud data

  /* pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>); */
  /* pcl::PLYReader reader; */
  /* reader.read("data/1.ply", *cloud); */
  /* pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>); */
  /* pcl::PCDReader reader; */
  /* pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>); */
  /* reader.read ("data/table_scene_lms400.pcd", *cloud); */
  void getPlanes()
  {
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    reader.read ("/home/akshatha/pcl/cluster/13_Depth.pcd", *cloud);
    std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud (cloud);
    // for 1_pc
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    // for 2_pc
    /* vg.setLeafSize (0.001f, 0.001f, 0.001f); */
    vg.filter (*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> segx;
    pcl::PointIndices::Ptr inliersx (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficientsx (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_planex (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PCDWriter writer;
    segx.setOptimizeCoefficients (true);
    segx.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    segx.setAxis(Eigen::Vector3f (1,0,0));
    segx.setMethodType (pcl::SAC_RANSAC);
    segx.setMaxIterations (1000);
    segx.setEpsAngle(pcl::deg2rad (5.0));
    segx.setDistanceThreshold (0.1);

    // Write the filtered pointcloud
    writer.write<pcl::PointXYZ> ("/home/akshatha/pcl/cluster/cloud_filtered.pcd", *cloud_filtered, false); //*
    int k=0;
    int x_i=0;
    int i=0, nr_pointsx = (int) cloud_filtered->points.size ();
    while (cloud_filtered->points.size () > 0.2 * nr_pointsx)
    {

      // Segment the largest planar component from the remaining cloud
      segx.setInputCloud (cloud_filtered);
      segx.segment (*inliersx, *coefficientsx);
      if (inliersx->indices.size () == 0)
      {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        break;
      }
      /*std::cerr<< "Model coefficients: " << coefficientsx->values[0] << " "
                                      << coefficientsx->values[1] << " "
                                      << coefficientsx->values[2] << " "
                                      << coefficientsx->values[3] << std::endl;*/
      x_bounds[x_i]=-(coefficientsx->values[3])/coefficientsx->values[0];
      x_i++;
      // Extract the planar inliers from the input cloud
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud (cloud_filtered);
      extract.setIndices (inliersx);
      extract.setNegative (false);

      // Get the points associated with the planar surface
      extract.filter (*cloud_planex);
      std::cout << "PointCloud representing the planar component: " << cloud_planex->points.size () << " data points." << std::endl;
      std::stringstream ss;
      ss << "/home/akshatha/pcl/cluster/p" << k << ".pcd";
      writer.write<pcl::PointXYZ> (ss.str (), *cloud_planex, false);
      // Remove the planar inliers, extract the rest
      extract.setNegative (true);
      extract.filter (*cloud_f);
      *cloud_filtered = *cloud_f;
      k++;
    }
    double x_left=x_bounds[0];
    double x_right=x_bounds[0];
    for(int p=0;p<k;p++)
    {
      if(x_bounds[p]<x_left)
      {
        x_left=x_bounds[p];
      }
      if(x_bounds[p]>x_right)
      {
        x_right=x_bounds[p];
      }
    }
    std::cout<<"leftmost : "<<x_left<<std::endl;
    std::cout<<"rightmost : "<<x_right<<std::endl;
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (x_left, x_right);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_filtered);

    writer.write<pcl::PointXYZ> ("/home/akshatha/pcl/cluster/cloud_filtered.pcd", *cloud_filtered, false);



    // Working to segment stair planes after removing the side walls


    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setAxis(Eigen::Vector3f (0,0,1));
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setEpsAngle(pcl::deg2rad (6.0));
    seg.setDistanceThreshold (0.04);

    // Create pointcloud to publish inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pub (new pcl::PointCloud<pcl::PointXYZ> ());
    int original_size=cloud_filtered->height*cloud_filtered->width;
    int n_planes(0);
    k=0;
    i=0;
    int nr_points = (int) cloud_filtered->points.size ();
    while (cloud_filtered->points.size () > 0.3 * original_size)
    {

      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud (cloud_filtered);
      seg.segment (*inliers, *coefficients);
      if (inliers->indices.size () == 0)
      {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        break;
      }
      std::cerr<< "Model coefficients: " << coefficients->values[0] << " "
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " "
                                      << coefficients->values[3] << std::endl;
      for (int l=0;l<4;l++ )
      {
        coeff_init[m][l]=coefficients->values[l];
      }
      m++;
      // Extract the planar inliers from the input cloud
      double mean_error(0);
      double max_error(0);
      double min_error(100000);
      std::vector<double> err;
      for (int i=0;i<inliers->indices.size();i++)
      {
          /* std::cout<<"entered first for"<<std::endl; */

          // Get Point
          pcl::PointXYZ pt = cloud_filtered->points[inliers->indices[i]];

          // Compute distance
          double d = point2planedistance(pt,coefficients)*1000;// mm
          err.push_back(d);

          // Update statistics
          mean_error += d;
          if (d>max_error) max_error = d;
          if (d<min_error) min_error = d;

      }
      mean_error/=inliers->indices.size();

      // Compute Standard deviation
      ColorMap cm(min_error,max_error);
      double sigma(0);
      for (int i=0;i<inliers->indices.size();i++){
          //std::cout<<"entered second for"<<std::endl;

          sigma += pow(err[i] - mean_error,2);

          // Get Point
          pcl::PointXYZ pt = cloud_filtered->points[inliers->indices[i]];

          // Copy point to noew cloud
          pcl::PointXYZRGB pt_color;
          pt_color.x = pt.x;
          pt_color.y = pt.y;
          pt_color.z = pt.z;
          uint32_t rgb;
          if (_color_pc_with_error)
              rgb = cm.getColor(err[i]);
          else
              rgb = colors[n_planes].getColor();
          pt_color.rgb = *reinterpret_cast<float*>(&rgb);
          cloud_pub->points.push_back(pt_color);

      }
      std::cout<<"exit for"<<std::endl;
      sigma = sqrt(sigma/inliers->indices.size());

      // Extract inliers
      extract.setInputCloud(cloud_filtered);
      extract.setIndices(inliers);
      extract.setNegative(true);
      pcl::PointCloud<pcl::PointXYZ> cloudF;
      extract.filter(cloudF);
      cloud_filtered->swap(cloudF);

      // Display infor
      std::cout<< _name<<": fitted plane "<< n_planes << ": " << coefficients->values[0] << "x" << (coefficients->values[1]>=0?"+":"") << coefficients->values[1] <<"y" << (coefficients->values[2]>=0?"+":"") << coefficients->values[2] << "z"<< (coefficients->values[3]>=0?"+":"") << coefficients->values[3] << "=0 (inliers: " << inliers->indices.size() << "u/" << original_size << ")" << std::endl;
      std::cout << _name << ": mean error: " << mean_error << "(mm), standard deviation: " << sigma << " (mm), max error: " << max_error << "(mm)" << std::endl;
      std::cout << _name << ": points left in cloud " << cloud_filtered->width<<" "<<cloud->height << std::endl;
      std::cout << _name << ": number of points in cloud " << cloud_filtered->size() << std::endl;

      // Nest iteration
      n_planes++;
    }
    cloud_pub->width=cloud_pub->points.size();
    cloud_pub->height=1;
    // Publish points
    /* sensor_msgs::PointCloud2 cloud_publish; */
    /* pcl::toROSMsg(*cloud_pub,cloud_publish); */
    /* cloud_publish.header = msg->header; */
    /* _pub_inliers.publish(cloud_publish); */

    pcl::io::savePCDFileASCII ("test_pcd.pcd", *cloud_pub);
    //pcl::io::savePLYFileBinary("test_ply.ply", *cloud_pub);
    std::cerr << "Saved " << cloud_pub->points.size () << " data points to test_pcd.pcd." << std::endl;
}

};

// first sort parallel planes
// find point on the plane and find distance from parallel
//average over two planes
// average over all planes
int main(int argc,char** argv){

    // Initialize ROS
    /* ros::init(argc,argv,"planeFitter"); */
    /* ros::NodeHandle nh("~"); */

    PlaneFitter pf;
    pf.getPlanes();
    /* pf.spin(); */

    return 0;
}
