#ifndef DETECTION_H
#define DETECTION_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <opencv2/opencv.hpp>
#include <queue>
#include "helper/ObjectArray.h"
#include "helper/tools.h"
#include <tf/transform_listener.h>

namespace detection {

using namespace sensor_msgs;
using namespace helper;

struct ObjectAttributes {
    float side_min, side_max;
    float height_min, height_max;
    float semantic_min;
};

struct Parameter {
    float grid_range_max;
    float grid_cell_size;

    ObjectAttributes car_spawn, car_update;
    ObjectAttributes ped_spawn, ped_update;
};

// Semantic information of cluster
struct Semantic {
    std::map<int, int> classes;
    float confidence;
    int id;
    std::string name;
    int diff_counter;
};

// Geometric information of cluster
struct Geometric {
    float x, y, z;
    float width, height, length;
    float orientation;
    std::vector<cv::Point> cells;
    int num_cells;
};

// Information of cluster
struct Cluster {
    int id;
    int kernel; // kernel size
    Semantic semantic;
    Geometric geometric;
    cv::RotatedRect rect;
    cv::Scalar color;
    bool is_new_track;
};

typedef pcl::PointXYZRGB VRGBPoint;
typedef pcl::PointCloud<VRGBPoint> VRGBPointCloud;

class Detection {

public:

    // Constructor
    Detection(ros::NodeHandle nh, ros::NodeHandle private_nh);

    // Virtual destructor
    virtual ~Detection();

    virtual void process(
        const PointCloud2ConstPtr & msg_pointcloud_elevated
    );


private:

    // Node handle 
    ros::NodeHandle nh_, pnh_;

    // Class members
    std::vector<Cluster> clusters_;
    ObjectArray object_array_;
    Parameter params_;
    Tools tools_;
    tf::TransformListener listener_;

    // Subscriber
    ros::Subscriber sub_pointcloud_elevated_;

    // Publisher
    ros::Publisher pub_detected_objects_;
    ros::Publisher pub_pointcloud_per_cell_;

    // Params
    float sensor_frame_z_;
    int cart_grid_height_, cart_grid_width_;
    float cart_grid_cell_size_;
    int time_frame_;

    // Functions
    void produce2Dgrid(const VRGBPointCloud::Ptr & pcl_cloud);

    bool isKittiValidSemantic(const int semantic_class);

    void fromVeloCoordsToCartesianCell(
        const float x, const float y, int & grid_x, int & grid_y
    );

    void fromCartesianCellToVeloCoords(
        const int grid_x, const int grid_y, float & x, float & y
    );

    void fromRectangleCoordsToVeloCoords(
        const float grid_x, const float grid_y, float & x, float & y
    );

    // Class functions
    void fillObjectList();
    void addObject(const Cluster & c);
    bool spawnPed(const Cluster & c);
    bool spawnCar(const Cluster & c);
    bool updatePed(const Cluster & c);
    bool updateCar(const Cluster & c);
    void printCluster(const Cluster & c);
    void printObject(const Object & o);

}; // class Detection

} // namespace detection

#endif // DETECTION_H