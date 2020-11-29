#include <detection_lib/detection.h>

namespace detection {

Detection::Detection(ros::NodeHandle nh, ros::NodeHandle pnh):
	nh_(nh),
	pnh_(pnh)
	{

	pnh_.param("sensor_frame/z",
		sensor_frame_z_, sensor_frame_z_);

	// Get parameter
	pnh_.param("cart_grid/height",
		cart_grid_height_, cart_grid_height_);
	pnh_.param("cart_grid/width",
		cart_grid_width_, cart_grid_width_);
	pnh_.param("cart_grid/cell_size",
		cart_grid_cell_size_, cart_grid_cell_size_);
	// private_nh_.param("grid/range/max", params_.grid_range_max,
	// 	params_.grid_range_max);
	// private_nh_.param("grid/cell/size", params_.grid_cell_size,
	// 	params_.grid_cell_size);
	pnh_.param("pedestrian/spawn/side/min", params_.ped_spawn.side_min,
		params_.ped_spawn.side_min);
	pnh_.param("pedestrian/spawn/side/max", params_.ped_spawn.side_max,
		params_.ped_spawn.side_max);
	pnh_.param("pedestrian/spawn/height/min", params_.ped_spawn.height_min,
		params_.ped_spawn.height_min);
	pnh_.param("pedestrian/spawn/height/max", params_.ped_spawn.height_max,
		params_.ped_spawn.height_max);
	pnh_.param("pedestrian/spawn/semantic/min", params_.ped_spawn.semantic_min,
		params_.ped_spawn.semantic_min);
	pnh_.param("car/spawn/side/min", params_.car_spawn.side_min,
		params_.car_spawn.side_min);
	pnh_.param("car/spawn/side/max", params_.car_spawn.side_max,
		params_.car_spawn.side_max);
	pnh_.param("car/spawn/height/min", params_.car_spawn.height_min,
		params_.car_spawn.height_min);
	pnh_.param("car/spawn/height/max", params_.car_spawn.height_max,
		params_.car_spawn.height_max);
	pnh_.param("car/spawn/semantic/min", params_.car_spawn.semantic_min,
		params_.car_spawn.semantic_min);
	pnh_.param("pedestrian/update/side/min", params_.ped_update.side_min,
		params_.ped_update.side_min);
	pnh_.param("pedestrian/update/side/max", params_.ped_update.side_max,
		params_.ped_update.side_max);
	pnh_.param("pedestrian/update/height/min", params_.ped_update.height_min,
		params_.ped_update.height_min);
	pnh_.param("pedestrian/update/height/max", params_.ped_update.height_max,
		params_.ped_update.height_max);
	pnh_.param("pedestrian/update/semantic/min", params_.ped_update.semantic_min,
		params_.ped_update.semantic_min);
	pnh_.param("car/update/side/min", params_.car_update.side_min,
		params_.car_update.side_min);
	pnh_.param("car/update/side/max", params_.car_update.side_max,
		params_.car_update.side_max);
	pnh_.param("car/update/height/min", params_.car_update.height_min,
		params_.car_update.height_min);
	pnh_.param("car/update/height/max", params_.car_update.height_max,
		params_.car_update.height_max);
	pnh_.param("car/update/semantic/min", params_.car_update.semantic_min,
		params_.car_update.semantic_min);

	// // Print parameters
	// ROS_INFO_STREAM("ped_spawn.side_min " << params_.ped_spawn.side_min);
	// ROS_INFO_STREAM("ped_spawn.side_max " << params_.ped_spawn.side_max);
	// ROS_INFO_STREAM("ped_spawn.height_min " << params_.ped_spawn.height_min);
	// ROS_INFO_STREAM("ped_spawn.height_max " << params_.ped_spawn.height_max);
	// ROS_INFO_STREAM("ped_spawn.semantic_min " << params_.ped_spawn.semantic_min);
	// ROS_INFO_STREAM("car_spawn.side_min " << params_.car_spawn.side_min);
	// ROS_INFO_STREAM("car_spawn.side_max " << params_.car_spawn.side_max);
	// ROS_INFO_STREAM("car_spawn.height_min " << params_.car_spawn.height_min);
	// ROS_INFO_STREAM("car_spawn.height_max " << params_.car_spawn.height_max);
	// ROS_INFO_STREAM("car_spawn.semantic_min " << params_.car_spawn.semantic_min);
	// ROS_INFO_STREAM("ped_update.side_min " << params_.ped_update.side_min);
	// ROS_INFO_STREAM("ped_update.side_max " << params_.ped_update.side_max);
	// ROS_INFO_STREAM("ped_update.height_min " << params_.ped_update.height_min);
	// ROS_INFO_STREAM("ped_update.height_max " << params_.ped_update.height_max);
	// ROS_INFO_STREAM("ped_update.semantic_min " << params_.ped_update.semantic_min);
	// ROS_INFO_STREAM("car_update.side_min " << params_.car_update.side_min);
	// ROS_INFO_STREAM("car_update.side_max " << params_.car_update.side_max);
	// ROS_INFO_STREAM("car_update.height_min " << params_.car_update.height_min);
	// ROS_INFO_STREAM("car_update.height_max " << params_.car_update.height_max);
	// ROS_INFO_STREAM("car_update.semantic_min " << params_.car_update.semantic_min);

	// Init counter for publishing
	time_frame_ = 0;

	// Define Subscriber
	sub_pointcloud_elevated_ = nh.subscribe(
		"/sensors/elevated/pointcloud", 2, &Detection::process, this);

	// Define Publisher
	pub_pointcloud_per_cell_ = nh_.advertise<PointCloud2>(
		"/detection/pointcloud_per_cell", 2);
	pub_detected_objects_ = nh_.advertise<ObjectArray>(
		"/detection/objects", 2);
}

Detection::~Detection(){

}

void Detection::process(const PointCloud2ConstPtr & msg_pointcloud_elevated) {

    VRGBPointCloud::Ptr pcl_cloud(new VRGBPointCloud);
    pcl::fromROSMsg(*msg_pointcloud_elevated, *pcl_cloud);

    produce2Dgrid(pcl_cloud);
    fillObjectList();
    object_array_.header = msg_pointcloud_elevated->header;
    object_array_.header.stamp.nsec = msg_pointcloud_elevated->header.stamp.nsec;
    pub_detected_objects_.publish(object_array_);

    for (int i=0; i < object_array_.list.size(); ++i) {
        printObject(object_array_.list[i]);
    }

    time_frame_++;
}

void Detection::produce2Dgrid(const VRGBPointCloud::Ptr & pcl_cloud) {

	// Hash table of points of semantic point cloud in each cell
	std::map<int, std::map<int, int>> cell_hash_table;

	// Buffer
	int grid_x, grid_y;
	cv::Mat grid_min_z(cart_grid_height_, cart_grid_width_, CV_32FC1, cv::Scalar(100.0));
	cv::Mat grid_max_z(cart_grid_height_, cart_grid_width_, CV_32FC1, cv::Scalar(-100.0));

	// Loop through semantic point cloud
	for (auto & point: pcl_cloud->points) {

		// Get cartesian grid indices
		fromVeloCoordsToCartesianCell(point.x, point.y, grid_x, grid_y);
		int grid_occ = grid_y * cart_grid_width_ + grid_x;

		// Get semantic class
		int semantic_class = tools_.SEMANTIC_COLOR_TO_CLASS[point.r + point.g + point.b];

		cell_hash_table[grid_occ][semantic_class]++;

		grid_min_z.at<float>(grid_y, grid_x) = std::min(grid_min_z.at<float>(grid_y, grid_x), point.z);
		grid_max_z.at<float>(grid_y, grid_x) = std::max(grid_min_z.at<float>(grid_y, grid_x), point.z);

	}

	VRGBPointCloud::Ptr pcl_cloud_per_cell(new VRGBPointCloud);
	pcl_cloud_per_cell->points.clear();

	float x, y;

	// Loop over hash table to find most dominant semantic label
	std::map<int, std::map<int, int>>::iterator it;
	cv::Mat grid_semantic(cart_grid_height_, cart_grid_width_, CV_8UC1, cv::Scalar(100));

	for (it = cell_hash_table.begin(); it != cell_hash_table.end(); it++) {

		std::map<int, int>::iterator it2;
		int max_semantic_counter = -1;
		int max_class;

		for (it2 = it->second.begin(); it2 != it->second.end(); it2++) {
			if (it2->second > max_semantic_counter) {
				max_semantic_counter = it2->second;
				max_class = it2->first;
			}
		}
		
		// Determine cartesian grid indices
		int grid_x = it->first % cart_grid_width_;
		int grid_y = it->first / cart_grid_width_;

		// Calculate velodyne coordinates
		fromCartesianCellToVeloCoords(grid_x, grid_y, x, y);

		// Write point to sparse point cloud
		VRGBPoint point;
		point.x = x;
		point.y = y;
		point.z = sensor_frame_z_;
		point.r = tools_.SEMANTIC_CLASS_TO_COLOR(max_class,0);
		point.g = tools_.SEMANTIC_CLASS_TO_COLOR(max_class,1);
		point.b = tools_.SEMANTIC_CLASS_TO_COLOR(max_class,2);
		pcl_cloud_per_cell->points.push_back(point);

		// Fill detection grid with semantic class
		grid_semantic.at<uchar>(grid_y, grid_x) = max_class;
	}

	// Publish sparse semantic cloud
	pcl_cloud_per_cell->header.frame_id = pcl_cloud->header.frame_id;
	pcl_cloud_per_cell->header.stamp = pcl_cloud->header.stamp;
	pub_pointcloud_per_cell_.publish(pcl_cloud_per_cell);

	// Cluster segmentation
	clusters_.clear();

	for (int y  = 0; y < cart_grid_height_; y++) {
		for (int x = 0; x < cart_grid_width_; x++) {

			int semantic_class = grid_semantic.at<uchar>(y, x);

			if(!isKittiValidSemantic(semantic_class)){
				continue;
			}

			// Flag cell as visited
			grid_semantic.at<uchar>(y, x) += 100;

			// New cluster
			Cluster c = Cluster();
			c.kernel = tools_.getClusterKernel(semantic_class);

			// Init neighbor queue and add cell
			std::queue<cv::Point> neighbor_queue;
			neighbor_queue.push(cv::Point(x,y));

			// Init non-neighbor list
			std::vector<cv::Point> non_neighbor_list;

			// Search neighbor cells
			while (!neighbor_queue.empty()) {

				c.geometric.cells.push_back(neighbor_queue.front());
				int c_x = neighbor_queue.front().x;
				int c_y = neighbor_queue.front().y;
				neighbor_queue.pop();

				for (int k = -c.kernel; k <= c.kernel; ++k) {
					for (int l = -c.kernel; l <= c.kernel; ++l) {
						
						if (k == 0 || l == 0) continue;

						// Calculate neighbor cell indices
						int n_x = c_x + k;
						int n_y = c_y + l;

						// Check if neighbor cell in grid
						if (n_x >= 0 && n_x < cart_grid_width_ && 
							n_y >= 0 && n_y < cart_grid_height_) {
							
							// Get semantic class of neighbor cell
							int n_semantic_class = grid_semantic.at<uchar>(n_y, n_x);

							// Check if it matches with cluster semantic
							if (n_semantic_class == semantic_class) {

								// Flag it
								grid_semantic.at<uchar>(n_y, n_x) += 100;
								neighbor_queue.push(cv::Point(n_x, n_y));
							}
							// If it is not pedestrian nor car
							else if (!isKittiValidSemantic(n_semantic_class) && n_semantic_class < 100) {
								grid_semantic.at<uchar>(n_y,n_x) += 100;
								non_neighbor_list.push_back(cv::Point(n_x,n_y));
								c.semantic.diff_counter++;
							}
						}
					}
				}
			} // while loop through neighbor queue

			// Write non neighbor list back to unvisited
			for(int nn = 0; nn < non_neighbor_list.size(); ++nn){

				int n_x = non_neighbor_list[nn].x;
				int n_y = non_neighbor_list[nn].y;
				// grid.at<cv::Vec3f>(n_y,n_x)[0] += 20;
				grid_semantic.at<uchar>(n_y,n_x) -= 100;
			}

			// Add semantic information
			c.semantic.id = semantic_class;
			c.geometric.num_cells = c.geometric.cells.size();
			c.semantic.confidence = float(c.geometric.num_cells) / 
				(c.geometric.num_cells + c.semantic.diff_counter);
			c.semantic.name = tools_.SEMANTIC_NAMES[c.semantic.id];

			// Push back cluster
			clusters_.push_back(c);
		}
	}

	// Get object from cluster
	object_array_.list.clear();
	float gx, gy;

	for (int i = 0; i < clusters_.size(); i++) {

		Cluster & c = clusters_[i];
		c.id = i;

		cv::RotatedRect rect = cv::minAreaRect(cv::Mat(c.geometric.cells));
		fromRectangleCoordsToVeloCoords(rect.center.x, rect.center.y, c.geometric.x, c.geometric.y);

		// Cluster size
		c.geometric.width = (rect.size.width + 1) * cart_grid_cell_size_;
		c.geometric.length = (rect.size.height + 1) * cart_grid_cell_size_;

		// Find max min in z coordinate
		float min_low_z = grid_min_z.at<float>(
			c.geometric.cells[0].y, c.geometric.cells[0].x);
		float max_high_z = grid_max_z.at<float>(
			c.geometric.cells[0].y,c.geometric.cells[0].x);
		for(int j = 1; j < c.geometric.cells.size(); ++j){
			min_low_z = std::min(min_low_z, grid_min_z.at<float>(
				c.geometric.cells[j].y, c.geometric.cells[j].x));
			max_high_z = std::max(max_high_z, grid_max_z.at<float>(
				c.geometric.cells[j].y, c.geometric.cells[j].x));
		}

		// Get ground level and height of cluster
		c.geometric.z = min_low_z;
		c.geometric.height = max_high_z - min_low_z;

		// Get orientation of bounding box
		// Minus since opencv y,x is the opposite of the velodyne frame
		c.geometric.orientation = -rect.angle;
		// Store rect as back up
		c.rect = rect;

		// Get color BGR
		int r = tools_.SEMANTIC_CLASS_TO_COLOR(c.semantic.id, 0);
		int g = tools_.SEMANTIC_CLASS_TO_COLOR(c.semantic.id, 1);
		int b = tools_.SEMANTIC_CLASS_TO_COLOR(c.semantic.id, 2);
		c.color = cv::Scalar(r, g, b);

		// Determine if cluster can be a new track
		// Car
		if(c.semantic.id == 13){
			if(updateCar(c)){
				if(spawnCar(c)){
					c.is_new_track = true;
				}
				addObject(c);
			}
		}
		// Pedestrian
		else if(c.semantic.id == 11){
			if(updatePed(c)){
				if(spawnPed(c)){
					c.is_new_track = true;
				}
				addObject(c);
			}
		}
	}
}

void Detection::fillObjectList(){

	// Transform objects in camera and world frame
	try{
		for(int i = 0; i < object_array_.list.size(); ++i){

			listener_.transformPoint("world",
				object_array_.list[i].velo_pose,
				object_array_.list[i].world_pose);

			listener_.transformPoint("camera_color_left",
				object_array_.list[i].velo_pose,
				object_array_.list[i].cam_pose);
		}
	}
	catch(tf::TransformException& ex){
		ROS_ERROR("Received an exception trying to transform a point from"
			"\"velo_link\" to \"world\": %s", ex.what());
	}
}

void Detection::addObject(const Cluster & c){

	// Create object
	Object object;
	object.id = c.id;

	// Pose in velo frame
	object.velo_pose.header.frame_id = "velo_link";
	object.velo_pose.point.x = c.geometric.x;
	object.velo_pose.point.y = c.geometric.y;
	object.velo_pose.point.z = c.geometric.z;

	// Geometry
	object.width = c.geometric.width;
	object.length = c.geometric.length;
	object.height = c.geometric.height;
	object.orientation = c.geometric.orientation;

	// Semantic
	object.semantic_id = c.semantic.id;
	object.semantic_confidence = c.semantic.confidence;
	object.semantic_name = c.semantic.name;

	// Color with BGR encoding
	object.r = c.color[0];
	object.g = c.color[1];
	object.b = c.color[2];
	if(c.is_new_track){
		object.a = 0.75f;
	}
	else{
		object.a = 0.35f;
	}

	// Tracking
	object.is_new_track = c.is_new_track;

	// Push back object to list
	object_array_.list.push_back(object);
}

bool Detection::spawnPed(const Cluster & c){
	return (c.geometric.width > params_.ped_spawn.side_min ||
		c.geometric.length > params_.ped_spawn.side_min)
		&&
		(c.geometric.width < params_.ped_spawn.side_max &&
		c.geometric.length < params_.ped_spawn.side_max)
		&&
		(c.geometric.height > params_.ped_spawn.height_min && 
		c.geometric.height < params_.ped_spawn.height_max)
		&&
		(c.semantic.confidence > params_.ped_spawn.semantic_min);
}

bool Detection::spawnCar(const Cluster & c){
	return (c.geometric.width > params_.car_spawn.side_min ||
		c.geometric.length > params_.car_spawn.side_min)
		&&
		(c.geometric.width < params_.car_spawn.side_max &&
		c.geometric.length < params_.car_spawn.side_max)
		&&
		(c.geometric.height > params_.car_spawn.height_min && 
		c.geometric.height < params_.car_spawn.height_max)
		&&
		(c.semantic.confidence > params_.car_spawn.semantic_min);
}

bool Detection::updatePed(const Cluster & c){
	return (c.geometric.width > params_.ped_update.side_min ||
		c.geometric.length > params_.ped_update.side_min)
		&&
		(c.geometric.width < params_.ped_update.side_max &&
		c.geometric.length < params_.ped_update.side_max)
		&&
		(c.geometric.height > params_.ped_update.height_min && 
		c.geometric.height < params_.ped_update.height_max)
		&&
		(c.semantic.confidence > params_.ped_update.semantic_min);
}

bool Detection::updateCar(const Cluster & c){
	return (c.geometric.width > params_.car_update.side_min ||
		c.geometric.length > params_.car_update.side_min)
		&&
		(c.geometric.width < params_.car_update.side_max &&
		c.geometric.length < params_.car_update.side_max)
		&&
		(c.geometric.height > params_.car_update.height_min && 
		c.geometric.height < params_.car_update.height_max)
		&&
		(c.semantic.confidence > params_.car_update.semantic_min);
}

// bool Detection::isValidSemantic(const int semantic_class){
// 	return semantic_class > 10;
// }

bool Detection::isKittiValidSemantic(const int semantic_class)
{

	// Only allow Cars and Pedestrians
	return (semantic_class == 11 || semantic_class == 13);
}

void Detection::printCluster(const Cluster & c){

	ROS_INFO("Cluster %d Label %s New Track %d with to [%f,%d,%d],"
		" pos[x,y,z] [%f,%f,%f]"
		" form[w,l,h,o] [%f,%f,%f,%f]",
		c.id, c.semantic.name.c_str(), c.is_new_track ,
		c.semantic.confidence, c.geometric.num_cells, c.semantic.diff_counter,
		c.geometric.x, c.geometric.y, c.geometric.z,
		c.geometric.width, c.geometric.length,
		c.geometric.height, c.geometric.orientation);
}

void Detection::printObject(const Object & o){

	ROS_INFO("Object %d Label %s New Track %d with to [%f],"
		" pos[x,y,z] [%f,%f,%f]"
		" form[w,l,h,o] [%f,%f,%f,%f]",
		o.id, o.semantic_name.c_str(), o.is_new_track ,o.semantic_confidence,
		o.velo_pose.point.x, o.velo_pose.point.y, o.velo_pose.point.z,
		o.width, o.length,
		o.height, o.orientation);
}

void Detection::fromVeloCoordsToCartesianCell(
	const float x, const float y, int & grid_x, int & grid_y
) {
	grid_y = cart_grid_height_ - x / cart_grid_cell_size_;
	grid_x = cart_grid_height_ - y / cart_grid_cell_size_;
}

void Detection::fromCartesianCellToVeloCoords(
	const int grid_x, const int grid_y, float & x, float & y){

	x = (cart_grid_height_ - grid_y) * cart_grid_cell_size_ - 
		cart_grid_cell_size_ / 2;
	y = (cart_grid_height_ - grid_x) * cart_grid_cell_size_ -
		cart_grid_cell_size_ / 2;
}

void Detection::fromRectangleCoordsToVeloCoords(
		const float grid_x, const float grid_y, float & x, float & y){

	x = cart_grid_height_ * cart_grid_cell_size_ -
		(grid_y * cart_grid_cell_size_)
		- cart_grid_cell_size_ / 2;
	y = cart_grid_height_ * cart_grid_cell_size_ -
		(grid_x * cart_grid_cell_size_)
		- cart_grid_cell_size_ / 2;
}

} // namespace detection
