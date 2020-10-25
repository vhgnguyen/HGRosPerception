#include <helper/tools.h>

Tools::Tools()
{

    // Transformation matrices
    TRANS_VELO_TO_CAM = MatrixXf::Zero(4, 4);
    TRANS_VELO_TO_CAM << 
        7.533745e-03, -9.999714e-01, -6.166020e-04, -4.069766e-03,
        1.480249e-02, 7.280733e-04, -9.998902e-01, -7.631618e-02,
        9.998621e-01, 7.523790e-03, 1.480755e-02, -2.717806e-01,
        0, 0, 0, 0;
    
    TRANS_CAM_TO_RECTCAM = MatrixXf::Zero(4, 4);
    TRANS_CAM_TO_RECTCAM << 
        9.999239e-01, 9.837760e-03, -7.445048e-03, 0,
        -9.869795e-03, 9.999421e-01, -4.278459e-03, 0,
        7.402527e-03, 4.351614e-03, 9.999631e-01, 0,
        0, 0, 0, 1;

    TRANS_RECTCAM_TO_IMAGE = MatrixXf::Zero(3, 4);
    TRANS_RECTCAM_TO_IMAGE <<
        7.215377e+02, 0.000000e+00, 6.095593e+02, 4.485728e+01,
        0.000000e+00, 7.215377e+02, 1.728540e+02, 2.163791e-01,
        0.000000e+00, 0.000000e+00, 1.000000e+00, 2.745884e-03;

    SEMANTIC_NAMES = std::vector<string>{
        // Static objects
        "Road", "Sidewalk", "Building", "Wall", "Fence", "Pole",
        "Traffic light", "Traffic sign", "Vegetation", "Terrain", "Sky",
        // Dynamic objects
        "Pedestrian", "Rider", "Car", "Truck", "Bus", "Train", "Motocycle", "Bicycle"
    };

    SEMANTIC_COLOR_TO_CLASS = std::map<int, int>{
        // Sum of RGB values
		// Static objects
		{320, 0}, {511, 1}, {210, 2}, {260, 3}, {496, 4}, {459, 5},
		{450, 6}, {440, 7}, {284, 8}, {555, 9}, {380, 10},
		// Dynamic objects
		{300, 11}, {255, 12}, {142, 13}, {70, 14},{160, 15}, {180, 16}, {230, 17}, {162, 18}
	};

	SEMANTIC_CLASS_TO_COLOR = MatrixXi::Zero(19, 3);
	SEMANTIC_CLASS_TO_COLOR <<
	// Static objects
		128,  64, 128, // Road
		244,  35, 232, // Sidewalk
		 70,  70,  70, // Building
		102, 102, 156, // Wall
		190, 153, 153, // Fence
		153, 153, 153, // Pole
		250, 170,  30, // Traffic light
		220, 220,   0, // Traffic sign
		107, 142,  35, // Vegetation
		152, 251, 152, // Terrain
		 70, 130, 180, // Sky

	// Dynamic objects
		220,  20,  60, // Pedestrian
		255,   0,   0, // Rider
		  0,   0, 142, // Car
		  0,   0,  70, // Truck
		  0,  60, 100, // Bus
		  0,  80, 100, // Train
		  0,   0, 230, // Motocycle
		119,  11,  32;  // Bicycle

	SEMANTIC_KERNEL_SIZE = VectorXi::Zero(8);	
	SEMANTIC_KERNEL_SIZE <<
		1, // Pedestrian
		2, // Rider
		1, // Car
		4, // Truck
		5, // Bus
		5, // Train
		2, // Motocycle
		2; // Bicycle
}

Tools::~Tools(){

}

int Tools::getClusterKernel(const int semantic){

	if(semantic > 10)
		return SEMANTIC_KERNEL_SIZE(semantic - 11);
	else
		return -1;
}

MatrixXf Tools::getImage2DBoundingBox(
    const Point & point,
    const float width,
    const float hieght){

	MatrixXf velo_points = MatrixXf::Zero(4, 2);
	velo_points(0,0) = point.x;
	velo_points(1,0) = point.y + width;
	velo_points(2,0) = point.z + height;
	velo_points(3,0) = 1;
	velo_points(0,1) = point.x;
	velo_points(1,1) = point.y - width;
	velo_points(2,1) = point.z;
	velo_points(3,1) = 1;
	MatrixXf image_points = transformVeloToImage(velo_points);
	return image_points;
}

MatrixXf Tools::getImage2DBoundingBox(
	const Object o){
	
	// Velo orientation
	float rad_ori = o.orientation * M_PI_180;

	float half_length = o.length / 2;
	float half_width = o.width / 2;
	float cos_l = half_length * cos(rad_ori);
	float sin_l = half_length * sin(rad_ori);
	float cos_w = half_width * cos(rad_ori);
	float sin_w = half_width * sin(rad_ori);

	MatrixXf velo_points = MatrixXf::Zero(4, 8);
	velo_points(0,0) = o.velo_pose.point.x + cos_l + sin_w;
	velo_points(1,0) = o.velo_pose.point.y + sin_l - cos_w;
	velo_points(2,0) = o.velo_pose.point.z + o.height;
	velo_points(3,0) = 1;

	velo_points(0,1) = o.velo_pose.point.x + cos_l - sin_w;
	velo_points(1,1) = o.velo_pose.point.y + sin_l + cos_w;
	velo_points(2,1) = o.velo_pose.point.z + o.height;
	velo_points(3,1) = 1;

	velo_points(0,2) = o.velo_pose.point.x - cos_l + sin_w;
	velo_points(1,2) = o.velo_pose.point.y - sin_l - cos_w;
	velo_points(2,2) = o.velo_pose.point.z + o.height;
	velo_points(3,2) = 1;

	velo_points(0,3) = o.velo_pose.point.x - cos_l - sin_w;
	velo_points(1,3) = o.velo_pose.point.y - sin_l + cos_w;
	velo_points(2,3) = o.velo_pose.point.z + o.height;
	velo_points(3,3) = 1;

	velo_points(0,4) = o.velo_pose.point.x + cos_l + sin_w;
	velo_points(1,4) = o.velo_pose.point.y + sin_l - cos_w;
	velo_points(2,4) = o.velo_pose.point.z;
	velo_points(3,4) = 1;

	velo_points(0,5) = o.velo_pose.point.x + cos_l - sin_w;
	velo_points(1,5) = o.velo_pose.point.y + sin_l + cos_w;
	velo_points(2,5) = o.velo_pose.point.z;
	velo_points(3,5) = 1;

	velo_points(0,6) = o.velo_pose.point.x - cos_l + sin_w;
	velo_points(1,6) = o.velo_pose.point.y - sin_l - cos_w;
	velo_points(2,6) = o.velo_pose.point.z;
	velo_points(3,6) = 1;

	velo_points(0,7) = o.velo_pose.point.x - cos_l - sin_w;
	velo_points(1,7) = o.velo_pose.point.y - sin_l + cos_w;
	velo_points(2,7) = o.velo_pose.point.z;
	velo_points(3,7) = 1;

	MatrixXf image_points = transformVeloToImage(velo_points);

	float x_min = image_points(0,0);
	float x_max = image_points(0,0);
	float y_min = image_points(1,0);
	float y_max = image_points(1,0);
	for (int i=0; i<8; i++){
		x_min = (x_min < image_points(0,i)) ? x_min : image_points(0,i);
		x_max = (x_max > image_points(0,i)) ? x_max : image_points(0,i);
		y_min = (y_min < image_points(1,i)) ? y_min : image_points(1,i);
		y_max = (y_max > image_points(1,i)) ? y_max : image_points(1,i);
	}
	if (x_max > 1237) x_max = 1237.0;
	if (x_min < 0) x_min = 0.0;
	if (y_max > 370) y_max = 370.0;
	if (y_min < 0) y_min = 0.0;

	MatrixXf box = MatrixXf::Zero(2, 2);
	box(0,0) = x_min;
	box(1,0) = y_min;
	box(0,1) = x_max;
	box(1,1) = y_max;

	return box;
}

MatrixXf Tools::transformVeloToCam(const MatrixXf & velo_points){
	return TRANS_VELO_TO_CAM * velo_points;
}

MatrixXf Tools::transformCamToRectCam(const MatrixXf &)