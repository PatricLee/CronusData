#include <opencv.hpp>
#include <math.h>
#include <opencv.hpp>
#include <direct.h>

using namespace cv;
using namespace std;

//global switches and parameters
#define LiDarLineNumber 64

//define picture size in meter
//picture size is determined by PicWidth and PicHeight
//while field of view is up to other four
#define PicTop 30.4
#define PicDown 0
#define PicLeft 15.2
#define PicRight -15.2
#define PicWidth 304
#define PicHeight 304

//label smaller than this number on any demension should be ignored
#define MinLabelSize 50//in pixel
#define MinLabelSizeMeter MinLabelSize/PicHeight*(PicTop-PicDown)//in meter

//define height range in meter
#define HeightMax 1
#define HeightMin -4

float vertical_angle_64[64] = {
	2.0f,1.6667f,1.3333f,1.0f,0.6667f,0.3333f,0.0f,-0.3333f,
	-0.6667f,-1.0f,-1.3333f,-1.6667f,-2.0f,-2.3333f,-2.6667f,-3.0f,
	-3.3333f,-3.6667f,-4.0f,-4.3333f,-4.6667f,-5.0f,-5.3333f,-5.6667f,
	-6.0f,-6.3333f,-6.6667f,-7.0f,-7.3333f,-7.6667f,-8.0f,-8.3333f,
	-8.8333f,-9.3333f,-9.8333f,-10.3333f,-10.8333f,-11.3333f,-11.8333f,-12.3333f,
	-12.8333f,-13.3333f,-13.8333f,-14.3333f,-14.8333f,-15.3333f,-15.8333f,-16.3333f,
	-16.8333f,-17.3333f,-17.8333f,-18.3333f,-18.8333f,-19.3333f,-19.8333f,-20.3333f,
	-20.8333f,-21.3333f,-21.8333f,-22.3333f,-22.8333f,-23.3333f,-23.8333f,-24.3333f
};

extern long total_frame_counter;

//this struct contains a 3d point in xyz format from kitti dataset
struct KittiPoint {
	float x;
	float y;
	float z;

	//raw distance and rotation could be calculated from xyz coordinate
	float distance;
	float rotation;

	float reflectance;
};


float CalcHeightChannel(float in);
int LocatePixel(KittiPoint *kitti_point, int *width_out, int *height_out);
int GetObjectClass(char *object_type);

//load a velodyne file and convert to Mat form
int KittiPointtoPic(char* file_name, Mat *pic){
	FILE *load_file;
	float read_element;
	KittiPoint *kitti_point;
	kitti_point = new KittiPoint[150000];
	int point_counter = 0;
	int i, j, k;
	int err;

	if (fopen_s(&load_file, file_name, "rb") != 0) {
		return -100;
	}

	while (true) {
		fread_s(&read_element, sizeof(float), sizeof(float), 1, load_file);
		kitti_point[point_counter].x = read_element;
		fread_s(&read_element, sizeof(float), sizeof(float), 1, load_file);
		kitti_point[point_counter].y = read_element;
		fread_s(&read_element, sizeof(float), sizeof(float), 1, load_file);
		kitti_point[point_counter].z = read_element;
		fread_s(&read_element, sizeof(float), sizeof(float), 1, load_file);
		kitti_point[point_counter].reflectance = read_element;

		point_counter++;

		if (feof(load_file) != 0) {
			break;
		}
	}
	fclose(load_file);
	
	//create and initialize picture
	(*pic).create(PicHeight, PicWidth, CV_8UC3);
	for (i = 0; i < PicHeight; i++) {
		for(j=0;j<PicWidth;j++){
			for (k = 0; k < 3; k++) {
				(*pic).at<Vec3b>(i, j)[k] = 0;
			}
		}
	}

	int pix_col, pix_row;
	unsigned char pix_height, pix_ref;
	for(i = 0;i < point_counter; i++) {
		//check if in range
		if (LocatePixel(&(kitti_point[i]), &pix_col, &pix_row) == -1) {
			continue;
		}
		
		//reflectance channel
		if ((kitti_point[i]).reflectance == 0.0f) {
			continue;
		}
		pix_ref = (unsigned char)((kitti_point[i]).reflectance * 255.0f);
		if ( pix_ref > pic->at<Vec3b>(pix_row, pix_col)[1]) {
			(*pic).at<Vec3b>(pix_row, pix_col)[1] = pix_ref;
		}
		
		//height channel
		pix_height = (unsigned char)(CalcHeightChannel((kitti_point[i]).z) * 255.0f);
		if(pix_height > pic->at<Vec3b>(pix_row, pix_col)[2]){
			(*pic).at<Vec3b>(pix_row, pix_col)[2] = pix_height;
		}
		
		//number channel
		if(pic->at<Vec3b>(pix_row, pix_col)[0] != 250){
			(*pic).at<Vec3b>(pix_row, pix_col)[0] += 25;
		}
	}
	
	free(kitti_point);
	return 0;
}

float CalcHeightChannel(float in){
	return (in-(float)HeightMin)/(float)((float)HeightMax-(float)HeightMin);
}

int PlotRotatedBox(Mat img, float w, float l, float x, float y, float rz) {
	float corner_x[4], corner_y[4];
	int i, j;

	//first calculate 4 corners, in float (0,1)
	//without positioning, just at origion
	corner_x[0] = l / 2.0f*cos(rz) - w / 2.0f*sin(rz);
	corner_y[0] = l / 2.0f*sin(rz) + w / 2.0f*cos(rz);
	corner_x[1] = l / 2.0f*cos(rz) + w / 2.0f*sin(rz);
	corner_y[1] = l / 2.0f*sin(rz) - w / 2.0f*cos(rz);
	corner_x[2] = -l / 2.0f*cos(rz) + w / 2.0f*sin(rz);
	corner_y[2] = -l / 2.0f*sin(rz) - w / 2.0f*cos(rz);
	corner_x[3] = -l / 2.0f*cos(rz) - w / 2.0f*sin(rz);
	corner_y[3] = -l / 2.0f*sin(rz) + w / 2.0f*cos(rz);
	//now with offset
	for (i = 0; i < 4; i++) {
		corner_x[i] += x;
		corner_y[i] += y;
	}
	//convert into pixel
	for (i = 0; i < 4; i++) {
		corner_x[i] *= PicHeight;
		corner_y[i] *= PicWidth;
	}
	//plot lines
	for (i = 0; i < 4; i++) {
		line(img, Point(corner_y[i], corner_x[i]), Point(corner_y[(i + 1) % 4], corner_x[(i + 1) % 4]), Scalar(255, 255, 255));
	}

	return 0;
}

//-1 if no match
int GetObjectClass(char *object_type) {
	if (strcmp("Car", object_type) == 0) return 0;
	if (strcmp("Van", object_type) == 0) return 0;
	if (strcmp("Truck", object_type) == 0) return 0;
	if (strcmp("Pedestrian", object_type) == 0) return -1;
	if (strcmp("Person(sitting)", object_type) == 0) return -1;
	if (strcmp("Cyclist", object_type) == 0) return -1;
	if (strcmp("Tram", object_type) == 0) return 0;
	if (strcmp("Misc", object_type) == 0) return -1;

	return -1;
}

bool PointIsInRange(KittiPoint *kitti_point) {
	if (kitti_point->x <= PicDown || kitti_point->x >= PicTop || kitti_point->y <= PicRight || kitti_point->y >= PicLeft) {
		return false;
	}
	if (kitti_point->z <= HeightMin || kitti_point->z >= HeightMax) {
		return false;
	}

	return true;
}
//-1 when out of boarder
int LocatePixel(KittiPoint *kitti_point, int *width_out, int *height_out){
	if (PointIsInRange(kitti_point) == false) {
		return -1;
	}
	
	*width_out = (int)(((float)PicLeft - kitti_point->y) / ((float)PicLeft - (float)PicRight)*(float)PicWidth);
	*height_out = (int)(((float)PicTop - kitti_point->x) / ((float)PicTop - (float)PicDown)*(float)PicHeight);

	if (*width_out < 0 || *width_out >= PicWidth || *height_out < 0 || *height_out >= PicHeight) {
		return -1;
	}
	
	return 0;
}
