// CronusData.cpp : 定义控制台应用程序的入口点。
//
 
#include <thread>
#include "stdafx.h"
#include <opencv.hpp>
#include <sstream>
#include <iostream>
#include <fstream>
#include "lidar_to_image.hpp"
#include <stdio.h>

#define TRAINNUM 4000

struct KittiLabel {
	char obj_class[32];
	float truncation;
	int occulation;
	float alpha;

	//2d box
	float x1;
	float y1;
	float x2;
	float y2;

	//3d box
	//in velodyne coordinates, while labels are in camera coordinates
	float h;
	float w;
	float l;
	float x;
	float y;
	float z;
	float rz;

	//retun -1 if line is empty
	int Parse(string line) {
		if (line.empty())return -1;
		istringstream line_ss(line);
		line_ss >> obj_class >> truncation >> occulation >> alpha >> x1 >> y1 >> x2 >> y2 >> h >> w >> l >> y >> z >> x >> rz;
		y = -y;
		z = -z;
		rz -= 1.57;
		rz = -rz;
		while (rz<-1.57 || rz>1.57) {
			if (rz < -1.57) rz += 3.14;
			else rz -= 3.14;
		}
		line_ss.clear();
		return 0;
	}
};

//return -1 if out of bound
//out of bound if center(x,y,z) is out of bound
int CalcLabel(KittiLabel *label) {
	if ((*label).x >= PicTop || (*label).x <= PicDown || (*label).y >= PicLeft || (*label).y <= PicRight || (*label).z >= HeightMax || (*label).z <= HeightMin) {
		return -1;
	}

	//normalize coordinates
	(*label).l = (*label).l / (float)(PicTop - PicDown);
	(*label).w = (*label).w / (float)(PicLeft - PicRight);
	(*label).h = (*label).h / (float)(HeightMax - HeightMin);
	(*label).x = ((float)PicTop - (*label).x) / (float)(PicTop - PicDown);
	(*label).y = ((float)PicLeft - (*label).y) / (float)(PicLeft - PicRight);
	(*label).z = ((float)HeightMax - (*label).z) / (float)(HeightMax - HeightMin);
	if ((*label).rz > 1.57)(*label).rz -= 3.14;
	if ((*label).rz < -1.57)(*label).rz += 3.14;

	return 0;
}

int main()
{
	char dir[128] = "E:/KITTI/data_object_velodyne/training/";

	char dir_out[128] = "D:/Kitti/";

	Mat img;
	FILE *train, *val;
	FILE *label_out;
	int file_counter = 0;
	char velo_file_name[128], img_file_name[128], label_file_name[128], label_out_file_name[128];
	char train_file_name[128], val_file_name[128];
	sprintf_s(train_file_name, "%strain.txt", dir_out);
	sprintf_s(val_file_name, "%sval.txt", dir_out);
	fopen_s(&train, train_file_name, "w");
	fopen_s(&val, val_file_name, "w");
	string label_line;
	KittiLabel l;
	while (true) {
		sprintf_s(velo_file_name, "%svelodyne/%.6d.bin", dir, file_counter);
		sprintf_s(img_file_name, "%svelo/%.6d.png", dir_out, file_counter);
		sprintf_s(label_out_file_name, "%svelo/%.6d.txt", dir_out, file_counter);
		sprintf_s(label_file_name, "%slabel/%.6d.txt", dir, file_counter);

		if (KittiPointtoPic(velo_file_name, &img) != 0) {
			break;
		}
		imwrite(img_file_name, img);

		fopen_s(&label_out, label_out_file_name, "w");
		ifstream label(label_file_name);
		while (true) {
			getline(label, label_line);
			if (l.Parse(label_line) == -1)break;
			if (CalcLabel(&l) == -1 || GetObjectClass(l.obj_class) == -1)continue;
			fprintf(label_out, "%d %f %f %f %f %f %f %f\n", GetObjectClass(l.obj_class), l.y, l.x, l.w, l.l, l.rz, l.z, l.h);
			PlotRotatedBox(img, l.w, l.l, l.x, l.y, l.rz);
		}
		label.close();
		label.clear();
		fclose(label_out);

		if (file_counter < TRAINNUM) {
			fprintf(train, "%s\n", img_file_name);
		}
		else {
			fprintf(val, "%s\n", img_file_name);
		}

		imshow("pic", img);

		waitKey(10);
		file_counter++;
	}
	fclose(train);
	fclose(val);
	printf("total of %d frames converted\n", file_counter);

	waitKey(50);

	return 0;
}

