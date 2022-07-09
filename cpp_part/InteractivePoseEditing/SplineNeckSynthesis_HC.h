#pragma once
//#include <gbasic/GSkeleton.h>
#include <gbasic/GCamera.h>
#include <gbasic/eigen_utility.h>
#include <opencv2/opencv.hpp>
#include <map>
#include "ZSkeleton.h"
#include "levmar.h"
using namespace std;

#define C_INTERFACE_API __declspec(dllexport)

//optimization user data
struct levmar_UserData
{
	ZSkeleton::Ptr sk_source;  //mskel
	vector<string> joints_names; // joints' names
	vector < cv::Point2i> joints_2d; //joints' 2d coordinate
	GCameraPers view; //camera
	std::map<ZName, unsigned int> index_map; //dof-index
	float joint_rot_smooth_weight; 
	float root_rot_smooth_weight; 
	float root_tran_smooth_weight; 
	vector<float> weights;
	float joint_2d_weight;
	float scale;
	set<string> bone_set;
};

struct optimizeParam
{
	float joint_rot_smooth_weight;
	float root_rot_smooth_weight;
	float root_tran_smooth_weight;
	float joint_2d_weight;

	optimizeParam()
	{
		joint_rot_smooth_weight = 0.01; 
		root_rot_smooth_weight = 0.01; 
		root_tran_smooth_weight = 1.0; 
		joint_2d_weight = 0.1;
	}
};

struct MayaCamera
{
	EVec3f pos_cm;
	EVec3f angle;
	float focal_mm;
	float film_width_inch;
	float film_height_inch;
	int width_pixel;
	int height_pixel;
};

void readMayaCamera(string file, int width, int height, vector<MayaCamera>& cameras);
GCameraPers maycam2gcam(const MayaCamera &mcam);
EMat3f getRot3X3fromAngle(const EVec3f& pose);


//void disval_posesolver(float* p, float* hx, int m, int n, void* data);
//void jacos_posesolver(float* p, float* jac, int m, int n, void* data);
void disval_posesolver_v2(float* p, float* hx, int m, int n, void* data);
void jacos_posesolver_v2(float* p, float* jac, int m, int n, void* data);

void setMSkelPoseByMap(ZSkeleton::Ptr sk_source, const map<string, EVecXf>& pose_map);
void extractMSkelPoseToMap(ZSkeleton::Ptr sk_source, map<string, EVecXf>& pose_map);

////solve by newton-gaussian
//void adjust_by_joints2d_mskel(ZSkeleton::Ptr sk_source, map<string,EVecXf>& pose_map, const map<string, string>& bone_map, const GCameraPers& cur_view, const vector<cv::Point2i>& joints, const vector<string>& joints_names, int flag, float scale, int iterMax);

//solve by levmar (recommended)
/*
input:
sk_source - mskel skeleton
pose_map -  input pose with the format <bone_name, joint angles>
bone_set - which dofs will be optimized
cur_view - camera
joints - joints coordinates
joints_names - joints' names
scale - unite converted to m ( if cm to m, then 0.01, if mm to m, then 0.001)
param - optimization params. ( can be adjusted by the user)

output:
pose_map - output pose with the format <bone_name, joint angles>
*/


//void adjust_by_joints2d_mskel_levmar(ZSkeleton::Ptr sk_source, map<string, EVecXf>& pose_map, const set<string>& bone_set, const GCameraPers& cur_view, const vector<cv::Point2i>& joints, const vector<string>& joints_names, const vector<string>& optmized_bones, float scale, optimizeParam param);
extern "C" {
	C_INTERFACE_API void adjust_by_joints2d_mskel_levmar_v2(ZSkeleton::Ptr sk_source, map<string, EVecXf>& pose_map, const set<string>& bone_set, const GCameraPers& cur_view, const vector<cv::Point2i>& joints, const vector<string>& joints_names, float scale, optimizeParam param);
}