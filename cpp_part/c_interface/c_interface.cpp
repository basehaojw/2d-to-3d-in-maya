#include <iostream>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>

#include <gbasic/GCamera.h>
#include <gbasic/eigen_utility.h>
#include <gbasic/GSkeleton.h>


#include "c_interface.h"
#include "../InteractivePoseEditing/SplineNeckSynthesis_HC.h"


using namespace std;
using namespace nlohmann;

vector<GCameraPers> readCamera(string file, int width, int height, vector<GCameraPers>& cameras_default, vector<EMat4f>& trans)
{
	ifstream is(file.c_str());

	string str;
	for (int i = 0; i < 2; i++)
		getline(is, str);
	//line3 
	getline(is, str);
	vector<string> tokens;
	boost::split(tokens, str, boost::is_any_of(","));

	int startFrame = stoi(tokens[2]);
	int endFrame = stoi(tokens[3]);

	float film_width = stof(tokens[4]) * 25.4; // inch to mm
	float film_height = stof(tokens[5]) * 25.4;

	cout << startFrame << " " << endFrame << " " << film_width << " " << film_height << endl;

	vector<GCameraPers> real_cameras;
	for (int i = startFrame; i <= endFrame; i++)
	{
		getline(is, str);
		boost::split(tokens, str, boost::is_any_of(","));
		EVec3f pos(stof(tokens[1]), stof(tokens[2]), stof(tokens[3]));
		pos = pos * 0.01; //cm to m

		EVec3f angle(stof(tokens[4]), stof(tokens[5]), stof(tokens[6]));

		float focal_mm = stof(tokens[7]);


		float focal_pixel = focal_mm / film_width * width;		cout << i << " " << pos.transpose() << " " << angle.transpose() << " " << focal_mm << " " << focal_pixel << endl;
		GCameraPers camera_default(width, height, 0.1, 10.0, GCameraIntrinsic(focal_pixel, focal_pixel, width / 2, height / 2), GCameraInfo(GVector3(0, 0, 0), GVector3(0, 0, 1), GVector3(0, 1, 0)));
		EMat3f rot = getRot3X3fromAngle(angle);

		EVec3f target = pos + EVec3f(rot(0, 2), rot(1, 2), rot(2, 2)) * (-1);
		EVec3f up = EVec3f(rot(0, 1), rot(1, 1), rot(2, 1));

		GCameraPers camera_real(width, height, 0.1, 10.0, GCameraIntrinsic(focal_pixel, focal_pixel, width / 2, height / 2), GCameraInfo(GVector3(pos.x(), pos.y(), pos.z()), GVector3(target.x(), target.y(), target.z()), GVector3(up.x(), up.y(), up.z())));

		//cout << "rot:" << rot << endl;
		//cout << "target:" << target.transpose() << endl;
		//cout << "up:" << up.transpose() << endl;

		cameras_default.push_back(camera_default);
		real_cameras.push_back(camera_real);

		EMat4f camera_trans = EMat4f::Zero();
		for (int yy = 0; yy < 3; yy++)
		{
			for (int xx = 0; xx < 3; xx++)
			{
				camera_trans(yy, xx) = rot(yy, xx);
			}
		}
		camera_trans(0, 3) = pos.x();
		camera_trans(1, 3) = pos.y();
		camera_trans(2, 3) = pos.z();
		camera_trans(3, 3) = 1.0;

		EVec3f angle2(0, 180, 0);
		EMat3f rot2 = getRot3X3fromAngle(angle2);

		EMat4f camera_trans2 = EMat4f::Zero();
		for (int yy = 0; yy < 3; yy++)
		{
			for (int xx = 0; xx < 3; xx++)
			{
				camera_trans2(yy, xx) = rot2(yy, xx);
			}
		}
		camera_trans2(0, 3) = 0;
		camera_trans2(1, 3) = 0;
		camera_trans2(2, 3) = 0;
		camera_trans2(3, 3) = 1.0;

		EMat4f final_tran = camera_trans * camera_trans2;

		//cout << final_tran << endl;
		trans.push_back(final_tran);
	}

	is.close();

	return real_cameras;
}

namespace
{
	enum RotSeq { zyx, zyz, zxy, zxz, yxz, yxy, yzx, yzy, xyz, xyx, xzy, xzx };

	void threeaxisrot(double r11, double r12, double r21, double r31, double r32, double res[]) {
		res[0] = atan2(r31, r32);
		if (r21 > 1)
			r21 = 1;
		if (r21 < -1)
			r21 = -1;


		res[1] = asin(r21);
		res[2] = atan2(r11, r12);
	}

	void twoaxisrot(double r11, double r12, double r21, double r31, double r32, double res[]) {
		res[0] = atan2(r11, r12);
		res[1] = acos(r21);
		res[2] = atan2(r31, r32);
	}

	void quaternion2Euler(const Eigen::Quaterniond& q, double res[], RotSeq rotSeq)
	{
		switch (rotSeq) {
		case zyx:
			threeaxisrot(2 * (q.x()*q.y() + q.w()*q.z()),
				q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z(),
				-2 * (q.x()*q.z() - q.w()*q.y()),
				2 * (q.y()*q.z() + q.w()*q.x()),
				q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z(),
				res);
			break;

		case zyz:
			twoaxisrot(2 * (q.y()*q.z() - q.w()*q.x()),
				2 * (q.x()*q.z() + q.w()*q.y()),
				q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z(),
				2 * (q.y()*q.z() + q.w()*q.x()),
				-2 * (q.x()*q.z() - q.w()*q.y()),
				res);
			break;

		case zxy:
			threeaxisrot(-2 * (q.x()*q.y() - q.w()*q.z()),
				q.w()*q.w() - q.x()*q.x() + q.y()*q.y() - q.z()*q.z(),
				2 * (q.y()*q.z() + q.w()*q.x()),
				-2 * (q.x()*q.z() - q.w()*q.y()),
				q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z(),
				res);
			break;

		case zxz:
			twoaxisrot(2 * (q.x()*q.z() + q.w()*q.y()),
				-2 * (q.y()*q.z() - q.w()*q.x()),
				q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z(),
				2 * (q.x()*q.z() - q.w()*q.y()),
				2 * (q.y()*q.z() + q.w()*q.x()),
				res);
			break;

		case yxz:
			threeaxisrot(2 * (q.x()*q.z() + q.w()*q.y()),
				q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z(),
				-2 * (q.y()*q.z() - q.w()*q.x()),
				2 * (q.x()*q.y() + q.w()*q.z()),
				q.w()*q.w() - q.x()*q.x() + q.y()*q.y() - q.z()*q.z(),
				res);
			break;

		case yxy:
			twoaxisrot(2 * (q.x()*q.y() - q.w()*q.z()),
				2 * (q.y()*q.z() + q.w()*q.x()),
				q.w()*q.w() - q.x()*q.x() + q.y()*q.y() - q.z()*q.z(),
				2 * (q.x()*q.y() + q.w()*q.z()),
				-2 * (q.y()*q.z() - q.w()*q.x()),
				res);
			break;

		case yzx:
			threeaxisrot(-2 * (q.x()*q.z() - q.w()*q.y()),
				q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z(),
				2 * (q.x()*q.y() + q.w()*q.z()),
				-2 * (q.y()*q.z() - q.w()*q.x()),
				q.w()*q.w() - q.x()*q.x() + q.y()*q.y() - q.z()*q.z(),
				res);
			break;

		case yzy:
			twoaxisrot(2 * (q.y()*q.z() + q.w()*q.x()),
				-2 * (q.x()*q.y() - q.w()*q.z()),
				q.w()*q.w() - q.x()*q.x() + q.y()*q.y() - q.z()*q.z(),
				2 * (q.y()*q.z() - q.w()*q.x()),
				2 * (q.x()*q.y() + q.w()*q.z()),
				res);
			break;

		case xyz:
			threeaxisrot(-2 * (q.y()*q.z() - q.w()*q.x()),
				q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z(),
				2 * (q.x()*q.z() + q.w()*q.y()),
				-2 * (q.x()*q.y() - q.w()*q.z()),
				q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z(),
				res);
			break;

		case xyx:
			twoaxisrot(2 * (q.x()*q.y() + q.w()*q.z()),
				-2 * (q.x()*q.z() - q.w()*q.y()),
				q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z(),
				2 * (q.x()*q.y() - q.w()*q.z()),
				2 * (q.x()*q.z() + q.w()*q.y()),
				res);
			break;

		case xzy:
			threeaxisrot(2 * (q.y()*q.z() + q.w()*q.x()),
				q.w()*q.w() - q.x()*q.x() + q.y()*q.y() - q.z()*q.z(),
				-2 * (q.x()*q.y() - q.w()*q.z()),
				2 * (q.x()*q.z() + q.w()*q.y()),
				q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z(),
				res);
			break;

		case xzx:
			twoaxisrot(2 * (q.x()*q.z() - q.w()*q.y()),
				2 * (q.x()*q.y() + q.w()*q.z()),
				q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z(),
				2 * (q.x()*q.z() + q.w()*q.y()),
				-2 * (q.x()*q.y() - q.w()*q.z()),
				res);
			break;
		default:
			std::cout << "Unknown rotation sequence" << std::endl;
			break;
		}
	}
}

void sps2amc(const ZSkeleton::Ptr sk_source, const GSkeleton& sk, const  map<string, string>& bone_map, const float &scale, GPoseF& out_pose)
{
	out_pose.resize(sk.NumFreedoms(), 0);

	//process root
	out_pose[0] = sk_source->getRoot()->getTranslation().x() * scale;
	out_pose[1] = sk_source->getRoot()->getTranslation().y() * scale;
	out_pose[2] = sk_source->getRoot()->getTranslation().z() * scale;

	EMat3d rot = sk_source->getRoot()->worldMatrix().linear();
	EMat3d jo = sk_source->getRoot()->JOMatrix().linear();

	/*cout << rot << endl;
	cout << jo << endl;*/

	EMat3d amc_rot = rot * jo.inverse();

	Eigen::Quaterniond q(amc_rot);

	double euler[3];
	quaternion2Euler(q, euler, zyx);

	out_pose[3] = euler[0] / M_PI * 180.0;
	out_pose[4] = euler[1] / M_PI * 180.0;
	out_pose[5] = euler[2] / M_PI * 180.0;

	for (auto iter = bone_map.begin(); iter != bone_map.end(); iter++)
	{
		if (!sk_source->getJointByName(iter->first)->getParent()) //root already processed
			continue;
		ZMath::ZVector3D angle = sk_source->getJointByName(iter->first)->getRotate();
		if (sk.findBone(iter->second)->b2ja[0] != -1)
			out_pose[sk.findBone(iter->second)->b2ja[0]] = angle.x() / M_PI * 180;
		else
			cout << iter->second << " does not have dof 0" << endl;
		if (sk.findBone(iter->second)->b2ja[1] != -1)
			out_pose[sk.findBone(iter->second)->b2ja[1]] = angle.y() / M_PI * 180;
		else
			cout << iter->second << " does not have dof 1" << endl;

		if (sk.findBone(iter->second)->b2ja[2] != -1)
			out_pose[sk.findBone(iter->second)->b2ja[2]] = angle.z() / M_PI * 180;
		else
			cout << iter->second << " does not have dof 2" << endl;

	}
}



static ZSkeleton::Ptr sk_source;
static GSkeleton sk;
static map<string, string> mskel2asf_bonemap; //for visualization


void init_video_maker(char* yaml_path) {
	// string _yaml_file_path("joint2d_mskel.yaml");
	string _yaml_file_path(yaml_path);
	YAML::Node config = YAML::LoadFile(_yaml_file_path);

	string sk_file = config["sk_file"].as<string>();
	string mskel_file = config["mskel_file"].as<string>();
	string sps_file = config["sps_file"].as<string>();


	bool ret = sk.LoadASF(sk_file);
	if (!ret) {
		cout << "init sk error";
		return;
	}
	sk.scaleSKSelf(0.001);

	sk_source = std::make_shared<ZSkeleton>("liubo");

	sk_source->loadSK(mskel_file);

	sk_source->loadMotion(sps_file);

	mskel2asf_bonemap.insert(std::pair<string, string>("root", "root"));
	mskel2asf_bonemap.insert(std::pair<string, string>("spline1", "spline1-spline2"));
	mskel2asf_bonemap.insert(std::pair<string, string>("spline2", "spline2-spline3"));
	mskel2asf_bonemap.insert(std::pair<string, string>("spline3", "spline3-spline4"));
	mskel2asf_bonemap.insert(std::pair<string, string>("neck1", "neck1-neck2"));
	mskel2asf_bonemap.insert(std::pair<string, string>("neck2", "neck2-head"));
	mskel2asf_bonemap.insert(std::pair<string, string>("head", "head-head_end"));

}


void optimize_frame(c_pose* head, c_pose* neck1, c_pose* neck2, c_pose* root, c_pose* spline1, c_pose* spline2, c_pose* spline3,
	int mm_cm_m, maya_param* mparam, return_pose* repose) {

	map<string, EVecXf> pose_map;
	EVecXf head_pose = EVecXf::Zero(3);
	head_pose[0] = head->p0;
	head_pose[1] = head->p1;
	head_pose[2] = head->p2;
	//std::cout << "head " << head_pose[0] << " " << head_pose[1] << " " << head_pose[2] << "\n";
	pose_map["head"] = head_pose;
	// ========================
	EVecXf neck1_pose = EVecXf::Zero(3);
	neck1_pose[0] = neck1->p0;
	neck1_pose[1] = neck1->p1;
	neck1_pose[2] = neck1->p2;
	//std::cout << "neck1 " << neck1_pose[0] << " " << neck1_pose[1] << " " << neck1_pose[2] << "\n";
	pose_map["neck1"] = neck1_pose;
	// =============================
	EVecXf neck2_pose = EVecXf::Zero(3);
	neck2_pose[0] = neck2->p0;
	neck2_pose[1] = neck2->p1;
	neck2_pose[2] = neck2->p2;
	//std::cout << "neck2 " << neck2_pose[0] << " " << neck2_pose[1] << " " << neck2_pose[2] << "\n";
	pose_map["neck2"] = neck2_pose;
	// =============================
	EVecXf root_pose = EVecXf::Zero(6);
	root_pose[0] = root->p0;
	root_pose[1] = root->p1;
	root_pose[2] = root->p2;
	root_pose[3] = root->p3;
	root_pose[4] = root->p4;
	root_pose[5] = root->p5;
	//std::cout << "root " << root_pose[0] << " " << root_pose[1] << " " << root_pose[2] << " " << root_pose[3] << " " << root_pose[4] << " " << root_pose[5] << "\n";
	pose_map["root"] = root_pose;
	// =============================
	EVecXf spline1_pose = EVecXf::Zero(3);
	spline1_pose[0] = spline1->p0;
	spline1_pose[1] = spline1->p1;
	spline1_pose[2] = spline1->p2;
	//std::cout << "spline1 " << spline1_pose[0] << " " << spline1_pose[1] << " " << spline1_pose[2] << "\n";
	pose_map["spline1"] = spline1_pose;
	// =============================
	EVecXf spline2_pose = EVecXf::Zero(3);
	spline2_pose[0] = spline2->p0;
	spline2_pose[1] = spline2->p1;
	spline2_pose[2] = spline2->p2;
	//std::cout << "spline2 " << spline2_pose[0] << " " << spline2_pose[1] << " " << spline2_pose[2] << "\n";
	pose_map["spline2"] = spline2_pose;
	// =============================
	EVecXf spline3_pose = EVecXf::Zero(3);
	spline3_pose[0] = spline3->p0;
	spline3_pose[1] = spline3->p1;
	spline3_pose[2] = spline3->p2;
	//std::cout << "spline3 " << spline3_pose[0] << " " << spline3_pose[1] << " " << spline3_pose[2] << "\n";
	pose_map["spline3"] = spline3_pose;
	// =============================

	float scale;
	if (mm_cm_m == 0) {
		// mm
		scale = 0.001;
	}
	else if (mm_cm_m == 1) {
		// cm
		scale = 0.01;
	}else {
		scale = 1.0;
	}
	//std::cout << "scale "<< scale<<"\n";
	
	setMSkelPoseByMap(sk_source, pose_map);
	GPoseF cur_pose;
	sps2amc(sk_source, sk, mskel2asf_bonemap, 0.001, cur_pose);
	sk.UpdatePose(cur_pose, false);


	cout << "before optmize:" << sk_source->getRoot()->getTranslation().transpose() << endl;

	// =====================================
	string pos_cm_str(mparam->pos_cm);
	vector<string> pos_cm_vec;
	boost::split(pos_cm_vec, pos_cm_str, boost::is_any_of(", "), boost::token_compress_on);
	EVecXf pos_cm_float_vec = EVecXf::Zero(3);
	for (int i = 0; i < pos_cm_vec.size(); i++) {
		pos_cm_float_vec[i] = std::stof(pos_cm_vec[i]);
	}

	string angle_str(mparam->angle);
	vector<string> angle_vec;
	boost::split(angle_vec, angle_str, boost::is_any_of(", "), boost::token_compress_on);
	EVecXf angle_float_vec = EVecXf::Zero(3);
	for (int i = 0; i < angle_vec.size(); i++) {
		angle_float_vec[i] = std::stof(angle_vec[i]);
	}

	MayaCamera cur_maya_camera;
	cur_maya_camera.pos_cm = pos_cm_float_vec;
	cur_maya_camera.angle = angle_float_vec;

	cur_maya_camera.focal_mm = mparam->focal_mm;
	cur_maya_camera.film_width_inch = mparam->film_width_inch;
	cur_maya_camera.film_height_inch = mparam->film_height_inch;
	cur_maya_camera.width_pixel = mparam->width_pixel;
	cur_maya_camera.height_pixel = mparam->height_pixel;
	//std::cout << "angle " << cur_maya_camera.angle[0] << " " << cur_maya_camera.angle[1] << " " << cur_maya_camera.angle[2] << "\n";
	//std::cout << "pos_cm " << cur_maya_camera.pos_cm[0] << " " << cur_maya_camera.pos_cm[1] << " " << cur_maya_camera.pos_cm[2] << "\n";
	//std::cout << "film_height_inch " << cur_maya_camera.film_height_inch << "\n";
	//std::cout << "film_width_inch " << cur_maya_camera.film_width_inch << "\n";
	//std::cout << "focal_mm " << cur_maya_camera.focal_mm << "\n";
	//std::cout << "height_pixel " << cur_maya_camera.height_pixel<<" "<< mparam->height_pixel << "\n";
	//std::cout << "width_pixel " << cur_maya_camera.width_pixel <<" " << mparam->width_pixel << "\n";
	//convert maya cam to GCameraPers
	GCameraPers cur_cam = maycam2gcam(cur_maya_camera);
	// =====================================

	//adjust_by_joints2d_mskel(sk_source, i, mskel2asf_bonemap, tmp_real_cameras[i], joints, joint_names_selected,  flag, 10);

	//input joints 2d coordinate
	vector<cv::Point2i> joints;
	//joints.push_back(cv::Point2i(lshoulder_2d.x + 2, lshoulder_2d.y + 2));
	//joints.push_back(cv::Point2i(rshoulder_2d.x + 2, rshoulder_2d.y + 2));
	std::cout << "joints: \n";
	vector<string> joint_pair_vector;
	string joints_string(mparam->joints);
	boost::split(joint_pair_vector, joints_string, boost::is_any_of("; "), boost::token_compress_on);
	for (int i = 0; i < joint_pair_vector.size(); i++) {
		vector<string> joint_x_y;
		boost::split(joint_x_y, joint_pair_vector[i], boost::is_any_of(", "), boost::token_compress_on);
		//std::cout << joint_x_y[0] << " " << joint_x_y[1] << "\n";
		joints.push_back(cv::Point2i(std::stoi(joint_x_y[0]), std::stoi(joint_x_y[1])));
	}

	// input joints 2d's corresponding names
	vector<string> cur_joint_names_selected;
	//cur_joint_names_selected.push_back("lhumerus");
	//cur_joint_names_selected.push_back("rhumerus");
	std::string cur_joint_names_selected_s(mparam->selected_joints_name);
	boost::split(cur_joint_names_selected, cur_joint_names_selected_s, boost::is_any_of(", "), boost::token_compress_on);
	std::cout << "cur_joint_names_selected: \n";
	for (int i = 0; i < cur_joint_names_selected.size(); i++) {
		//std::cout << cur_joint_names_selected[i] << "\n";
	}

	set<string> cur_mskel_bones; //bones that need to be optimized;
	//cur_mskel_bones.insert("root");
	//cur_mskel_bones.insert("spline1");
	//cur_mskel_bones.insert("spline2");
	//cur_mskel_bones.insert("spline3");
	std::vector<std::string> cur_mskel_bones_names;
	boost::split(cur_mskel_bones_names, mparam->cur_mskel_bones_name, boost::is_any_of(","), boost::token_compress_on);
	std::cout << "cur_mskel_bones: \n";
	for (int i = 0; i < cur_mskel_bones_names.size(); i++) {
		//std::cout << cur_mskel_bones_names[i] << "\n";
		cur_mskel_bones.insert(cur_mskel_bones_names[i]);
	}


	//mskel_poses[i] is the current pose( should from maya)
	adjust_by_joints2d_mskel_levmar_v2(sk_source, pose_map, cur_mskel_bones, cur_cam, joints, cur_joint_names_selected, scale, optimizeParam());
	repose->head.p0 = pose_map["head"][0];
	repose->head.p1 = pose_map["head"][1];
	repose->head.p2 = pose_map["head"][2];
	// =======================
	repose->neck1.p0 = pose_map["neck1"][0];
	repose->neck1.p1 = pose_map["neck1"][1];
	repose->neck1.p2 = pose_map["neck1"][2];
	// =======================
	repose->neck2.p0 = pose_map["neck2"][0];
	repose->neck2.p1 = pose_map["neck2"][1];
	repose->neck2.p2 = pose_map["neck2"][2];
	// =======================
	repose->root.p0 = pose_map["root"][0];
	repose->root.p1 = pose_map["root"][1];
	repose->root.p2 = pose_map["root"][2];
	repose->root.p3 = pose_map["root"][3];
	repose->root.p4 = pose_map["root"][4];
	repose->root.p5 = pose_map["root"][5];
	// =======================
	repose->spline1.p0 = pose_map["spline1"][0];
	repose->spline1.p1 = pose_map["spline1"][1];
	repose->spline1.p2 = pose_map["spline1"][2];
	// =======================
	repose->spline2.p0 = pose_map["spline2"][0];
	repose->spline2.p1 = pose_map["spline2"][1];
	repose->spline2.p2 = pose_map["spline2"][2];
	// =======================
	repose->spline3.p0 = pose_map["spline3"][0];
	repose->spline3.p1 = pose_map["spline3"][1];
	repose->spline3.p2 = pose_map["spline3"][2];
	// =======================
}


void test_py_arguments(c_pose* pose, char* p) {
	std::cout << pose->p0 << " " << pose->p1 << " " << pose->p2 << " " << pose->p3 << " " << pose->p4 << " " << pose->p5 << "\n";
	string sp(p);
	std::cout << sp << "\n";
	return;
}

// =====================haojw test========================
void test_init_video_maker_haojw(char* mskel_path) {
	
	string mskel_file = mskel_path;
	sk_source = std::make_shared<ZSkeleton>("haojw");
	sk_source->loadSK(mskel_file);

}

void test_func() {
	float scale = 0.01;
	// camera
	EVec3f pos(159.63264374229792, 149.34348478139492, 751.329038022193);
	EVec3f angle(-2.742, -21.093, -2.879);
	float focal = 65.967;
	float film_width = 0.9877110803749809;
	float film_height= 0.5555874827109267;
	int width_pixel = 2112;
	int height_pixel = 1188;
	
	// joint parameters
	map<string, EVecXf> pose_map;
	
	EVecXf joint_pos_1 = EVecXf::Zero(6);
	joint_pos_1[0] = 486.0021474979542;
	joint_pos_1[1] = 100.59616113150561;
	joint_pos_1[2] = -95.07569896512562;
	joint_pos_1[3] = -45.37092857780281;
	joint_pos_1[4] = -3.6184764071747066;
	joint_pos_1[5] = 3.219924778320171;
	EVecXf joint_pos_2 = EVecXf::Zero(3);
	joint_pos_2[0] = 0.0;
	joint_pos_2[1] = 0.0;
	joint_pos_2[2] = 0.0;
	EVecXf joint_pos_3 = EVecXf::Zero(3);
	joint_pos_3[0] = 0.0;
	joint_pos_3[1] = 0.0;
	joint_pos_3[2] = 0.0;
	EVecXf joint_pos_4 = EVecXf::Zero(3);
	joint_pos_4[0] = 5.018;
	joint_pos_4[1] = 0.847;
	joint_pos_4[2] = 2.175;
	EVecXf joint_pos_5 = EVecXf::Zero(3);
	joint_pos_5[0] = 4.999;
	joint_pos_5[1] = 0.767;
	joint_pos_5[2] = 2.282;
	EVecXf joint_pos_6 = EVecXf::Zero(3);
	joint_pos_6[0] = 4.775;
	joint_pos_6[1] = 1.784;
	joint_pos_6[2] = 2.438;
	pose_map["Chest_M"] = joint_pos_1;
	pose_map["Scapula_R"] = joint_pos_2;
	pose_map["Scapula_L"] = joint_pos_3;
	pose_map["Neck_M"] = joint_pos_4;
	pose_map["Neck1_M"] = joint_pos_5;
	pose_map["Head_M"] = joint_pos_6;
	
	set<string> cur_mskel_bones; //bones that need to be optimized;
	cur_mskel_bones.insert("Chest_M");
	//cur_mskel_bones.insert("Neck1_M");
	//cur_mskel_bones.insert("Head_M");

	
	vector<cv::Point2i> joints; // 2d joints
	
	joints.push_back(cv::Point2i(1206.5566527973108, 320.71911800615726));
	joints.push_back(cv::Point2i(1219.739354035548, 352.1153445572886));

	vector<string> cur_joint_names_selected;
	
	cur_joint_names_selected.push_back("Head_M");
	cur_joint_names_selected.push_back("Neck1_M");

	//=======================================
	MayaCamera cur_maya_camera;
	cur_maya_camera.pos_cm = pos;
	cur_maya_camera.angle = angle;
	cur_maya_camera.focal_mm = focal;
	cur_maya_camera.film_width_inch = film_width;
	cur_maya_camera.film_height_inch = film_height;
	cur_maya_camera.width_pixel = width_pixel;
	cur_maya_camera.height_pixel = height_pixel;

	GCameraPers cur_cam = maycam2gcam(cur_maya_camera);
	
	adjust_by_joints2d_mskel_levmar_v2(sk_source, pose_map, cur_mskel_bones, cur_cam, joints, cur_joint_names_selected, scale, optimizeParam());
	
	std::cout << "Chest_M: " << pose_map["Chest_M"] << "\n";
	std::cout << "Scapula_R" << pose_map["Scapula_R"] << "\n";
	std::cout << "Scapula_L" << pose_map["Scapula_L"] << "\n";
	std::cout << "Neck_M" << pose_map["Neck_M"] << "\n";
	std::cout << "Neck1_M" << pose_map["Neck1_M"] << "\n";
	std::cout << "Head_M" << pose_map["Head_M"] << "\n";
}

void test_projection_haojw(int mm_cm_m, maya_param* mparam, return_pose_haojw* result)
{	
	std::cout << "projection----haojw" << "\n";
	// scale
	float scale;
	if (mm_cm_m == 0) {
		// mm
		scale = 0.001;
	}
	else if (mm_cm_m == 1) {
		// cm
		scale = 0.01;
	}
	else {
		scale = 1.0;
	}
	// camera part
	string pos_cm_str(mparam->pos_cm);
	vector<string> pos_cm_vec;
	boost::split(pos_cm_vec, pos_cm_str, boost::is_any_of(", "), boost::token_compress_on);
	EVecXf pos_cm_float_vec = EVecXf::Zero(3);
	for (int i = 0; i < pos_cm_vec.size(); i++) {
		pos_cm_float_vec[i] = std::stof(pos_cm_vec[i]);
	}

	string angle_str(mparam->angle);
	vector<string> angle_vec;
	boost::split(angle_vec, angle_str, boost::is_any_of(", "), boost::token_compress_on);
	EVecXf angle_float_vec = EVecXf::Zero(3);
	for (int i = 0; i < angle_vec.size(); i++) {
		angle_float_vec[i] = std::stof(angle_vec[i]);
	}

	MayaCamera cur_maya_camera;
	cur_maya_camera.pos_cm = pos_cm_float_vec;
	cur_maya_camera.angle = angle_float_vec;

	cur_maya_camera.focal_mm = mparam->focal_mm;
	cur_maya_camera.film_width_inch = mparam->film_width_inch;
	cur_maya_camera.film_height_inch = mparam->film_height_inch;
	cur_maya_camera.width_pixel = mparam->width_pixel;
	cur_maya_camera.height_pixel = mparam->height_pixel;

	GCameraPers cur_cam = maycam2gcam(cur_maya_camera);

	// ============================================
	vector<cv::Point3f> joints;

	vector<string> joint_pair_vector;
	
	string joints_string(mparam->joints);

	boost::split(joint_pair_vector, joints_string, boost::is_any_of("; "), boost::token_compress_on);

	for (int i = 0; i < joint_pair_vector.size(); i++) {
		vector<string> joint_x_y_z;
		boost::split(joint_x_y_z, joint_pair_vector[i], boost::is_any_of(", "), boost::token_compress_on);
		joints.push_back(cv::Point3f(std::stof(joint_x_y_z[0]), std::stof(joint_x_y_z[1]), std::stof(joint_x_y_z[2])));
	}

	vector<string> cur_joint_names_selected;
	std::string cur_joint_names_selected_s(mparam->selected_joints_name);
	boost::split(cur_joint_names_selected, cur_joint_names_selected_s, boost::is_any_of(", "), boost::token_compress_on);
	
	//std::cout << "joints: " << joints << "\n";
	//std::cout << joints. << joints[1] << joints[2] << "\n";
	// ============================================
	json json_result;

	for (int j = 0; j < joints.size(); j++)
	{

		string joint_name;

		joint_name = cur_joint_names_selected[j];
		// std::cout << joint_name << "\n";

		//EVec3f cur_pos = sk_source->getJointByName(joint_name)->worldMatrix().translation().cast<float>() * scale;
		
		GVector2 cur_pos_2d = cur_cam.Project(GVector3(joints[j].x * scale, joints[j].y * scale, joints[j].z * scale));

		//std::cout << "joint_name: " << joint_name << "----" << cur_pos_2d << "\n";

		string joint_name_x = joint_name + "_x";
		string joint_name_y = joint_name + "_y";
		json_result[joint_name_x] = cur_pos_2d.x;
		json_result[joint_name_y] = cur_pos_2d.y;
		
	}
	string re_str = json_result.dump();
	//std::cout << "projection result: " << re_str;
	int len = strlen(re_str.c_str());
	char *fn_result = new char[len + 1];
	strcpy_s(fn_result, len + 1, re_str.c_str());

	result->result = fn_result;
}

void test_opt_haojw(char *json_str, int mm_cm_m, maya_param* mparam, return_pose_haojw* result)
{
	std::cout << "========-------------OPT-INFO-------------========" << "\n";
	map<string, EVecXf> pose_map;

	json j;
	j = json::parse(json_str);
	std::cout << "haojw_json_parameters: " << j << "\n";


	for (auto&i : j.items())
	{
		string joint_name = i.key();
		auto joint_value = i.value();
		ZJoint::Ptr jnt = sk_source->getJointByName(joint_name);

		if (!jnt->getParent())
		{
			EVecXf joint_pos = EVecXf::Zero(6);
			joint_pos[0] = joint_value[0].get<float>();
			joint_pos[1] = joint_value[1].get<float>();
			joint_pos[2] = joint_value[2].get<float>();
			joint_pos[3] = joint_value[3].get<float>();
			joint_pos[4] = joint_value[4].get<float>();
			joint_pos[5] = joint_value[5].get<float>();

			pose_map.insert(std::pair<string, EVecXf>(joint_name, joint_pos));
		}

		else
		{
			EVecXf joint_pos = EVecXf::Zero(3);

			joint_pos[0] = joint_value[0].get<float>();
			joint_pos[1] = joint_value[1].get<float>();
			joint_pos[2] = joint_value[2].get<float>();

			pose_map.insert(std::pair<string, EVecXf>(joint_name, joint_pos));
		}
	}

	float scale;
	if (mm_cm_m == 0) {
		// mm
		scale = 0.001;
	}
	else if (mm_cm_m == 1) {
		// cm
		scale = 0.01;
	}
	else {
		scale = 1.0;
	}
	
	setMSkelPoseByMap(sk_source, pose_map);
	GPoseF cur_pose;
	
	cout << "before optmize:" << sk_source->getRoot()->getTranslation().transpose() << endl;
	//========================================

	string pos_cm_str(mparam->pos_cm);
	vector<string> pos_cm_vec;
	boost::split(pos_cm_vec, pos_cm_str, boost::is_any_of(", "), boost::token_compress_on);
	EVecXf pos_cm_float_vec = EVecXf::Zero(3);
	for (int i = 0; i < pos_cm_vec.size(); i++) {
		pos_cm_float_vec[i] = std::stof(pos_cm_vec[i]);
	}

	string angle_str(mparam->angle);
	vector<string> angle_vec;
	boost::split(angle_vec, angle_str, boost::is_any_of(", "), boost::token_compress_on);
	EVecXf angle_float_vec = EVecXf::Zero(3);
	for (int i = 0; i < angle_vec.size(); i++) {
		angle_float_vec[i] = std::stof(angle_vec[i]);
	}

	MayaCamera cur_maya_camera;
	cur_maya_camera.pos_cm = pos_cm_float_vec;
	cur_maya_camera.angle = angle_float_vec;

	cur_maya_camera.focal_mm = mparam->focal_mm;
	cur_maya_camera.film_width_inch = mparam->film_width_inch;
	cur_maya_camera.film_height_inch = mparam->film_height_inch;
	cur_maya_camera.width_pixel = mparam->width_pixel;
	cur_maya_camera.height_pixel = mparam->height_pixel;

	GCameraPers cur_cam = maycam2gcam(cur_maya_camera);

	//===========================================
	//input joints 2d coordinate
	vector<cv::Point2i> joints;
	
	
	vector<string> joint_pair_vector;
	string joints_string(mparam->joints);
	std::cout << "2d joints: " << joints_string << "\n";
	boost::split(joint_pair_vector, joints_string, boost::is_any_of("; "), boost::token_compress_on);
	for (int i = 0; i < joint_pair_vector.size(); i++) {
		vector<string> joint_x_y;
		boost::split(joint_x_y, joint_pair_vector[i], boost::is_any_of(", "), boost::token_compress_on);
		//std::cout << joint_x_y[0] << " " << joint_x_y[1] << "\n";
		joints.push_back(cv::Point2i(std::stof(joint_x_y[0]), std::stof(joint_x_y[1])));
	}
		
	vector<string> cur_joint_names_selected;
	std::string cur_joint_names_selected_s(mparam->selected_joints_name);
	boost::split(cur_joint_names_selected, cur_joint_names_selected_s, boost::is_any_of(", "), boost::token_compress_on);
	std::cout << "2d_joint_names: ";
	for (int i = 0; i < cur_joint_names_selected.size(); i++) {
		std::cout << cur_joint_names_selected[i] << ", ";
	}
	std::cout << "\n";
	set<string> cur_mskel_bones; //bones that need to be optimized;
	std::vector<std::string> cur_mskel_bones_names;
	boost::split(cur_mskel_bones_names, mparam->cur_mskel_bones_name, boost::is_any_of(","), boost::token_compress_on);
	std::cout << "cur_mskel_bones: ";
	for (int i = 0; i < cur_mskel_bones_names.size(); i++) {
		std::cout << cur_mskel_bones_names[i] << " ";
		cur_mskel_bones.insert(cur_mskel_bones_names[i]);
	}
	std::cout << "\n";
	adjust_by_joints2d_mskel_levmar_v2(sk_source, pose_map, cur_mskel_bones, cur_cam, joints, cur_joint_names_selected, scale, optimizeParam());
	cout << "after optmize:" << sk_source->getRoot()->getTranslation().transpose() << endl;
	json json_temp;

	/*for (auto&i : j.items()) {
		string joint_name = i.key();
		json_temp[joint_name][0] = pose_map[joint_name][0];
		json_temp[joint_name][1] = pose_map[joint_name][1];
		json_temp[joint_name][2] = pose_map[joint_name][2];
		json_temp[joint_name][3] = pose_map[joint_name][3];
		json_temp[joint_name][4] = pose_map[joint_name][4];
		json_temp[joint_name][5] = pose_map[joint_name][5];
	}*/
	
	for (auto&i : j.items()) {
		string joint_name = i.key();
		ZJoint::Ptr jnt = sk_source->getJointByName(joint_name);
		
		if (!jnt->getParent())
		{
			
			json_temp[joint_name][0] = pose_map[joint_name][0];
			json_temp[joint_name][1] = pose_map[joint_name][1];
			json_temp[joint_name][2] = pose_map[joint_name][2];
			json_temp[joint_name][3] = pose_map[joint_name][3];
			json_temp[joint_name][4] = pose_map[joint_name][4];
			json_temp[joint_name][5] = pose_map[joint_name][5];
		}
		else
		{
			json_temp[joint_name][0] = pose_map[joint_name][0];
			json_temp[joint_name][1] = pose_map[joint_name][1];
			json_temp[joint_name][2] = pose_map[joint_name][2];
		}
	}
	
	
	string re_str = json_temp.dump();
	std::cout << "result_str: " << re_str << std::endl;
	int len = strlen(re_str.c_str());
	char *fn_result = new char[len + 1];
	strcpy_s(fn_result, len+1, re_str.c_str());
	
	result->result = fn_result;
	//result->result = const_cast<char*>(re_str.c_str());

	//repose = const_cast<char*>(re_str.c_str());

	//repose = (char *)json_temp.dump().c_str();

}

int main() {
	string re_str = "C:/Users/admin/haojw/GitLab/maya/PLE-22602/target_tool/mskel/Ling_BNRF_Tpose.mskel";

	
	//test_init_video_maker_haojw(re_str);
	test_func();
	std::cin.get();
}
