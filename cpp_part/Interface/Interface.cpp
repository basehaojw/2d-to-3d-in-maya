// VideoMaker.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#include <iostream>
#include <gbasic/OpenGLInitializer.h>
#include <opencv2/opencv.hpp>
#include <time.h>
#include <string>
using namespace std;
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>

#include <gbasic/eigen_utility.h>
#include <yaml-cpp/yaml.h>


#include <gbasic/CodeTimerSync.h>
#include <gbasic/GSkeleton.h>
#include <gbasic/AMCMotion.h>
#include <gbasic/GCamera.h>
#include <gbasic/GMesh.h>
#include <gbasic/GMeshIO.h>
#include <gbasic/LinearBlendSkining.h>
#include <gbasic/GSkeletonDrawerStick.h>
#include <gbasic/GMeshDrawer.h>
#include <gbasiccuda/FBORender.h>
#include "ZSkeleton.h"
#include "SplineNeckSynthesis_HC.h"



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
std::vector<std::string> get_file_list(const std::string& path, const std::string& extension)
{
	std::vector<std::string> m_file_list;
	if (!path.empty())
	{
		namespace fs = boost::filesystem;

		fs::path apk_path(path);
		fs::recursive_directory_iterator end;

		for (fs::recursive_directory_iterator i(apk_path); i != end; ++i)
		{
			const fs::path cp = (*i);
			string cur_extension = fs::extension(cp);

			if (cur_extension == extension)
				m_file_list.push_back(cp.string());
		}
	}
	return m_file_list;
}




struct Joint
{
	int id;
	string name;
	int pid; //parent id
	vector<int> cids; //children id;
	EVec3f translation, jo;
	EMat4f LC, WC;
	bool is_end_effector;
	vector<pair<float, float>> joint_limts;

	Joint()
	{
		id = -1;
		pid = -1;
		translation = EVec3f::Zero();
		jo = EVec3f::Zero();
		LC = EMat4f::Identity();
		WC = EMat4f::Identity();
		is_end_effector = false;
	}
};

struct BoneHierData
{
	string pname;
	string cname;
	string name;
};

//EMat3f getRot3X3fromAngle(const EVec3f& pose)
//{
//	Eigen::Isometry3f transZ(Eigen::AngleAxisf(pose[2] / 180 * M_PI, Eigen::Vector3f(0, 0, 1)));
//	Eigen::Isometry3f transY(Eigen::AngleAxisf(pose[1] / 180 * M_PI, Eigen::Vector3f(0, 1, 0)));
//	Eigen::Isometry3f transX(Eigen::AngleAxisf(pose[0] / 180 * M_PI, Eigen::Vector3f(1, 0, 0)));
//
//	EMat3f rot = transZ.linear() * transY.linear() * transX.linear();
//
//	return rot;
//}

void mskel2asf(string mskel_file, float scale)
{
	vector<Joint> joints_list;
	map<string, int> name_map;
	std::ifstream fin(mskel_file.c_str());
	if (!fin.is_open())
	{
		cout << mskel_file << " does not exist";
		return;
	}
	std::string cur_line;
	int line_num = 0;
	while (std::getline(fin, cur_line))
	{
		line_num++;
		boost::trim(cur_line);
		if (boost::to_lower_copy(cur_line) == "mskel")
		{
			continue;
		}
		if (cur_line.empty() || cur_line[0] == '#')
		{
			continue;
		}
		std::vector<std::string> split_character;
		boost::split(split_character, cur_line, boost::is_any_of("\t "), boost::token_compress_on);
		if (split_character.size() == 2 && split_character[0] == "joint")
		{
			int m_bone_num = std::stoi(split_character[1]);
			for (unsigned int ii = 0; ii < m_bone_num; ii++)
			{
				if (std::getline(fin, cur_line))
				{
					line_num++;
					boost::trim(cur_line);
					if (cur_line.empty() || cur_line[0] == '#')
					{
						continue;
					}
					std::vector<std::string> split_joints;
					boost::split(split_joints, cur_line, boost::is_any_of("\t "), boost::token_compress_on);
					if (split_joints.size() < 20)
					{
						cout << mskel_file << " parsing MSKel error at line:" << line_num;
						return;
					}

					Joint joint;
					joint.name = split_joints[0];
					joint.id = ii;
					joint.translation = EVec3f(std::stof(split_joints[2]), std::stof(split_joints[3]), std::stof(split_joints[4]));
					joint.jo = EVec3f(std::stof(split_joints[5]), std::stof(split_joints[6]), std::stof(split_joints[7]));


					name_map.insert(pair<string, int>(joint.name, joint.id));

					joints_list.push_back(joint);


				}
				else
				{
					cout << mskel_file << " MSKel num not correct";
					return;
				}
			}
		}
		if (split_character.size() == 1 && split_character[0] == "hierarchy")
		{
			while (std::getline(fin, cur_line))
			{
				line_num++;
				boost::trim(cur_line);
				if (cur_line.empty() || cur_line[0] == '#')
				{
					continue;
				}
				std::vector<std::string> split_hierachy;
				boost::split(split_hierachy, cur_line, boost::is_any_of("\t "), boost::token_compress_on);
				if (split_hierachy.size() != 2)
				{
					cout << mskel_file << " parsing MSKel error at line:" << line_num;
					return;
				}
				if (split_hierachy[0] == "marker")
				{
					break;
				}
				string bone_name1 = split_hierachy[0];
				string bone_name2 = split_hierachy[1];

				if (name_map.find(bone_name1) == name_map.end())
				{
					cout << mskel_file << " can not find name :" << bone_name1 << " at line:" << line_num;
				}
				if (name_map.find(bone_name2) == name_map.end())
				{
					cout << mskel_file << "can not find name :" << bone_name2 << " at line:" << line_num;
				}

				joints_list[name_map[bone_name1]].pid = joints_list[name_map[bone_name2]].id;
				joints_list[name_map[bone_name2]].cids.push_back(joints_list[name_map[bone_name1]].id);
			}
		}


	}

	//update LC WC
	for (int i = 0; i < joints_list.size(); i++)
	{

		EMat4f parent_WC = EMat4f::Identity();

		if (joints_list[i].pid != -1)
		{
			parent_WC = joints_list[joints_list[i].pid].WC;
		}
		//update LC
		EMat4f translation_LC = EMat4f::Identity();
		translation_LC(0, 3) = joints_list[i].translation[0];
		translation_LC(1, 3) = joints_list[i].translation[1];
		translation_LC(2, 3) = joints_list[i].translation[2];


		EMat4f jo_LC = EMat4f::Identity();
		EMat3f jo_rot = getRot3X3fromAngle(joints_list[i].jo);

		for (int yy = 0; yy < 3; yy++)
		{
			for (int xx = 0; xx < 3; xx++)
			{
				jo_LC(yy, xx) = jo_rot(yy, xx);
			}
		}

		joints_list[i].LC = translation_LC * jo_LC;
		joints_list[i].WC = parent_WC * joints_list[i].LC;
	}
	cout << "output result" << endl;
	vector<BoneHierData> bhdatas;
	ofstream os("sk_temp.asf");

	set<string> dof_joints;
	dof_joints.insert("spline1");
	dof_joints.insert("spline2");
	dof_joints.insert("spline3");
	dof_joints.insert("neck1");
	dof_joints.insert("neck2");
	dof_joints.insert("head");

	os << ":root" << endl;
	os << "  axis xyz" << endl;
	os << "  order tx ty tz rx ry rz" << endl;
	os << "  position 0.000000 0.000000 0.000000" << endl;
	os << "  orientation 0 0 0" << endl;
	os << ":bonedata" << endl;
	int bone_id = 1;
	for (int i = 0; i < joints_list.size(); i++)
	{
		//cout << joints_list[i].name << " " << joints_list[i].pid <<" " << joints_list[i].cids.size() << endl;
		if (joints_list[i].cids.size() == 0)
		{
			float length = 0;
			EVec3f dir = EVec3f(0, 1, 0);


			EMat3f rot;
			for (int yy = 0; yy < 3; yy++)
			{
				for (int xx = 0; xx < 3; xx++)
				{
					rot(yy, xx) = joints_list[i].WC(yy, xx);
				}
			}

			EVec3f eulerAngle = rot.eulerAngles(2, 1, 0) / M_PI * 180; //rz, ry, rx

			os << "  begin" << endl;
			os << "    id " << bone_id << endl;
			os << "    name " << joints_list[i].name << "-" << joints_list[i].name << endl;
			os << "    direction " << dir.transpose() << endl;
			os << "    length " << length << endl;
			os << "    axis " << eulerAngle[2] << " " << eulerAngle[1] << " " << eulerAngle[0] << " xyz" << endl;

			if (dof_joints.find(joints_list[i].name) != dof_joints.end())
			{
				os << "    dof rx ry rz" << endl;
			}

			os << "  end" << endl;

			BoneHierData bhdata;
			bhdata.pname = joints_list[i].name;
			bhdata.cname = joints_list[i].name;
			bhdata.name = joints_list[i].name + "-" + joints_list[i].name;
			bhdatas.push_back(bhdata);
			bone_id++;
		}
		else
		{
			for (int j = 0; j < joints_list[i].cids.size(); j++)
			{
				EVec3f dir;
				dir[0] = joints_list[joints_list[i].cids[j]].WC(0, 3) - joints_list[i].WC(0, 3);
				dir[1] = joints_list[joints_list[i].cids[j]].WC(1, 3) - joints_list[i].WC(1, 3);
				dir[2] = joints_list[joints_list[i].cids[j]].WC(2, 3) - joints_list[i].WC(2, 3);

				float length = dir.norm();

				if (length < 1E-6)
				{
					length = 0;
					dir = EVec3f(0, 1, 0);
				}
				dir.normalize();

				EMat3f rot;
				for (int yy = 0; yy < 3; yy++)
				{
					for (int xx = 0; xx < 3; xx++)
					{
						rot(yy, xx) = joints_list[i].WC(yy, xx);
					}
				}

				EVec3f eulerAngle = rot.eulerAngles(2, 1, 0) / M_PI * 180; //rz, ry, rx

				os << "  begin" << endl;
				os << "    id " << bone_id << endl;
				os << "    name " << joints_list[i].name << "-" << joints_list[joints_list[i].cids[j]].name << endl;
				os << "    direction " << dir.transpose() << endl;
				os << "    length " << length << endl;
				os << "    axis " << eulerAngle[2] << " " << eulerAngle[1] << " " << eulerAngle[0] << " xyz" << endl;
				if (dof_joints.find(joints_list[i].name) != dof_joints.end())
				{
					os << "    dof rx ry rz" << endl;
				}
				os << "  end" << endl;
				BoneHierData bhdata;
				bhdata.pname = joints_list[i].name;
				bhdata.cname = joints_list[joints_list[i].cids[j]].name;
				bhdata.name = joints_list[i].name + "-" + joints_list[joints_list[i].cids[j]].name;
				bhdatas.push_back(bhdata);
				bone_id++;
				cout << joints_list[i].name << "-" << joints_list[joints_list[i].cids[j]].name << " : " << length << " " << dir.transpose() << " " << eulerAngle.transpose() << endl;
			}
		}

	}
	os << ":hierarchy" << endl;
	os << "  begin" << endl;

	//process root
	os << "    root ";
	for (int i = 0; i < bhdatas.size(); i++)
	{
		if (bhdatas[i].pname == joints_list[0].name)
		{
			os << bhdatas[i].name << " ";
		}
	}
	os << endl;

	for (int i = 0; i < bhdatas.size(); i++)
	{
		string cname = bhdatas[i].cname;

		vector<string> childs;
		for (int j = 0; j < bhdatas.size(); j++)
		{
			if (bhdatas[j].pname == cname && bhdatas[j].name != bhdatas[i].name)
			{
				childs.push_back(bhdatas[j].name);
			}
		}


		os << "    " << bhdatas[i].name << " ";
		for (int j = 0; j < childs.size(); j++)
			os << childs[j] << " ";
		os << endl;

	}
	/*for (int i = 0; i < joints_list.size(); i++)
	{
		if (i == 0)
		{
			os << "    root ";
			for (int j = 0; j < joints_list[i].cids.size(); j++)
			{
				os << joints_list[i].name << "-" << joints_list[joints_list[i].cids[j]].name << " ";
			}
			os << endl;
		}
		else
		{
			os << "    " << joints_list[i].name<<" ";
			for (int j = 0; j < joints_list[i].cids.size(); j++)
			{
				os << joints_list[joints_list[i].cids[j]].name << " ";
			}
			os << endl;
		}
	}*/
	os << "  end" << endl;
	os.close();

	//return GSkeleton("sk_temp.asf").scaleSK(scale);

}

//struct MayaCamera
//{
//	EVec3f pos_cm;
//	EVec3f angle;
//	float focal_mm;
//	float film_width_inch;
//	float film_height_inch;
//	int width_pixel;
//	int height_pixel;
//};
//
//void readMayaCamera(string file, int width, int height, vector<MayaCamera>& cameras)
//{
//	ifstream is(file.c_str());
//
//	string str;
//	for (int i = 0; i < 2; i++)
//		getline(is, str);
//	//line3 
//	getline(is, str);
//	vector<string> tokens;
//	boost::split(tokens, str, boost::is_any_of(","));
//
//	int startFrame = stoi(tokens[2]);
//	int endFrame = stoi(tokens[3]);
//
//	float film_width_inch = stof(tokens[4]);
//	float film_height_inch = stof(tokens[5]);
//
//	float film_width = stof(tokens[4]) * 25.4; // inch to mm
//	float film_height = stof(tokens[5]) * 25.4;
//
//
//
//	for (int i = startFrame; i <= endFrame; i++)
//	{
//		MayaCamera mcam;
//		mcam.width_pixel = width;
//		mcam.height_pixel = height;
//		mcam.film_height_inch = film_height_inch;
//		mcam.film_width_inch = film_width_inch;
//
//
//		getline(is, str);
//		boost::split(tokens, str, boost::is_any_of(","));
//		EVec3f pos(stof(tokens[1]), stof(tokens[2]), stof(tokens[3]));
//
//		mcam.pos_cm = pos;
//
//
//
//		EVec3f angle(stof(tokens[4]), stof(tokens[5]), stof(tokens[6]));
//
//		mcam.angle = angle;
//
//		float focal_mm = stof(tokens[7]);
//		mcam.focal_mm = focal_mm;
//
//		cameras.push_back(mcam);
//
//	}
//
//	is.close();
//}
//
//GCameraPers maycam2gcam(const MayaCamera &mcam)
//{
//	float film_width = mcam.film_width_inch * 25.4; // inch to mm
//	float film_height = mcam.film_height_inch * 25.4;
//
//	float focal_pixel = mcam.focal_mm / film_width * mcam.width_pixel;
//
//	EVec3f pos = mcam.pos_cm * 0.01;
//	EMat3f rot = getRot3X3fromAngle(mcam.angle);
//
//	EVec3f target = pos + EVec3f(rot(0, 2), rot(1, 2), rot(2, 2)) * (-1);
//	EVec3f up = EVec3f(rot(0, 1), rot(1, 1), rot(2, 1));
//
//	GCameraPers camera_real(mcam.width_pixel, mcam.height_pixel, 0.1, 10.0, GCameraIntrinsic(focal_pixel, focal_pixel, mcam.width_pixel / 2, mcam.height_pixel / 2), GCameraInfo(GVector3(pos.x(), pos.y(), pos.z()), GVector3(target.x(), target.y(), target.z()), GVector3(up.x(), up.y(), up.z())));
//	return camera_real;
//}

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



		float focal_pixel = focal_mm / film_width * width;
		cout << i << " " << pos.transpose() << " " << angle.transpose() << " " << focal_mm << " " << focal_pixel << endl;


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


struct Joint2DData
{
	vector<cv::Point2i> joints;
	vector<cv::Scalar> joints_color;
	cv::Mat img;
	int select_idx;
};
void onMouseDrag(int event, int x, int y, int flags, void* param)
{
	Joint2DData *ud = (Joint2DData*)param;
	float minDis;
	cv::Mat img;

	///cout << "x:" << x << " y:" << y << endl;
	switch (event)
	{
	case cv::EVENT_LBUTTONDOWN:
		//cout << "mouse clicked" << endl;
		ud->select_idx = -1;
		minDis = 10000;
		for (int i = 0; i < ud->joints.size(); i++)
		{
			cv::Point2i diff(x - ud->joints[i].x, y - ud->joints[i].y);
			float dis = sqrt(diff.dot(diff));

			if (dis < minDis)
			{
				minDis = dis;
				ud->select_idx = i;
			}
		}

		break;
	case cv::EVENT_MOUSEMOVE:
		//cout << "mouse move" << endl;
		if (ud->select_idx != -1)
		{
			ud->joints[ud->select_idx].x = x;
			ud->joints[ud->select_idx].y = y;
		}


		img = ud->img.clone();
		for (int i = 0; i < ud->joints.size(); i++)
		{
			cv::circle(img, ud->joints[i], 3, ud->joints_color[i], -1);
		}

		cv::imshow("interface", img);
		break;
	case cv::EVENT_LBUTTONUP:
		ud->joints[ud->select_idx].x = x;
		ud->joints[ud->select_idx].y = y;
		ud->select_idx = -1;

		img = ud->img.clone();
		for (int i = 0; i < ud->joints.size(); i++)
		{
			cv::circle(img, ud->joints[i], 3, ud->joints_color[i], -1);
		}

		cv::imshow("interface", img);
		break;
	default:
		break;
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








void adjust_joint_2d_interface()
{
	OpenGLInitializer::initGLContext1();

	cv::namedWindow("interface");

	cv::Mat temp_img = cv::Mat::zeros(5, 5, CV_8UC3);
	cv::imshow("interface", temp_img);
	cv::waitKey(1);

	YAML::Node config = YAML::LoadFile("joint2d_mskel.yaml");



	/*string camera_file = R"(S:\users\jianjie\HeadChange\20220120\indoor\DSC_0790\19201080TRACK\DSC_0790_data\CamInfo_Camera01_2Node.txt)";
	string data_dir = R"(S:\users\jianjie\HeadChange\20220120\indoor\DSC_0790\19201080TRACK\)";
	string sk_file = R"(S:\users\jianjie\HeadChange\yuxuan_asf\yuxuan_ok_dof.asf)";
	string amc_file = R"(S:\users\jianjie\HeadChange\20220120\indoor\DSC_0790\19201080TRACK\final\poses.amc)";*/
	//string mesh_file = R"(S:\users\liubo\data\yuxuan.obj)";
	//string mesh_weights_file = R"(S:\users\liubo\data\yuxuan.mesh.weights.txt)";

	string camera_file = config["camera_file"].as<string>();
	string data_dir = config["data_dir"].as<string>();
	string sk_file = config["sk_file"].as<string>();
	string mskel_file = config["mskel_file"].as<string>();
	string sps_file = config["sps_file"].as<string>();
	string mesh_file = config["mesh_file"].as<string>();
	string mesh_weights_file = config["mesh_weights_file"].as<string>();
	int startIdx = config["start_frame"].as<int>();
	int endIdx = config["end_frame"].as<int>();


	map<string, string> joint_names;
	joint_names.insert(std::pair<string, string>("head", "head-head_end"));
	joint_names.insert(std::pair<string, string>("lshoulder", "lhumerus-lradius"));
	joint_names.insert(std::pair<string, string>("rshoulder", "rhumerus-rradius"));
	joint_names.insert(std::pair<string, string>("neck", "neck1-neck2"));
	joint_names.insert(std::pair<string, string>("upper", "spline2-spline3"));
	joint_names.insert(std::pair<string, string>("root", "root"));
	joint_names.insert(std::pair<string, string>("spline", "spline4-neck1"));

	vector<GCameraPers> tmp_cameras;
	vector<GCameraPers> tmp_real_cameras;
	vector<EMat4f> camera_trans;
	tmp_real_cameras = readCamera(camera_file, 1920, 1080, tmp_cameras, camera_trans);



	vector<MayaCamera> mcams;
	readMayaCamera(camera_file, 1920, 1080, mcams);

	GSkeleton sk(sk_file);
	sk.scaleSKSelf(0.001);


	ZSkeleton::Ptr sk_source = std::make_shared<ZSkeleton>("liubo");

	sk_source->loadSK(mskel_file);

	sk_source->loadMotion(sps_file);

	auto frame_range = sk_source->getRoot()->getFrameRange();



	GPoseF pose;
	pose.resize(sk.NumFreedoms(), 0);
	sk.UpdatePose(pose, false);

	//string mesh_file = R"(S:\users\liubo\data\yuxuan.obj)";
	GMeshf::Ptr mesh_ptr;
	GMeshOBJReader(mesh_file, 0.001)(mesh_ptr);
	mesh_ptr->computeNormals();

	//string pw_file = R"(S:\users\liubo\data\yuxuan.mesh.weights.txt)";
	auto lbs_data = boost::make_shared<LBSData>();
	lbs_data->loadWeights(mesh_weights_file, sk, *mesh_ptr);
	cout << "load done" << endl;
	LinearBlendSkining lbs(sk, mesh_ptr->vertices().begin(), lbs_data);

	//cout << "dof:" << sk.NumFreedoms() << endl;
	auto param = GSkeletonDrawerStickParams::Red();



	GCameraPers camera = tmp_cameras[0];

	cout << "focal: " << camera.getCameraIntrinsic().fx << endl;

	FBOColorRender fbo(camera.npixel_v, camera.npixel_u);
	for (int i = 0; i < tmp_real_cameras.size(); i++)
		fbo.addView(util::MakeClone(tmp_real_cameras[i]));




	vector<float4> data;
	data.resize(camera.npixel_u * camera.npixel_v);
	fbo.setLighting();

	GSkeletonDrawerStick skDrawer(param);
	GMeshDrawer meshDrawer;

	//SplineNeckSynthesis_HC sns(sk, camera, joint_names, 0.1);



	boost::format fmt("%04d");


	Joint2DData j2d;
	j2d.joints.push_back(cv::Point2i(0, 0));
	j2d.joints.push_back(cv::Point2i(0, 0));
	j2d.joints.push_back(cv::Point2i(0, 0));
	j2d.joints.push_back(cv::Point2i(0, 0));

	j2d.joints_color.push_back(cv::Scalar(0, 0, 255));
	j2d.joints_color.push_back(cv::Scalar(0, 255, 0));
	j2d.joints_color.push_back(cv::Scalar(255, 0, 0));
	j2d.joints_color.push_back(cv::Scalar(255, 255, 0));
	j2d.img = cv::Mat::zeros(1200, 960, CV_8UC3);

	cv::setMouseCallback("interface", onMouseDrag, &j2d);

	vector<string> joint_names_selected;
	joint_names_selected.push_back("lhumerus");
	joint_names_selected.push_back("rhumerus");
	joint_names_selected.push_back("spline4");
	joint_names_selected.push_back("head");


	map<string, string> mskel2asf_bonemap; //for visualization
	mskel2asf_bonemap.insert(std::pair<string, string>("root", "root"));
	mskel2asf_bonemap.insert(std::pair<string, string>("spline1", "spline1-spline2"));
	mskel2asf_bonemap.insert(std::pair<string, string>("spline2", "spline2-spline3"));
	mskel2asf_bonemap.insert(std::pair<string, string>("spline3", "spline3-spline4"));
	mskel2asf_bonemap.insert(std::pair<string, string>("neck1", "neck1-neck2"));
	mskel2asf_bonemap.insert(std::pair<string, string>("neck2", "neck2-head"));
	mskel2asf_bonemap.insert(std::pair<string, string>("head", "head-head_end"));


	set<string> mskel_bones; //bones that have dofs;
	mskel_bones.insert("root");
	mskel_bones.insert("spline1");
	mskel_bones.insert("spline2");
	mskel_bones.insert("spline3");
	mskel_bones.insert("neck1");
	mskel_bones.insert("neck2");
	mskel_bones.insert("head");


	vector<map<string, EVecXf>> mskel_poses;

	for (int i = frame_range.first; i <= frame_range.second; i++)
	{
		map<string, EVecXf> pose;
		sk_source->setKey(i);

		for (auto iter = mskel_bones.begin(); iter != mskel_bones.end(); iter++)
		{
			string bone_name = (*iter);

			ZJoint::Ptr jnt = sk_source->getJointByName(bone_name);

			if (!jnt->getParent())
			{
				EVecXf pp = EVecXf::Zero(6);
				pp[0] = jnt->getTranslation()[0];
				pp[1] = jnt->getTranslation()[1];
				pp[2] = jnt->getTranslation()[2];
				pp[3] = jnt->getRotate()[0];
				pp[4] = jnt->getRotate()[1];
				pp[5] = jnt->getRotate()[2];

				pose.insert(std::pair<string, EVecXf>(bone_name, pp));
			}

			else
			{
				EVecXf pp = EVecXf::Zero(3);

				pp[0] = jnt->getRotate()[0];
				pp[1] = jnt->getRotate()[1];
				pp[2] = jnt->getRotate()[2];

				pose.insert(std::pair<string, EVecXf>(bone_name, pp));
			}
		}

		mskel_poses.push_back(pose);

	}



	bool drawMesh = false;
	int flag = 0;
	//render
	if (true)
	{
		for (int i = 0; i < 1000; i++)
		{

			while (1)
			{
				//sk_source->setKey(i + frame_range.first);
				setMSkelPoseByMap(sk_source, mskel_poses[i]);
				GPoseF cur_pose;
				sps2amc(sk_source, sk, mskel2asf_bonemap, 0.001, cur_pose);
				sk.UpdatePose(cur_pose, false);
				GMeshf::Ptr mesh_ptr2 = lbs.deformMesh(sk, *mesh_ptr);

				if (drawMesh == false)
				{
					fbo.renderBatch(i, 1, [&](int idx) {
						//meshDrawer.draw(*mesh_ptr2);
						skDrawer.draw(sk);
					});
				}
				else
				{
					fbo.renderBatch(i, 1, [&](int idx) {
						meshDrawer.draw(*mesh_ptr2);
						skDrawer.draw(sk);
					});
				}


				fbo.getRenderedCPU(0, 0, true, &data);

				cv::Mat retImg(camera.npixel_v, camera.npixel_u, CV_32FC4, data.data());

				cv::Mat retImg2 = cv::Mat::zeros(camera.npixel_v, camera.npixel_u, CV_8UC3);

				for (int yy = 0; yy < retImg.rows; yy++)
				{
					for (int xx = 0; xx < retImg.cols; xx++)
					{
						retImg2.at<cv::Vec3b>(yy, xx)[0] = retImg.at<cv::Vec4f>(yy, xx)[2] * 255;
						retImg2.at<cv::Vec3b>(yy, xx)[1] = retImg.at<cv::Vec4f>(yy, xx)[1] * 255;
						retImg2.at<cv::Vec3b>(yy, xx)[2] = retImg.at<cv::Vec4f>(yy, xx)[0] * 255;
					}
				}

				cout << "Image Idx:" << i << endl;
				fmt%i;
				cv::Mat img = cv::imread(data_dir + fmt.str() + ".jpg");
				//cv::flip(img, img, 1);

				float alpha = 0.3;

				for (int yy = 0; yy < retImg2.rows; yy++)
				{
					for (int xx = 0; xx < retImg2.cols; xx++)
					{
						if (retImg2.at<cv::Vec3b>(yy, xx)[0] > 0 || retImg2.at<cv::Vec3b>(yy, xx)[1] > 0 || retImg2.at<cv::Vec3b>(yy, xx)[2] > 0)
						{
							img.at<cv::Vec3b>(yy, xx)[0] = alpha * img.at<cv::Vec3b>(yy, xx)[0] + (1 - alpha) * retImg2.at<cv::Vec3b>(yy, xx)[0];
							img.at<cv::Vec3b>(yy, xx)[1] = alpha * img.at<cv::Vec3b>(yy, xx)[1] + (1 - alpha) * retImg2.at<cv::Vec3b>(yy, xx)[1];
							img.at<cv::Vec3b>(yy, xx)[2] = alpha * img.at<cv::Vec3b>(yy, xx)[2] + (1 - alpha) * retImg2.at<cv::Vec3b>(yy, xx)[2];
						}

					}
				}
				EVec3f lshoulder_3d = sk.PointW(sk.findBone(joint_names["lshoulder"])->boneIndex, EVec3f(0, 0, 0));
				EVec3f rshoulder_3d = sk.PointW(sk.findBone(joint_names["rshoulder"])->boneIndex, EVec3f(0, 0, 0));
				EVec3f spline_3d = sk.PointW(sk.findBone(joint_names["spline"])->boneIndex, EVec3f(0, 0, 0));
				EVec3f head_3d = sk.PointW(sk.findBone(joint_names["head"])->boneIndex, EVec3f(0, 0, 0));

				GVector2 lshoulder_2d = tmp_real_cameras[i].Project(GVector3(lshoulder_3d.x(), lshoulder_3d.y(), lshoulder_3d.z()));
				GVector2 rshoulder_2d = tmp_real_cameras[i].Project(GVector3(rshoulder_3d.x(), rshoulder_3d.y(), rshoulder_3d.z()));
				GVector2 spline_2d = tmp_real_cameras[i].Project(GVector3(spline_3d.x(), spline_3d.y(), spline_3d.z()));
				GVector2 head_2d = tmp_real_cameras[i].Project(GVector3(head_3d.x(), head_3d.y(), head_3d.z()));

				/*cv::circle(img, cv::Point2i(lshoulder_2d.x, lshoulder_2d.y), 5, cv::Scalar(0, 0, 255), -1);
				cv::circle(img, cv::Point2i(rshoulder_2d.x, rshoulder_2d.y), 5, cv::Scalar(0, 255, 0), -1);
				cv::circle(img, cv::Point2i(spline_2d.x, spline_2d.y), 5, cv::Scalar(255, 0, 0), -1);*/

				j2d.joints[0] = cv::Point2i(lshoulder_2d.x, lshoulder_2d.y);
				j2d.joints[1] = cv::Point2i(rshoulder_2d.x, rshoulder_2d.y);
				j2d.joints[2] = cv::Point2i(spline_2d.x, spline_2d.y);
				j2d.joints[3] = cv::Point2i(head_2d.x, head_2d.y);
				j2d.img = img.clone();
				j2d.select_idx = -1;

				cv::Mat show_img = j2d.img.clone();
				for (int j = 0; j < j2d.joints.size(); j++)
				{
					cv::circle(show_img, j2d.joints[j], 3, j2d.joints_color[j], -1);
				}
				cv::imshow("interface", show_img);
				int key = cv::waitKey(0);
				cout << key << " is pressed" << endl;

				if (key == 97)
				{
					cout << "optimize" << endl;


					/*joints.push_back(j2d.joints[0]);
					joints.push_back(j2d.joints[1]);
					joints.push_back(j2d.joints[2]);*/
					if (flag == 1)
					{
						cout << "Current Mode: refine root pose only" << endl;
					}
					else if (flag == 2)
					{
						cout << "Current Mode: refine root only" << endl;
					}
					else
					{
						cout << "Current Mode: refine all" << endl;
					}
					cout << "before optmize:" << sk_source->getRoot()->getTranslation().transpose() << endl;

					//convert maya cam to GCameraPers
					GCameraPers cur_cam = maycam2gcam(mcams[i]);
					//adjust_by_joints2d_mskel(sk_source, i, mskel2asf_bonemap, tmp_real_cameras[i], joints, joint_names_selected,  flag, 10);

					//input joints 2d coordinate
					vector<cv::Point2i> joints;
					joints.push_back(j2d.joints[0]);
					joints.push_back(j2d.joints[1]);

					// input joints 2d's corresponding names
					vector<string> cur_joint_names_selected;

					cur_joint_names_selected.push_back("lhumerus");
					cur_joint_names_selected.push_back("rhumerus");
					//cur_joint_names_selected.push_back("spline4");
					//cur_joint_names_selected.push_back("head");


					set<string> cur_mskel_bones; //bones that need to be optimized;
					cur_mskel_bones.insert("root");
					cur_mskel_bones.insert("spline1");
					cur_mskel_bones.insert("spline2");
					cur_mskel_bones.insert("spline3");
					/*cur_mskel_bones.insert("neck1");
					cur_mskel_bones.insert("neck2");
					cur_mskel_bones.insert("head");*/

					//mskel_poses[i] is the current pose( should from maya)
					adjust_by_joints2d_mskel_levmar_v2(sk_source, mskel_poses[i], cur_mskel_bones, cur_cam, joints, cur_joint_names_selected, 0.001, optimizeParam());


					cout << "after optmize:" << sk_source->getRoot()->getTranslation().transpose() << endl;
					cout << "optimize done" << endl;
				}

				if (key == 98)
				{
					cout << "mesh trigger" << endl;
					if (drawMesh == false)
						drawMesh = true;
					else
						drawMesh = false;
				}

				if (key == 99) //c
				{
					flag = (flag + 1) % 3;

					if (flag == 1)
					{
						cout << "enter Mode:refine root pose only" << endl;
					}
					else if (flag == 2)
					{
						cout << "enter Mode refine root only" << endl;
					}
					else
					{
						cout << "enter Mode refine all" << endl;
					}
				}

				if (key == 103) //g
				{
					cout << " please input the frame number:" << endl;
					int frameNo;
					cin >> frameNo;

					if (frameNo < 0)
						frameNo = 0;
					if (frameNo > frame_range.second - frame_range.first)
						frameNo = frame_range.second - frame_range.first;

					i = frameNo - 1;
					break;
				}

				if (key == 105)//i
				{
					cout << "interpolation:" << endl;
					cout << "please input the start frame:" << endl;
					int startNo;
					cin >> startNo;
					cout << "please input the end frame:" << endl;
					int endNo;
					cin >> endNo;

					if (startNo < 0)
						startNo = 0;
					if (endNo > frame_range.second - frame_range.first)
						endNo = frame_range.second - frame_range.first;

					if (startNo < endNo)
					{
						cout << " root pos linear interpolation" << endl;

						/*for (int i = startNo + 1; i <= endNo - 1; i++)
						{
							mot[i][0] = mot[startNo][0] + (mot[endNo][0] - mot[startNo][0]) * (i - startNo) * 1.0 / (endNo - startNo);
							mot[i][1] = mot[startNo][1] + (mot[endNo][1] - mot[startNo][1]) * (i - startNo) * 1.0 / (endNo - startNo);
							mot[i][2] = mot[startNo][2] + (mot[endNo][2] - mot[startNo][2]) * (i - startNo) * 1.0 / (endNo - startNo);
						}*/
					}
					else
						cout << "startNo is larger then endNo, no interpolation is performed" << endl;
				}

				if (key == 115) //s
				{
					cout << "saving motion file" << endl;
					//mot.saveMotion(sk, amc_file);
					cout << "save done" << endl;
				}

				if (key == 27)
				{
					cout << "esc pressed" << endl;
					break;
				}

				if (i == frame_range.second - frame_range.first)
					i--;
			}





		}
	}

}

void test_unit()
{
	
	YAML::Node config = YAML::LoadFile("C:/Users/admin/haojw/GitLab/maya/PLE-22602/InteractivePoseEditing/Interface/joint2d_mskel.yaml");
	cout << "load yaml" << "\n";
	string camera_file = config["camera_file"].as<string>();
	string data_dir = config["data_dir"].as<string>();
	string sk_file = config["sk_file"].as<string>();
	string mskel_file = config["mskel_file"].as<string>();
	string sps_file = config["sps_file"].as<string>();
	string mesh_file = config["mesh_file"].as<string>();
	string mesh_weights_file = config["mesh_weights_file"].as<string>();
	int startIdx = config["start_frame"].as<int>();
	int endIdx = config["end_frame"].as<int>();


	map<string, string> joint_names;
	joint_names.insert(std::pair<string, string>("head", "head-head_end"));
	joint_names.insert(std::pair<string, string>("lshoulder", "lhumerus-lradius"));
	joint_names.insert(std::pair<string, string>("rshoulder", "rhumerus-rradius"));
	joint_names.insert(std::pair<string, string>("neck", "neck1-neck2"));
	joint_names.insert(std::pair<string, string>("upper", "spline2-spline3"));
	joint_names.insert(std::pair<string, string>("root", "root"));
	joint_names.insert(std::pair<string, string>("spline", "spline4-neck1"));



	vector<GCameraPers> tmp_cameras;
	vector<GCameraPers> tmp_real_cameras;
	vector<EMat4f> camera_trans;
	tmp_real_cameras = readCamera(camera_file, 1920, 1080, tmp_cameras, camera_trans);



	vector<MayaCamera> mcams;
	readMayaCamera(camera_file, 1920, 1080, mcams);

	GSkeleton sk(sk_file);
	sk.scaleSKSelf(0.001);



	ZSkeleton::Ptr sk_source = std::make_shared<ZSkeleton>("liubo");

	sk_source->loadSK(mskel_file);

	sk_source->loadMotion(sps_file);

	auto frame_range = sk_source->getRoot()->getFrameRange();
	cout << "frame_range:" << frame_range.first << "--" << frame_range.second << "\n";

	map<string, string> mskel2asf_bonemap; //for visualization
	mskel2asf_bonemap.insert(std::pair<string, string>("root", "root"));
	mskel2asf_bonemap.insert(std::pair<string, string>("spline1", "spline1-spline2"));
	mskel2asf_bonemap.insert(std::pair<string, string>("spline2", "spline2-spline3"));
	mskel2asf_bonemap.insert(std::pair<string, string>("spline3", "spline3-spline4"));
	mskel2asf_bonemap.insert(std::pair<string, string>("neck1", "neck1-neck2"));
	mskel2asf_bonemap.insert(std::pair<string, string>("neck2", "neck2-head"));
	mskel2asf_bonemap.insert(std::pair<string, string>("head", "head-head_end"));


	set<string> mskel_bones; //bones that have dofs;
	mskel_bones.insert("root");
	mskel_bones.insert("spline1");
	mskel_bones.insert("spline2");
	mskel_bones.insert("spline3");
	mskel_bones.insert("neck1");
	mskel_bones.insert("neck2");
	mskel_bones.insert("head");


	vector<map<string, EVecXf>> mskel_poses;

	for (int i = frame_range.first; i <= frame_range.second; i++)
	{
		map<string, EVecXf> pose;
		sk_source->setKey(i);

		for (auto iter = mskel_bones.begin(); iter != mskel_bones.end(); iter++)
		{
			string bone_name = (*iter);

			ZJoint::Ptr jnt = sk_source->getJointByName(bone_name);

			if (!jnt->getParent())
			{
				EVecXf pp = EVecXf::Zero(6);
				pp[0] = jnt->getTranslation()[0];
				pp[1] = jnt->getTranslation()[1];
				pp[2] = jnt->getTranslation()[2];
				pp[3] = jnt->getRotate()[0];
				pp[4] = jnt->getRotate()[1];
				pp[5] = jnt->getRotate()[2];
				
				pose.insert(std::pair<string, EVecXf>(bone_name, pp));
			}

			else
			{
				EVecXf pp = EVecXf::Zero(3);

				pp[0] = jnt->getRotate()[0];
				pp[1] = jnt->getRotate()[1];
				pp[2] = jnt->getRotate()[2];
				
				pose.insert(std::pair<string, EVecXf>(bone_name, pp));
			}
		}

		mskel_poses.push_back(pose);

	}

	int pose_idx = frame_range.first + 1;
	//sk_source->setKey(i + frame_range.first);
	setMSkelPoseByMap(sk_source, mskel_poses[pose_idx]);
	GPoseF cur_pose;
	sps2amc(sk_source, sk, mskel2asf_bonemap, 0.001, cur_pose);
	sk.UpdatePose(cur_pose, false);



	EVec3f lshoulder_3d = sk.PointW(sk.findBone(joint_names["lshoulder"])->boneIndex, EVec3f(0, 0, 0));
	EVec3f rshoulder_3d = sk.PointW(sk.findBone(joint_names["rshoulder"])->boneIndex, EVec3f(0, 0, 0));
	EVec3f spline_3d = sk.PointW(sk.findBone(joint_names["spline"])->boneIndex, EVec3f(0, 0, 0));
	EVec3f head_3d = sk.PointW(sk.findBone(joint_names["head"])->boneIndex, EVec3f(0, 0, 0));

	GVector2 lshoulder_2d = tmp_real_cameras[pose_idx].Project(GVector3(lshoulder_3d.x(), lshoulder_3d.y(), lshoulder_3d.z()));
	GVector2 rshoulder_2d = tmp_real_cameras[pose_idx].Project(GVector3(rshoulder_3d.x(), rshoulder_3d.y(), rshoulder_3d.z()));
	GVector2 spline_2d = tmp_real_cameras[pose_idx].Project(GVector3(spline_3d.x(), spline_3d.y(), spline_3d.z()));
	GVector2 head_2d = tmp_real_cameras[pose_idx].Project(GVector3(head_3d.x(), head_3d.y(), head_3d.z()));

	cout << "before optmize:" << sk_source->getRoot()->getTranslation().transpose() << endl;

	//convert maya cam to GCameraPers
	GCameraPers cur_cam = maycam2gcam(mcams[pose_idx]);
	//adjust_by_joints2d_mskel(sk_source, i, mskel2asf_bonemap, tmp_real_cameras[i], joints, joint_names_selected,  flag, 10);

	//input joints 2d coordinate
	vector<cv::Point2i> joints;
	joints.push_back(cv::Point2i(lshoulder_2d.x + 2, lshoulder_2d.y + 2));
	joints.push_back(cv::Point2i(rshoulder_2d.x + 2, rshoulder_2d.y + 2));

	// input joints 2d's corresponding names
	vector<string> cur_joint_names_selected;

	cur_joint_names_selected.push_back("lhumerus");
	cur_joint_names_selected.push_back("rhumerus");
	//cur_joint_names_selected.push_back("spline4");
	//cur_joint_names_selected.push_back("head");


	set<string> cur_mskel_bones; //bones that need to be optimized;
	cur_mskel_bones.insert("root");
	cur_mskel_bones.insert("spline1");
	cur_mskel_bones.insert("spline2");
	cur_mskel_bones.insert("spline3");
	/*cur_mskel_bones.insert("neck1");
	cur_mskel_bones.insert("neck2");
	cur_mskel_bones.insert("head");*/

	//mskel_poses[i] is the current pose( should from maya)
	adjust_by_joints2d_mskel_levmar_v2(sk_source, mskel_poses[pose_idx], cur_mskel_bones, cur_cam, joints, cur_joint_names_selected, 0.001, optimizeParam());

	cout << "after optmize:" << sk_source->getRoot()->getTranslation().transpose() << endl;

	cout << "root" << mskel_poses[pose_idx]["root"] << "\n";
	cout << "spline1" << mskel_poses[pose_idx]["spline1"] << "\n";
	cout << "spline2" << mskel_poses[pose_idx]["spline2"] << "\n";
	cout << "spline3" << mskel_poses[pose_idx]["spline3"] << "\n";
	cout << "neck1" << mskel_poses[pose_idx]["neck1"] << "\n";
	cout << "neck2" << mskel_poses[pose_idx]["neck2"] << "\n";
	cout << "head" << mskel_poses[pose_idx]["head"] << "\n";

}


void main(int argc, char** argv)
{
	std::cout << "Hello World!\n";

	//adjust_joint_2d_interface();
	test_unit();

}

// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单

// 入门使用技巧: 
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
//   3. 使用输出窗口查看生成输出和其他消息
//   4. 使用错误列表窗口查看错误
//   5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
//   6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件
