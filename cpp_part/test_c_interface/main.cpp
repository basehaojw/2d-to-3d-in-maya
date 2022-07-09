#include <string>
#include <iostream>
#include "../c_interface/c_interface.h"

using namespace std;

int main() {
	char yaml_path[] = "E:\\InteractivePoseEditing\\x64\\Release\\joint2d_mskel.yaml";
	init_video_maker(&yaml_path[0]);
	std::cout << "init done\n";
	// ====================
	c_pose head{ 0.0350713, -0.036958, 0.0577193, 0, 0, 0 };
	c_pose neck1{0.0482044, -0.0763037, 0.0150297, 0, 0, 0};
	c_pose neck2{ 0.0486032, -0.0563124, 0.0310278, 0, 0, 0 };
	c_pose root{ 4713.64, 1479.32, -5983.26, -2.35504, -0.00843069, 0.0394872 };
	c_pose spline1{ -0.248062, -0.0715346, 0.109785, 0, 0, 0 };
	c_pose spline2{ -0.258886, -0.0721044, 0.085626, 0, 0, 0 };
	c_pose spline3{ -0.269955, -0.0755426, 0.0464903, 0, 0, 0 };
	int mm_cm_m = 0;  // 0表示mm, scale为0.001, 1表示cm, scale为0.01, 2表示m, scale为1.0

	maya_param mparam;
	char joints[] = "897,440;661,479";
	mparam.joints = &joints[0];
	char selected_joints_name[] = "lhumerus,rhumerus" ;
	mparam.selected_joints_name = &selected_joints_name[0];
	char cur_mskel_bones_name[] = "root,spline1,spline2,spline3";
	mparam.cur_mskel_bones_name = &cur_mskel_bones_name[0];

	char angle[] = "177.859,37.5894,179.642";
	char pos_cm[] = "629.537,192.878,-835.797";
	mparam.angle = &angle[0];
	mparam.pos_cm = &pos_cm[0];
	mparam.film_width_inch = 0.925197;
	mparam.film_height_inch = 0.520423;
	mparam.width_pixel = 1920;
	mparam.height_pixel = 1080;
	mparam.focal_mm = 35;

	// =======================
	return_pose ret;
	optimize_frame(&head, &neck1, &neck2, &root, &spline1, &spline2, &spline3, mm_cm_m, &mparam, &ret);
	std::cout << "after adjust\n";
	std::cout << "head "<< ret.head.p0 << " " << ret.head.p1 << " " << ret.head.p2 << "\n";
	std::cout << "neck1 " << ret.neck1.p0 << " " << ret.neck1.p1 << " " << ret.neck1.p2 << "\n";
	std::cout << "neck2 " << ret.neck2.p0 << " " << ret.neck2.p1 << " " << ret.neck2.p2 << "\n";
	std::cout << "root " << ret.root.p0 << " " << ret.root.p1 << " " << ret.root.p2 << " "<< ret.root.p3<<" "<< ret.root.p4<<" "<< ret.root.p5<<"\n";
	std::cout << "spline1 " << ret.spline1.p0 << " " << ret.spline1.p1 << " " << ret.spline1.p2 << "\n";
	std::cout << "spline2 " << ret.spline2.p0 << " " << ret.spline2.p1 << " " << ret.spline2.p2 << "\n";
	std::cout << "spline3 " << ret.spline3.p0 << " " << ret.spline3.p1 << " " << ret.spline3.p2 << "\n";
	return 0;
}
