#pragma once

#define C_INTERFACE_API __declspec(dllexport)

extern "C" {
	C_INTERFACE_API typedef struct c_pose{
		float p0;
		float p1;
		float p2;
		float p3;
		float p4;
		float p5;
	};

	C_INTERFACE_API typedef struct return_pose {
		c_pose head;
		c_pose neck1;
		c_pose neck2;
		c_pose root;
		c_pose spline1;
		c_pose spline2;
		c_pose spline3;
	};


	C_INTERFACE_API typedef struct maya_param{
		char* joints;
		// ======
		char* selected_joints_name;
		// ========
		char* cur_mskel_bones_name;
		// ==========
		char* angle;
		char* pos_cm;
		float film_width_inch;
		float film_height_inch;
		int width_pixel;
		int height_pixel;
		float focal_mm;
	};

	C_INTERFACE_API typedef struct return_pose_haojw {
		char * result;
	};

	C_INTERFACE_API void init_video_maker(char* yaml_path);
	C_INTERFACE_API void optimize_frame(c_pose* head, c_pose* neck1, c_pose* neck2, c_pose* root, c_pose* spline1, c_pose* spline2, c_pose* spline3,
		int mm_cm_m, maya_param* mparam, return_pose* repose);
	C_INTERFACE_API void test_py_arguments(c_pose* pose, char* p);
	

	// =================haojw test=================
	C_INTERFACE_API void test_init_video_maker_haojw(char* yaml_path);
	// C_INTERFACE_API void test_opt_haojw(char* joint_name, c_pose* joint, int mm_cm_m, maya_param* mparam, return_pose_haojw* repose);
	C_INTERFACE_API void test_projection_haojw(int mm_cm_m, maya_param* mparam, return_pose_haojw* result);
	C_INTERFACE_API void test_opt_haojw(char *json_str, int mm_cm_m, maya_param* mparam, return_pose_haojw* repose);
}

