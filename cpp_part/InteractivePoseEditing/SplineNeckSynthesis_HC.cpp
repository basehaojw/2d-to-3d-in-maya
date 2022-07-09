#include "SplineNeckSynthesis_HC.h"




//void disval_posesolver(float* p, float* hx, int m, int n, void* data)
//{
//	auto ext_data = reinterpret_cast<levmar_UserData*>(data);
//	ZSkeleton::Ptr sk_source = ext_data->sk_source;
//	std::map<ZName, unsigned int> index_map = ext_data->index_map;
//	vector<string> joints_names = ext_data->joints_names;
//	vector < cv::Point2i> joints_2d = ext_data->joints_2d;
//	GCameraPers cur_view = ext_data->view;
//	float joint_rot_smooth_weight = ext_data->joint_rot_smooth_weight;
//	float root_rot_smooth_weight = ext_data->root_rot_smooth_weight;
//	float root_tran_smooth_weight = ext_data->root_tran_smooth_weight;
//	float joint_2d_weight = ext_data->joint_2d_weight;
//	float scale = ext_data->scale;
//	vector<float> weights = ext_data->weights;
//	int dof = m;
//
//	for (unsigned int ii = 0; ii < sk_source->getJointNum(); ii++)
//	{
//		ZJoint::Ptr jnt = sk_source->getJointByJID(ii);
//		//if (bone_map.find(jnt->getName().get_str()) == bone_map.end())
//		if (index_map.find(jnt->getName().get_str()) == index_map.end())
//		{
//			continue;
//		}
//
//		if (jnt->getDegrees() == 0)
//		{
//			continue;
//		}
//
//		unsigned int bone_index = index_map[jnt->getName()];
//		unsigned int degree_index = 0;
//
//		ZMath::ZVector3D rotate_vec = jnt->getRotate();
//		ZMath::ZVector3D translate_vec = jnt->getTranslation();
//
//
//
//		for (unsigned int jj = 0; jj < 3; jj++)
//		{
//			if (!jnt->get_locked(jj))
//			{
//				translate_vec(jj) = p[bone_index + degree_index];
//				degree_index++;
//			}
//		}
//
//		for (unsigned int jj = 3; jj < 6; jj++)
//		{
//			if (!jnt->get_locked(jj))
//			{
//
//				rotate_vec(jj - 3) = p[bone_index + degree_index];
//
//				degree_index++;
//			}
//		}
//		jnt->translate(translate_vec);
//		jnt->rotate(rotate_vec);
//	}
//
//
//
//	for (int j = 0; j < joints_2d.size(); j++)
//	{
//
//
//		string joint_name;
//		
//
//		joint_name = joints_names[j];
//
//		EVec3f cur_pos = sk_source->getJointByName(joint_name)->worldMatrix().translation().cast<float>() * scale;
//
//		GVector2 cur_pos_2d = cur_view.Project(GVector3(cur_pos.x(), cur_pos.y(), cur_pos.z()));
//
//		
//
//		/*hx[2 * j + 0] = joint_2d_weight * (0 - (cur_pos_2d.x - joints_2d[j].x));
//		hx[2 * j + 1] = joint_2d_weight * (0 - (cur_pos_2d.y - joints_2d[j].y));*/
//
//		hx[2 * j + 0] = joint_2d_weight * cur_pos_2d.x ;
//		hx[2 * j + 1] = joint_2d_weight * cur_pos_2d.y ;
//
//	}
//	
//	//for (int j = 0; j < dof; j++)
//	//{
//	//	float target_value = p[j];
//	//	if (j < 3)
//	//		hx[2 * joints_2d.size() + j] = 5 * root_tran_smooth_weight * p[j]  * scale; // mm to m
//	//	else if (j < 6)
//	//		hx[2 * joints_2d.size() + j] = 10 * root_rot_smooth_weight * p[j]  * 180 / M_PI; //rad to deg
//	//	else
//	//		hx[2 * joints_2d.size() + j] = 5 * joint_rot_smooth_weight * p[j]  * 180 / M_PI; //rad to deg
//	//}
//
//
//	for (int j = 0; j < dof; j++)
//	{
//		float target_value = p[j];
//		if (j < 3)
//			hx[2 * joints_2d.size() + j] = 5 * weights[j] * p[j] * scale; // mm to m
//		else if (j < 6)
//			hx[2 * joints_2d.size() + j] = 10 * weights[j] * p[j] * 180 / M_PI; //rad to deg
//		else
//			hx[2 * joints_2d.size() + j] = 5 * weights[j] * p[j] * 180 / M_PI; //rad to deg
//	}
//}

void disval_posesolver_v2(float* p, float* hx, int m, int n, void* data)
{
	auto ext_data = reinterpret_cast<levmar_UserData*>(data);
	ZSkeleton::Ptr sk_source = ext_data->sk_source;
	std::map<ZName, unsigned int> index_map = ext_data->index_map;
	vector<string> joints_names = ext_data->joints_names;
	vector < cv::Point2i> joints_2d = ext_data->joints_2d;
	GCameraPers cur_view = ext_data->view;
	float joint_rot_smooth_weight = ext_data->joint_rot_smooth_weight;
	float root_rot_smooth_weight = ext_data->root_rot_smooth_weight;
	float root_tran_smooth_weight = ext_data->root_tran_smooth_weight;
	float joint_2d_weight = ext_data->joint_2d_weight;
	float scale = ext_data->scale;
	vector<float> weights = ext_data->weights;
	set<string> bone_set = ext_data->bone_set;

	int dof = m;

	for (unsigned int ii = 0; ii < sk_source->getJointNum(); ii++)
	{
		ZJoint::Ptr jnt = sk_source->getJointByJID(ii);
		//if (bone_map.find(jnt->getName().get_str()) == bone_map.end())
		if (index_map.find(jnt->getName().get_str()) == index_map.end())
		{
			continue;
		}

		if (jnt->getDegrees() == 0)
		{
			continue;
		}

		unsigned int bone_index = index_map[jnt->getName()];
		unsigned int degree_index = 0;

		ZMath::ZVector3D rotate_vec = jnt->getRotate();
		ZMath::ZVector3D translate_vec = jnt->getTranslation();



		for (unsigned int jj = 0; jj < 3; jj++)
		{
			if (!jnt->get_locked(jj))
			{
				translate_vec(jj) = p[bone_index + degree_index];
				degree_index++;
			}
		}

		for (unsigned int jj = 3; jj < 6; jj++)
		{
			if (!jnt->get_locked(jj))
			{

				rotate_vec(jj - 3) = p[bone_index + degree_index];

				degree_index++;
			}
		}
		jnt->translate(translate_vec);
		jnt->rotate(rotate_vec);
	}



	for (int j = 0; j < joints_2d.size(); j++)
	{


		string joint_name;


		joint_name = joints_names[j];

		EVec3f cur_pos = sk_source->getJointByName(joint_name)->worldMatrix().translation().cast<float>() * scale;

		GVector2 cur_pos_2d = cur_view.Project(GVector3(cur_pos.x(), cur_pos.y(), cur_pos.z()));



		/*hx[2 * j + 0] = joint_2d_weight * (0 - (cur_pos_2d.x - joints_2d[j].x));
		hx[2 * j + 1] = joint_2d_weight * (0 - (cur_pos_2d.y - joints_2d[j].y));*/

		hx[2 * j + 0] = joint_2d_weight * cur_pos_2d.x;
		hx[2 * j + 1] = joint_2d_weight * cur_pos_2d.y;

	}

	//for (int j = 0; j < dof; j++)
	//{
	//	float target_value = p[j];
	//	if (j < 3)
	//		hx[2 * joints_2d.size() + j] = 5 * root_tran_smooth_weight * p[j]  * scale; // mm to m
	//	else if (j < 6)
	//		hx[2 * joints_2d.size() + j] = 10 * root_rot_smooth_weight * p[j]  * 180 / M_PI; //rad to deg
	//	else
	//		hx[2 * joints_2d.size() + j] = 5 * joint_rot_smooth_weight * p[j]  * 180 / M_PI; //rad to deg
	//}
	if (bone_set.find(sk_source->getRoot()->getName().get_str()) != bone_set.end()) //root is in
	{
		for (int j = 0; j < dof; j++)
		{
			float target_value = p[j];
			if (j < 3)
				hx[2 * joints_2d.size() + j] = 5 * weights[j] * p[j] * scale; // mm to m
			else if (j < 6)
				hx[2 * joints_2d.size() + j] = 10 * weights[j] * p[j] * 180 / M_PI; //rad to deg
			else
				hx[2 * joints_2d.size() + j] = 5 * weights[j] * p[j] * 180 / M_PI; //rad to deg
		}
	}
	else

	{
		for (int j = 0; j < dof; j++)
		{
			float target_value = p[j];
			
			hx[2 * joints_2d.size() + j] = 5 * weights[j] * p[j] * 180 / M_PI; //rad to deg
		}
	}

	
}

//void jacos_posesolver(float* p, float* jac, int m, int n, void* data)
//{
//	auto ext_data = reinterpret_cast<levmar_UserData*>(data);
//	memset(jac, 0, sizeof(float) * m * n);
//
//	
//
//	auto matrixIndex = [](unsigned int _i, unsigned int _j, unsigned int m) {
//		return _i * m + _j;
//	};
//	ZSkeleton::Ptr sk_source = ext_data->sk_source;
//	std::map<ZName, unsigned int> index_map = ext_data->index_map;
//	vector<string> joints_names = ext_data->joints_names;
//	vector < cv::Point2i> joints_2d = ext_data->joints_2d;
//	GCameraPers cur_view = ext_data->view;
//	float joint_rot_smooth_weight = ext_data->joint_rot_smooth_weight;
//	float root_rot_smooth_weight = ext_data->root_rot_smooth_weight;
//	float root_tran_smooth_weight = ext_data->root_tran_smooth_weight;
//	float joint_2d_weight = ext_data->joint_2d_weight;
//	float scale = ext_data->scale;
//	vector<float> weights = ext_data->weights;
//
//	int dof = m;
//
//	for (unsigned int ii = 0; ii < sk_source->getJointNum(); ii++)
//	{
//		ZJoint::Ptr jnt = sk_source->getJointByJID(ii);
//		//if (bone_map.find(jnt->getName().get_str()) == bone_map.end())
//		if (index_map.find(jnt->getName().get_str()) == index_map.end())
//		{
//			continue;
//		}
//
//		if (jnt->getDegrees() == 0)
//		{
//			continue;
//		}
//
//		unsigned int bone_index = index_map[jnt->getName()];
//		unsigned int degree_index = 0;
//
//		ZMath::ZVector3D rotate_vec = jnt->getRotate();
//		ZMath::ZVector3D translate_vec = jnt->getTranslation();
//
//
//
//		for (unsigned int jj = 0; jj < 3; jj++)
//		{
//			if (!jnt->get_locked(jj))
//			{
//				translate_vec(jj) = p[bone_index + degree_index];
//				degree_index++;
//			}
//		}
//
//		for (unsigned int jj = 3; jj < 6; jj++)
//		{
//			if (!jnt->get_locked(jj))
//			{
//
//				rotate_vec(jj - 3) = p[bone_index + degree_index];
//
//				degree_index++;
//			}
//		}
//		jnt->translate(translate_vec);
//		jnt->rotate(rotate_vec);
//	}
//
//
//
//	for (int j = 0; j < joints_2d.size(); j++)
//	{
//
//
//		string joint_name;
//		
//
//		joint_name = joints_names[j];
//
//		EVec3f cur_pos = sk_source->getJointByName(joint_name)->worldMatrix().translation().cast<float>() * scale;
//
//
//		//eigen column major, DGlobal row major
//		EMatXf temp_jaco = EMatXf::Zero(3, dof);
//
//		//sk_.DGlobal(sk_.findBone(joint_name)->boneIndex, GVector3(cur_pos.x(), cur_pos.y(), cur_pos.z()), true, true, 1.0, temp_jaco.data());
//		ZJoint::Ptr tmp_jnt = sk_source->getJointByName(joint_name);
//		while (tmp_jnt)
//		{
//			if (tmp_jnt->getDegrees() == 0)
//			{
//				if (tmp_jnt->getParent())
//				{
//					tmp_jnt = std::dynamic_pointer_cast<ZJoint>(tmp_jnt->getParent());
//				}
//				else {
//					tmp_jnt = nullptr;
//				}
//				continue;
//			}
//			if (index_map.find(tmp_jnt->getName()) == index_map.end())
//			{
//				if (tmp_jnt->getParent())
//				{
//					tmp_jnt = std::dynamic_pointer_cast<ZJoint>(tmp_jnt->getParent());
//				}
//				else {
//					tmp_jnt = nullptr;
//				}
//				continue;
//			}
//			/*if (bone_map.find(tmp_jnt->getName().get_str()) == bone_map.end())
//			{
//				if (tmp_jnt->getParent())
//				{
//					tmp_jnt = std::dynamic_pointer_cast<ZJoint>(tmp_jnt->getParent());
//				}
//				else {
//					tmp_jnt = nullptr;
//				}
//				continue;
//			}*/
//
//			unsigned int bone_index = index_map.at(tmp_jnt->getName());
//			unsigned int degree_index = 0;
//
//			ZMath::ZTransform3D parent_world_matrix = ZMath::ZIdentify4D();
//			if (tmp_jnt->getParent())
//			{
//				parent_world_matrix = tmp_jnt->getParent()->worldMatrix();
//			}
//
//			ZMath::ZVector3D axis_x_trans = (parent_world_matrix * ZMath::ZVector4D::UnitX()).head<3>();
//			ZMath::ZVector3D axis_y_trans = (parent_world_matrix * ZMath::ZVector4D::UnitY()).head<3>();
//			ZMath::ZVector3D axis_z_trans = (parent_world_matrix * ZMath::ZVector4D::UnitZ()).head<3>();
//
//			ZMath::ZVector3D axis_x_rotate = (parent_world_matrix * tmp_jnt->TMatrix() * tmp_jnt->JOMatrix() * tmp_jnt->RMatrix() * ZMath::ZVector4D::UnitX()).head(3);
//			ZMath::ZVector3D _rotate = tmp_jnt->getRotate();
//			ZMath::ZVector3D axis_y_rotate = (parent_world_matrix * tmp_jnt->TMatrix() * tmp_jnt->JOMatrix() * ZMath::ZRotationZ(_rotate(2)) * ZMath::ZRotationY(_rotate(1)) * ZMath::ZVector4D::UnitY()).head(3);
//			ZMath::ZVector3D axis_z_rotate = (parent_world_matrix * tmp_jnt->TMatrix() * tmp_jnt->JOMatrix() * ZMath::ZRotationZ(_rotate(2)) * ZMath::ZVector4D::UnitZ()).head(3);
//
//			ZMath::ZVector4D global_marker_pos = sk_source->getJointByName(joint_name)->worldMatrix() * ZMath::ZVector4D{ 0.0f, 0.0f, 0.0f, 1.0f };
//			ZMath::ZVector4D end_point_pos = tmp_jnt->worldMatrix().matrix() * ZMath::ZVector4D{ 0.0f, 0.0f, 0.0f, 1.0f };
//			ZMath::ZVector4D _e = global_marker_pos - end_point_pos; //mm to m
//
//			if (!tmp_jnt->get_locked(0))
//			{
//				temp_jaco(0, bone_index + degree_index) = axis_x_trans[0];
//				temp_jaco(1, bone_index + degree_index) = axis_x_trans[1];
//				temp_jaco(2, bone_index + degree_index) = axis_x_trans[2];
//				degree_index++;
//			}
//			if (!tmp_jnt->get_locked(1))
//			{
//				temp_jaco(0, bone_index + degree_index) = axis_y_trans[0];
//				temp_jaco(1, bone_index + degree_index) = axis_y_trans[1];
//				temp_jaco(2, bone_index + degree_index) = axis_y_trans[2];
//				degree_index++;
//			}
//			if (!tmp_jnt->get_locked(2))
//			{
//				temp_jaco(0, bone_index + degree_index) = axis_z_trans[0];
//				temp_jaco(1, bone_index + degree_index) = axis_z_trans[1];
//				temp_jaco(2, bone_index + degree_index) = axis_z_trans[2];
//				degree_index++;
//			}
//			if (!tmp_jnt->get_locked(3))
//			{
//				ZMath::ZVector3D grads = axis_x_rotate.cross(_e.head<3>());
//				temp_jaco(0, bone_index + degree_index) = grads[0];
//				temp_jaco(1, bone_index + degree_index) = grads[1];
//				temp_jaco(2, bone_index + degree_index) = grads[2];
//				degree_index++;
//			}
//			if (!tmp_jnt->get_locked(4))
//			{
//				ZMath::ZVector3D grads = axis_y_rotate.cross(_e.head<3>());
//				temp_jaco(0, bone_index + degree_index) = grads[0];
//				temp_jaco(1, bone_index + degree_index) = grads[1];
//				temp_jaco(2, bone_index + degree_index) = grads[2];
//				degree_index++;
//			}
//			if (!tmp_jnt->get_locked(5))
//			{
//				ZMath::ZVector3D grads = axis_z_rotate.cross(_e.head<3>());
//				temp_jaco(0, bone_index + degree_index) = grads[0];
//				temp_jaco(1, bone_index + degree_index) = grads[1];
//				temp_jaco(2, bone_index + degree_index) = grads[2];
//				degree_index++;
//			}
//			if (tmp_jnt->getParent())
//			{
//				tmp_jnt = std::dynamic_pointer_cast<ZJoint>(tmp_jnt->getParent());
//			}
//			else
//			{
//				tmp_jnt = nullptr;
//			}
//			//tmp_jnt = std::dynamic_pointer_cast<ZJoint>(tmp_jnt->getParent());
//		}
//
//
//		float d23[6];
//		cur_view.D23(GVector3(cur_pos.x(), cur_pos.y(), cur_pos.z()), d23);
//
//		Eigen::MatrixXf projMat = Eigen::Map<Eigen::Matrix<float, 2, 3, Eigen::RowMajor>>(d23);
//
//		
//		EMatXf cur_jaco = joint_2d_weight * projMat * scale * temp_jaco; //2 * dof
//
//		for (int yy = 0; yy < cur_jaco.rows(); yy++)
//		{
//			for (int xx = 0; xx < cur_jaco.cols(); xx++)
//			{
//				jac[matrixIndex(2 * j + yy, xx, m)] = cur_jaco(yy, xx);
//			}
//		}
//		
//		
//
//	}
//
//
//
//
//
//
//
//
//	EMatXf smooth_mat = EMatXf::Identity(dof, dof);
//
//	/*for (int yy = 0; yy < dof; yy++)
//	{
//		if (yy < 3)
//			smooth_mat(yy, yy) = 5 * root_tran_smooth_weight * scale;
//		else if (yy < 6)
//			smooth_mat(yy, yy) = 10 * root_rot_smooth_weight * 180 / M_PI;
//		else
//			smooth_mat(yy, yy) = 5 * joint_rot_smooth_weight * 180 / M_PI;
//	}*/
//
//	for (int yy = 0; yy < dof; yy++)
//	{
//		if (yy < 3)
//			smooth_mat(yy, yy) = 5 * weights[yy] * scale;
//		else if (yy < 6)
//			smooth_mat(yy, yy) = 10 * weights[yy] * 180 / M_PI;
//		else
//			smooth_mat(yy, yy) = 5 * weights[yy] * 180 / M_PI;
//	}
//
//	//cout << smooth_mat << endl;
//	
//	for (int yy = 0; yy < smooth_mat.rows(); yy++)
//	{
//		for (int xx = 0; xx < smooth_mat.cols(); xx++)
//		{
//			jac[matrixIndex(2 * joints_2d.size() + yy, xx, m)] = smooth_mat(yy, xx);
//		}
//	}
//}


void jacos_posesolver_v2(float* p, float* jac, int m, int n, void* data)
{
	auto ext_data = reinterpret_cast<levmar_UserData*>(data);
	memset(jac, 0, sizeof(float) * m * n);



	auto matrixIndex = [](unsigned int _i, unsigned int _j, unsigned int m) {
		return _i * m + _j;
	};
	ZSkeleton::Ptr sk_source = ext_data->sk_source;
	std::map<ZName, unsigned int> index_map = ext_data->index_map;
	vector<string> joints_names = ext_data->joints_names;
	vector < cv::Point2i> joints_2d = ext_data->joints_2d;
	GCameraPers cur_view = ext_data->view;
	float joint_rot_smooth_weight = ext_data->joint_rot_smooth_weight;
	float root_rot_smooth_weight = ext_data->root_rot_smooth_weight;
	float root_tran_smooth_weight = ext_data->root_tran_smooth_weight;
	float joint_2d_weight = ext_data->joint_2d_weight;
	float scale = ext_data->scale;
	vector<float> weights = ext_data->weights;
	set<string> bone_set = ext_data->bone_set;

	int dof = m;

	for (unsigned int ii = 0; ii < sk_source->getJointNum(); ii++)
	{
		ZJoint::Ptr jnt = sk_source->getJointByJID(ii);
		//if (bone_map.find(jnt->getName().get_str()) == bone_map.end())
		if (index_map.find(jnt->getName().get_str()) == index_map.end())
		{
			continue;
		}

		if (jnt->getDegrees() == 0)
		{
			continue;
		}

		unsigned int bone_index = index_map[jnt->getName()];
		unsigned int degree_index = 0;

		ZMath::ZVector3D rotate_vec = jnt->getRotate();
		ZMath::ZVector3D translate_vec = jnt->getTranslation();



		for (unsigned int jj = 0; jj < 3; jj++)
		{
			if (!jnt->get_locked(jj))
			{
				translate_vec(jj) = p[bone_index + degree_index];
				degree_index++;
			}
		}

		for (unsigned int jj = 3; jj < 6; jj++)
		{
			if (!jnt->get_locked(jj))
			{

				rotate_vec(jj - 3) = p[bone_index + degree_index];

				degree_index++;
			}
		}
		jnt->translate(translate_vec);
		jnt->rotate(rotate_vec);
	}



	for (int j = 0; j < joints_2d.size(); j++)
	{


		string joint_name;


		joint_name = joints_names[j];

		EVec3f cur_pos = sk_source->getJointByName(joint_name)->worldMatrix().translation().cast<float>() * scale;


		//eigen column major, DGlobal row major
		EMatXf temp_jaco = EMatXf::Zero(3, dof);

		//sk_.DGlobal(sk_.findBone(joint_name)->boneIndex, GVector3(cur_pos.x(), cur_pos.y(), cur_pos.z()), true, true, 1.0, temp_jaco.data());
		ZJoint::Ptr tmp_jnt = sk_source->getJointByName(joint_name);
		while (tmp_jnt)
		{
			if (tmp_jnt->getDegrees() == 0)
			{
				if (tmp_jnt->getParent())
				{
					tmp_jnt = std::dynamic_pointer_cast<ZJoint>(tmp_jnt->getParent());
				}
				else {
					tmp_jnt = nullptr;
				}
				continue;
			}
			if (index_map.find(tmp_jnt->getName()) == index_map.end())
			{
				if (tmp_jnt->getParent())
				{
					tmp_jnt = std::dynamic_pointer_cast<ZJoint>(tmp_jnt->getParent());
				}
				else {
					tmp_jnt = nullptr;
				}
				continue;
			}
			/*if (bone_map.find(tmp_jnt->getName().get_str()) == bone_map.end())
			{
				if (tmp_jnt->getParent())
				{
					tmp_jnt = std::dynamic_pointer_cast<ZJoint>(tmp_jnt->getParent());
				}
				else {
					tmp_jnt = nullptr;
				}
				continue;
			}*/

			unsigned int bone_index = index_map.at(tmp_jnt->getName());
			unsigned int degree_index = 0;

			ZMath::ZTransform3D parent_world_matrix = ZMath::ZIdentify4D();
			if (tmp_jnt->getParent())
			{
				parent_world_matrix = tmp_jnt->getParent()->worldMatrix();
			}

			ZMath::ZVector3D axis_x_trans = (parent_world_matrix * ZMath::ZVector4D::UnitX()).head<3>();
			ZMath::ZVector3D axis_y_trans = (parent_world_matrix * ZMath::ZVector4D::UnitY()).head<3>();
			ZMath::ZVector3D axis_z_trans = (parent_world_matrix * ZMath::ZVector4D::UnitZ()).head<3>();

			ZMath::ZVector3D axis_x_rotate = (parent_world_matrix * tmp_jnt->TMatrix() * tmp_jnt->JOMatrix() * tmp_jnt->RMatrix() * ZMath::ZVector4D::UnitX()).head(3);
			ZMath::ZVector3D _rotate = tmp_jnt->getRotate();
			ZMath::ZVector3D axis_y_rotate = (parent_world_matrix * tmp_jnt->TMatrix() * tmp_jnt->JOMatrix() * ZMath::ZRotationZ(_rotate(2)) * ZMath::ZRotationY(_rotate(1)) * ZMath::ZVector4D::UnitY()).head(3);
			ZMath::ZVector3D axis_z_rotate = (parent_world_matrix * tmp_jnt->TMatrix() * tmp_jnt->JOMatrix() * ZMath::ZRotationZ(_rotate(2)) * ZMath::ZVector4D::UnitZ()).head(3);

			ZMath::ZVector4D global_marker_pos = sk_source->getJointByName(joint_name)->worldMatrix() * ZMath::ZVector4D{ 0.0f, 0.0f, 0.0f, 1.0f };
			ZMath::ZVector4D end_point_pos = tmp_jnt->worldMatrix().matrix() * ZMath::ZVector4D{ 0.0f, 0.0f, 0.0f, 1.0f };
			ZMath::ZVector4D _e = global_marker_pos - end_point_pos; //mm to m

			if (!tmp_jnt->get_locked(0))
			{
				temp_jaco(0, bone_index + degree_index) = axis_x_trans[0];
				temp_jaco(1, bone_index + degree_index) = axis_x_trans[1];
				temp_jaco(2, bone_index + degree_index) = axis_x_trans[2];
				degree_index++;
			}
			if (!tmp_jnt->get_locked(1))
			{
				temp_jaco(0, bone_index + degree_index) = axis_y_trans[0];
				temp_jaco(1, bone_index + degree_index) = axis_y_trans[1];
				temp_jaco(2, bone_index + degree_index) = axis_y_trans[2];
				degree_index++;
			}
			if (!tmp_jnt->get_locked(2))
			{
				temp_jaco(0, bone_index + degree_index) = axis_z_trans[0];
				temp_jaco(1, bone_index + degree_index) = axis_z_trans[1];
				temp_jaco(2, bone_index + degree_index) = axis_z_trans[2];
				degree_index++;
			}
			if (!tmp_jnt->get_locked(3))
			{
				ZMath::ZVector3D grads = axis_x_rotate.cross(_e.head<3>());
				temp_jaco(0, bone_index + degree_index) = grads[0];
				temp_jaco(1, bone_index + degree_index) = grads[1];
				temp_jaco(2, bone_index + degree_index) = grads[2];
				degree_index++;
			}
			if (!tmp_jnt->get_locked(4))
			{
				ZMath::ZVector3D grads = axis_y_rotate.cross(_e.head<3>());
				temp_jaco(0, bone_index + degree_index) = grads[0];
				temp_jaco(1, bone_index + degree_index) = grads[1];
				temp_jaco(2, bone_index + degree_index) = grads[2];
				degree_index++;
			}
			if (!tmp_jnt->get_locked(5))
			{
				ZMath::ZVector3D grads = axis_z_rotate.cross(_e.head<3>());
				temp_jaco(0, bone_index + degree_index) = grads[0];
				temp_jaco(1, bone_index + degree_index) = grads[1];
				temp_jaco(2, bone_index + degree_index) = grads[2];
				degree_index++;
			}
			if (tmp_jnt->getParent())
			{
				tmp_jnt = std::dynamic_pointer_cast<ZJoint>(tmp_jnt->getParent());
			}
			else
			{
				tmp_jnt = nullptr;
			}
			//tmp_jnt = std::dynamic_pointer_cast<ZJoint>(tmp_jnt->getParent());
		}


		float d23[6];
		cur_view.D23(GVector3(cur_pos.x(), cur_pos.y(), cur_pos.z()), d23);

		Eigen::MatrixXf projMat = Eigen::Map<Eigen::Matrix<float, 2, 3, Eigen::RowMajor>>(d23);


		EMatXf cur_jaco = joint_2d_weight * projMat * scale * temp_jaco; //2 * dof

		for (int yy = 0; yy < cur_jaco.rows(); yy++)
		{
			for (int xx = 0; xx < cur_jaco.cols(); xx++)
			{
				jac[matrixIndex(2 * j + yy, xx, m)] = cur_jaco(yy, xx);
			}
		}



	}








	EMatXf smooth_mat = EMatXf::Identity(dof, dof);

	/*for (int yy = 0; yy < dof; yy++)
	{
		if (yy < 3)
			smooth_mat(yy, yy) = 5 * root_tran_smooth_weight * scale;
		else if (yy < 6)
			smooth_mat(yy, yy) = 10 * root_rot_smooth_weight * 180 / M_PI;
		else
			smooth_mat(yy, yy) = 5 * joint_rot_smooth_weight * 180 / M_PI;
	}*/
	if (bone_set.find(sk_source->getRoot()->getName().get_str()) != bone_set.end()) //root is in
	{
		for (int yy = 0; yy < dof; yy++)
		{
			if (yy < 3)
				smooth_mat(yy, yy) = 5 * weights[yy] * scale;
			else if (yy < 6)
				smooth_mat(yy, yy) = 10 * weights[yy] * 180 / M_PI;
			else
				smooth_mat(yy, yy) = 5 * weights[yy] * 180 / M_PI;
		}
	}
	else
	{
		for (int yy = 0; yy < dof; yy++)
		{
			
			smooth_mat(yy, yy) = 5 * weights[yy] * 180 / M_PI;
		}
	}
	

	//cout << smooth_mat << endl;

	for (int yy = 0; yy < smooth_mat.rows(); yy++)
	{
		for (int xx = 0; xx < smooth_mat.cols(); xx++)
		{
			jac[matrixIndex(2 * joints_2d.size() + yy, xx, m)] = smooth_mat(yy, xx);
		}
	}
}


void setMSkelPoseByMap(ZSkeleton::Ptr sk_source, const map<string, EVecXf>& pose_map)
{
	for (auto iter = pose_map.begin(); iter != pose_map.end(); iter++)
	{
		string bone_name = iter->first;

		ZJoint::Ptr jnt = sk_source->getJointByName(bone_name);

		if (!jnt->getParent())
		{
			EVecXf pp = iter->second;
			jnt->translate(ZMath::ZVector3D(pp[0], pp[1], pp[2]));
			jnt->rotate(ZMath::ZVector3D(pp[3], pp[4], pp[5]));

		}

		else
		{
			EVecXf pp = iter->second;

			jnt->rotate(ZMath::ZVector3D(pp[0], pp[1], pp[2]));
		}
	}
}

void extractMSkelPoseToMap(ZSkeleton::Ptr sk_source, map<string, EVecXf>& pose_map)
{
	for (auto iter = pose_map.begin(); iter != pose_map.end(); iter++)
	{
		string bone_name = iter->first;

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
			pose_map[iter->first] = pp;

		}

		else
		{
			EVecXf pp = EVecXf::Zero(3);
			
			pp[0] = jnt->getRotate()[0];
			pp[1] = jnt->getRotate()[1];
			pp[2] = jnt->getRotate()[2];
			pose_map[iter->first] = pp;
		}
	}
}


//void adjust_by_joints2d_mskel(ZSkeleton::Ptr sk_source, map<string, EVecXf>& pose_map, const map<string, string>& bone_map, const GCameraPers& cur_view, const vector<cv::Point2i>& joints_2d, const vector<string>& joints_names, int flag, float scale, int iterMax)
//{
//	std::map<ZName, unsigned int> index_map;
//	
//	//construct valid map
//	unsigned int index_index = 0;
//	for (unsigned int ii = 0; ii < sk_source->getJointNum(); ii++)
//	{
//		ZJoint::Ptr jnt = sk_source->getJointByName(sk_source->getJointByJID(ii)->getName());
//		if (!jnt || jnt->getDegrees() == 0 || bone_map.find(jnt->getName().get_str()) == bone_map.end())
//		{
//			continue;
//		}
//
//		index_map[jnt->getName()] = index_index;
//		index_index += 3;
//		if (!jnt->getParent())
//		{
//			index_index += 3;
//		}
//	}
//
//
//	int dof = 0; // input dimension
//	for (unsigned int ii = 0; ii < sk_source->getJointNum(); ii++)
//	{
//		//if (bone_map.find(sk_source->getJointByJID(ii)->getName().get_str()) == bone_map.end())
//		if (index_map.find(sk_source->getJointByJID(ii)->getName().get_str()) == index_map.end())
//		{
//			continue;
//		}
//		if (!sk_source->getJointByJID(ii)->getParent())
//		{
//			dof += 6;
//		}
//		else
//		{
//			dof += 3;
//		}
//	}
//
//
//
//	EVecXf p = EVecXf::Zero(dof);
//	setMSkelPoseByMap(sk_source, pose_map);
//	for (unsigned int ii = 0; ii < sk_source->getJointNum(); ii++)
//	{
//		//if (bone_map.find(sk_source->getJointByJID(ii)->getName().get_str()) == bone_map.end())
//		if (index_map.find(sk_source->getJointByJID(ii)->getName().get_str()) == index_map.end())
//		{
//			continue;
//		}
//
//
//		ZJoint::Ptr bone = sk_source->getJointByJID(ii);
//
//		unsigned int jnt_index = index_map[bone->getName()];
//		unsigned int degree_index = 0;
//		if (!bone->getParent())
//		{
//
//			if (!bone->get_locked(0))
//			{
//				//p[jnt_index + degree_index] = sk_virtual->getTranslation()(0) * height_scale;
//				p[jnt_index + degree_index] = bone->getTranslation()[0];
//				//x[jnt_index + degree_index + n] = weight_reg_trans * new_root(0) * height_scale;
//				degree_index++;
//			}
//			if (!bone->get_locked(1))
//			{
//				//p[jnt_index + degree_index] = sk_virtual->getTranslation()(1) * height_scale;
//				p[jnt_index + degree_index] = bone->getTranslation()[1];
//				//x[jnt_index + degree_index + n] = 0.0f * new_root(1) * height_scale;
//				degree_index++;
//			}
//			if (!bone->get_locked(2))
//			{
//				//p[jnt_index + degree_index] = sk_virtual->getTranslation()(2) * height_scale;
//				p[jnt_index + degree_index] = bone->getTranslation()[2];
//				//x[jnt_index + degree_index + n] = weight_reg_trans * new_root(2) * height_scale;
//				degree_index++;
//			}
//		}
//		if (!bone->get_locked(3))
//		{
//
//			p[jnt_index + degree_index] = bone->getRotate()(0);
//			//x[jnt_index + degree_index + n] = weight_ro * weight_mul_ro * vir_bone->getRotate()(0);
//
//			degree_index++;
//		}
//		if (!bone->get_locked(4))
//		{
//
//			p[jnt_index + degree_index] = bone->getRotate()(1);
//			//x[jnt_index + degree_index + n] = weight_ro * vir_bone->getRotate()(1);
//
//			degree_index++;
//		}
//		if (!bone->get_locked(5))
//		{
//
//			p[jnt_index + degree_index] = bone->getRotate()(2);
//			//x[jnt_index + degree_index + n] = weight_ro * vir_bone->getRotate()(2);
//			degree_index++;
//		}
//	}
//
//	float joint_rot_smooth_weight = 0.01; //rad to deg
//	float root_rot_smooth_weight = 0.01; //rad to deg
//	float root_tran_smooth_weight = 0.1; //mm to m unit
//	float joint_2d_weight = 0.01;
//
//	if (flag == 1)//refine root pos only
//	{
//		joint_rot_smooth_weight = 10000.0;
//		root_rot_smooth_weight = 10000.0;
//	}
//	else if (flag == 2) //refine root only
//	{
//		joint_rot_smooth_weight = 10000.0;
//	}
//
//	//variable count: orig input joint angle dimension; pc count: pca dimension; coef pc count * variable count
//	//EMatXf coef_t = pca_model_.getProjMat().cast<float>().transpose();
//	for (int i = 0; i < iterMax; i++)
//	{
//		EMatXf jacobian = EMatXf::Zero(2 * joints_2d.size() + dof, dof);
//		EVecXf delta = EVecXf::Zero(2 * joints_2d.size() + dof);
//
//		
//		for (unsigned int ii = 0; ii < sk_source->getJointNum(); ii++)
//		{
//			ZJoint::Ptr jnt = sk_source->getJointByJID(ii);
//			//if (bone_map.find(jnt->getName().get_str()) == bone_map.end())
//			if (index_map.find(jnt->getName().get_str()) == index_map.end())
//			{
//				continue;
//			}
//
//			if (jnt->getDegrees() == 0)
//			{
//				continue;
//			}
//
//			unsigned int bone_index = index_map[jnt->getName()];
//			unsigned int degree_index = 0;
//
//			ZMath::ZVector3D rotate_vec = jnt->getRotate();
//			ZMath::ZVector3D translate_vec = jnt->getTranslation();
//
//
//
//			for (unsigned int jj = 0; jj < 3; jj++)
//			{
//				if (!jnt->get_locked(jj))
//				{
//					translate_vec(jj) = p[bone_index + degree_index];
//					degree_index++;
//				}
//			}
//
//			for (unsigned int jj = 3; jj < 6; jj++)
//			{
//				if (!jnt->get_locked(jj))
//				{
//
//					rotate_vec(jj - 3) = p[bone_index + degree_index];
//
//					degree_index++;
//				}
//			}
//			jnt->translate(translate_vec);
//			jnt->rotate(rotate_vec);
//		}
//
//
//
//		for (int j = 0; j < joints_2d.size(); j++)
//		{
//
//
//			string joint_name;
//			//if (j == 0)
//			//	joint_name = joint_names_needed_["rshoulder"];// "Shoulder_R-ShoulderPart1_R";
//			//else if (j == 1)
//			//	joint_name = joint_names_needed_["lshoulder"];// "Shoulder_L-ShoulderPart1_L";
//			//else
//			//	joint_name = joint_names_needed_["upper"];
//
//			joint_name = joints_names[j];
//			
//			EVec3f cur_pos = sk_source->getJointByName(joint_name)->worldMatrix().translation().cast<float>() * scale;
//
//
//			//eigen column major, DGlobal row major
//			EMatXf temp_jaco = EMatXf::Zero(3, dof);
//
//			//sk_.DGlobal(sk_.findBone(joint_name)->boneIndex, GVector3(cur_pos.x(), cur_pos.y(), cur_pos.z()), true, true, 1.0, temp_jaco.data());
//			ZJoint::Ptr tmp_jnt = sk_source->getJointByName(joint_name);
//			while (tmp_jnt)
//			{
//				if (tmp_jnt->getDegrees() == 0)
//				{
//					if (tmp_jnt->getParent())
//					{
//						tmp_jnt = std::dynamic_pointer_cast<ZJoint>(tmp_jnt->getParent());
//					}
//					else {
//						tmp_jnt = nullptr;
//					}
//					continue;
//				}
//				if (index_map.find(tmp_jnt->getName()) == index_map.end())
//				{
//					if (tmp_jnt->getParent())
//					{
//						tmp_jnt = std::dynamic_pointer_cast<ZJoint>(tmp_jnt->getParent());
//					}
//					else {
//						tmp_jnt = nullptr;
//					}
//					continue;
//				}
//				/*if (bone_map.find(tmp_jnt->getName().get_str()) == bone_map.end())
//				{
//					if (tmp_jnt->getParent())
//					{
//						tmp_jnt = std::dynamic_pointer_cast<ZJoint>(tmp_jnt->getParent());
//					}
//					else {
//						tmp_jnt = nullptr;
//					}
//					continue;
//				}*/
//
//				unsigned int bone_index = index_map.at(tmp_jnt->getName());
//				unsigned int degree_index = 0;
//
//				ZMath::ZTransform3D parent_world_matrix = ZMath::ZIdentify4D();
//				if (tmp_jnt->getParent())
//				{
//					parent_world_matrix = tmp_jnt->getParent()->worldMatrix();
//				}
//
//				ZMath::ZVector3D axis_x_trans = (parent_world_matrix * ZMath::ZVector4D::UnitX()).head<3>();
//				ZMath::ZVector3D axis_y_trans = (parent_world_matrix * ZMath::ZVector4D::UnitY()).head<3>();
//				ZMath::ZVector3D axis_z_trans = (parent_world_matrix * ZMath::ZVector4D::UnitZ()).head<3>();
//
//				ZMath::ZVector3D axis_x_rotate = (parent_world_matrix * tmp_jnt->TMatrix() * tmp_jnt->JOMatrix() * tmp_jnt->RMatrix() * ZMath::ZVector4D::UnitX()).head(3);
//				ZMath::ZVector3D _rotate = tmp_jnt->getRotate();
//				ZMath::ZVector3D axis_y_rotate = (parent_world_matrix * tmp_jnt->TMatrix() * tmp_jnt->JOMatrix() * ZMath::ZRotationZ(_rotate(2)) * ZMath::ZRotationY(_rotate(1)) * ZMath::ZVector4D::UnitY()).head(3);
//				ZMath::ZVector3D axis_z_rotate = (parent_world_matrix * tmp_jnt->TMatrix() * tmp_jnt->JOMatrix() * ZMath::ZRotationZ(_rotate(2)) * ZMath::ZVector4D::UnitZ()).head(3);
//
//				ZMath::ZVector4D global_marker_pos = sk_source->getJointByName(joint_name)->worldMatrix() * ZMath::ZVector4D{ 0.0f, 0.0f, 0.0f, 1.0f };
//				ZMath::ZVector4D end_point_pos = tmp_jnt->worldMatrix().matrix() * ZMath::ZVector4D{ 0.0f, 0.0f, 0.0f, 1.0f };
//				ZMath::ZVector4D _e = global_marker_pos - end_point_pos; //mm to m
//
//				if (!tmp_jnt->get_locked(0))
//				{
//					temp_jaco(0, bone_index + degree_index) = axis_x_trans[0];
//					temp_jaco(1, bone_index + degree_index) = axis_x_trans[1];
//					temp_jaco(2, bone_index + degree_index) = axis_x_trans[2];
//					degree_index++;
//				}
//				if (!tmp_jnt->get_locked(1))
//				{
//					temp_jaco(0, bone_index + degree_index) = axis_y_trans[0];
//					temp_jaco(1, bone_index + degree_index) = axis_y_trans[1];
//					temp_jaco(2, bone_index + degree_index) = axis_y_trans[2];
//					degree_index++;
//				}
//				if (!tmp_jnt->get_locked(2))
//				{
//					temp_jaco(0, bone_index + degree_index) = axis_z_trans[0];
//					temp_jaco(1, bone_index + degree_index) = axis_z_trans[1];
//					temp_jaco(2, bone_index + degree_index) = axis_z_trans[2];
//					degree_index++;
//				}
//				if (!tmp_jnt->get_locked(3))
//				{
//					ZMath::ZVector3D grads = axis_x_rotate.cross(_e.head<3>());
//					temp_jaco(0, bone_index + degree_index) = grads[0];
//					temp_jaco(1, bone_index + degree_index) = grads[1];
//					temp_jaco(2, bone_index + degree_index) = grads[2];
//					degree_index++;
//				}
//				if (!tmp_jnt->get_locked(4))
//				{
//					ZMath::ZVector3D grads = axis_y_rotate.cross(_e.head<3>());
//					temp_jaco(0, bone_index + degree_index) = grads[0];
//					temp_jaco(1, bone_index + degree_index) = grads[1];
//					temp_jaco(2, bone_index + degree_index) = grads[2];
//					degree_index++;
//				}
//				if (!tmp_jnt->get_locked(5))
//				{
//					ZMath::ZVector3D grads = axis_z_rotate.cross(_e.head<3>());
//					temp_jaco(0, bone_index + degree_index) = grads[0];
//					temp_jaco(1, bone_index + degree_index) = grads[1];
//					temp_jaco(2, bone_index + degree_index) = grads[2];
//					degree_index++;
//				}
//				if (tmp_jnt->getParent())
//				{
//					tmp_jnt = std::dynamic_pointer_cast<ZJoint>(tmp_jnt->getParent());
//				}
//				else
//				{
//					tmp_jnt = nullptr;
//				}
//				//tmp_jnt = std::dynamic_pointer_cast<ZJoint>(tmp_jnt->getParent());
//			}
//
//			
//			float d23[6];
//			cur_view.D23(GVector3(cur_pos.x(), cur_pos.y(), cur_pos.z()), d23);
//
//			GVector2 cur_pos_2d = cur_view.Project(GVector3(cur_pos.x(), cur_pos.y(), cur_pos.z()));
//
//			Eigen::MatrixXf projMat = Eigen::Map<Eigen::Matrix<float, 2, 3, Eigen::RowMajor>>(d23);
//
//			//EMatXf temp_jaco_joint = temp_jaco.block(6, 0, sk_.NumFreedoms() - 6, 3);
//
//			jacobian.block(2 * j, 0, 2, dof) = joint_2d_weight * projMat * scale * temp_jaco;
//
//
//			delta(2 * j + 0) = joint_2d_weight * (0 - (cur_pos_2d.x - joints_2d[j].x));
//			delta(2 * j + 1) = joint_2d_weight * (0 - (cur_pos_2d.y - joints_2d[j].y));
//
//		}
//
//
//
//
//
//
//
//
//		EMatXf smooth_mat = EMatXf::Identity(dof, dof);
//
//		for (int yy = 0; yy < dof; yy++)
//		{
//			if (yy < 3)
//				smooth_mat(yy, yy) = 5 * root_tran_smooth_weight * scale;
//			else if (yy < 6)
//				smooth_mat(yy, yy) = 10 * root_rot_smooth_weight * 180 / M_PI;
//			else
//				smooth_mat(yy, yy) = 5 * joint_rot_smooth_weight * 180 / M_PI;
//		}
//
//
//		jacobian.block(2 * joints_2d.size(), 0, dof, dof) = smooth_mat;
//
//		//jacobian.block(3 * virtual_marker_locals_inputface.size() + 9, 0, 9, sk_.NumFreedoms()) = 1 * smooth_weight * coef_t.block(9, 0, 9, pca_model_.getPCCount());
//		for (int j = 0; j < dof; j++)
//		{
//			float target_value = p[j];
//			if (j < 3)
//				delta(2 * joints_2d.size() + j) = 5 * root_tran_smooth_weight * (0 - (p[j] - target_value)) * scale; // mm to m
//			else if (j < 6)
//				delta(2 * joints_2d.size() + j) = 10 * root_rot_smooth_weight * (0 - (p[j] - target_value)) * 180 / M_PI; //rad to deg
//			else
//				delta(2 * joints_2d.size() + j) = 5 * joint_rot_smooth_weight * (0 - (p[j] - target_value)) * 180 / M_PI; //rad to deg
//		}
//
//		
//		EVecXf dpose = (jacobian.transpose() * jacobian).ldlt().solve(jacobian.transpose() * delta);
//
//
//		for (int j = 0; j < dof; j++)
//		{
//			if (i < iterMax / 2)
//				p[j] += dpose[j] * 0.1;
//			else
//				p[j] += dpose[j];
//		}
//
//		
//	}
//
//	//update back to frame
//	
//	for (unsigned int ii = 0; ii < sk_source->getJointNum(); ii++)
//	{
//		ZJoint::Ptr jnt = sk_source->getJointByJID(ii);
//		//if (bone_map.find(jnt->getName().get_str()) == bone_map.end())
//		if (index_map.find(jnt->getName().get_str()) == index_map.end())
//		{
//			continue;
//		}
//
//		if (jnt->getDegrees() == 0)
//		{
//			continue;
//		}
//
//		unsigned int bone_index = index_map[jnt->getName()];
//		unsigned int degree_index = 0;
//
//		ZMath::ZVector3D rotate_vec = jnt->getRotate();
//		ZMath::ZVector3D translate_vec = jnt->getTranslation();
//
//
//
//		for (unsigned int jj = 0; jj < 3; jj++)
//		{
//			if (!jnt->get_locked(jj))
//			{
//				translate_vec(jj) = p[bone_index + degree_index];
//				degree_index++;
//			}
//		}
//
//		for (unsigned int jj = 3; jj < 6; jj++)
//		{
//			if (!jnt->get_locked(jj))
//			{
//
//				rotate_vec(jj - 3) = p[bone_index + degree_index];
//
//				degree_index++;
//			}
//		}
//		jnt->translate(translate_vec);
//		jnt->rotate(rotate_vec);
//	}
//	extractMSkelPoseToMap(sk_source, pose_map);
//}




//void adjust_by_joints2d_mskel_levmar(ZSkeleton::Ptr sk_source, map<string, EVecXf>& pose_map, const set<string>& bone_set, const GCameraPers& cur_view, const vector<cv::Point2i>& joints_2d, const vector<string>& joints_names, const vector<string>& optmized_bones, float scale, optimizeParam param)
//{
//	std::map<ZName, unsigned int> index_map;
//
//	//construct valid map
//	unsigned int index_index = 0;
//	for (unsigned int ii = 0; ii < sk_source->getJointNum(); ii++)
//	{
//		ZJoint::Ptr jnt = sk_source->getJointByName(sk_source->getJointByJID(ii)->getName());
//		if (!jnt || jnt->getDegrees() == 0 || bone_set.find(jnt->getName().get_str()) == bone_set.end())
//		{
//			continue;
//		}
//
//		index_map[jnt->getName()] = index_index;
//		index_index += 3;
//		if (!jnt->getParent())
//		{
//			index_index += 3;
//		}
//	}
//
//
//	int dof = 0; // input dimension
//	for (unsigned int ii = 0; ii < sk_source->getJointNum(); ii++)
//	{
//		if (index_map.find(sk_source->getJointByJID(ii)->getName().get_str()) == index_map.end())
//		{
//			continue;
//		}
//		if (!sk_source->getJointByJID(ii)->getParent())
//		{
//			dof += 6;
//		}
//		else
//		{
//			dof += 3;
//		}
//	}
//
//
//
//	EVecXf p = EVecXf::Zero(dof);
//	setMSkelPoseByMap(sk_source, pose_map);
//	for (unsigned int ii = 0; ii < sk_source->getJointNum(); ii++)
//	{
//		if (index_map.find(sk_source->getJointByJID(ii)->getName().get_str()) == index_map.end())
//		{
//			continue;
//		}
//
//
//		ZJoint::Ptr bone = sk_source->getJointByJID(ii);
//
//		unsigned int jnt_index = index_map[bone->getName()];
//		unsigned int degree_index = 0;
//		if (!bone->getParent())
//		{
//
//			if (!bone->get_locked(0))
//			{
//				//p[jnt_index + degree_index] = sk_virtual->getTranslation()(0) * height_scale;
//				p[jnt_index + degree_index] = bone->getTranslation()[0];
//				//x[jnt_index + degree_index + n] = weight_reg_trans * new_root(0) * height_scale;
//				degree_index++;
//			}
//			if (!bone->get_locked(1))
//			{
//				//p[jnt_index + degree_index] = sk_virtual->getTranslation()(1) * height_scale;
//				p[jnt_index + degree_index] = bone->getTranslation()[1];
//				//x[jnt_index + degree_index + n] = 0.0f * new_root(1) * height_scale;
//				degree_index++;
//			}
//			if (!bone->get_locked(2))
//			{
//				//p[jnt_index + degree_index] = sk_virtual->getTranslation()(2) * height_scale;
//				p[jnt_index + degree_index] = bone->getTranslation()[2];
//				//x[jnt_index + degree_index + n] = weight_reg_trans * new_root(2) * height_scale;
//				degree_index++;
//			}
//		}
//		if (!bone->get_locked(3))
//		{
//
//			p[jnt_index + degree_index] = bone->getRotate()(0);
//			//x[jnt_index + degree_index + n] = weight_ro * weight_mul_ro * vir_bone->getRotate()(0);
//
//			degree_index++;
//		}
//		if (!bone->get_locked(4))
//		{
//
//			p[jnt_index + degree_index] = bone->getRotate()(1);
//			//x[jnt_index + degree_index + n] = weight_ro * vir_bone->getRotate()(1);
//
//			degree_index++;
//		}
//		if (!bone->get_locked(5))
//		{
//
//			p[jnt_index + degree_index] = bone->getRotate()(2);
//			//x[jnt_index + degree_index + n] = weight_ro * vir_bone->getRotate()(2);
//			degree_index++;
//		}
//	}
//
//	
//
//	float joint_rot_smooth_weight = param.joint_rot_smooth_weight;
//	float root_rot_smooth_weight = param.root_rot_smooth_weight;
//	float root_tran_smooth_weight = param.root_tran_smooth_weight;
//	float joint_2d_weight = param.joint_2d_weight;
//
//
//	vector<float> weights;
//	weights.resize(dof, 10.0);
//
//	for (int j = 0; j < dof; j++)
//	{
//
//		if (j < 3)
//			weights[j] = root_tran_smooth_weight * 100; 
//		else if (j < 6)
//			weights[j] = root_rot_smooth_weight * 100; 
//		else 
//			weights[j] = joint_rot_smooth_weight * 100 ; 
//	}
//
//	for (int i = 0; i < optmized_bones.size(); i++)
//	{
//		if (optmized_bones[i] == "root_t")
//		{
//			int idx = index_map[sk_source->getRoot()->getName()];
//
//			for (int j = idx; j < idx + 3; j++)
//				weights[j] = root_tran_smooth_weight;
//
//		}
//		else if (optmized_bones[i] == "root_r")
//		{
//			int idx = index_map[sk_source->getRoot()->getName()];
//
//			for (int j = idx + 3; j < idx + 6; j++)
//				weights[j] = root_rot_smooth_weight;
//
//		}
//		else
//		{
//			int idx = index_map[optmized_bones[i]];
//
//			for (int j = idx ; j < idx + 3; j++)
//				weights[j] = joint_rot_smooth_weight;
//		}
//
//	}
//
//	for (int j = 0; j < weights.size(); j++)
//		cout << weights[j] << " ";
//	cout << endl;
//
//	//if (flag == 1)//refine root pos only
//	//{
//	//	joint_rot_smooth_weight = 10000.0;
//	//	root_rot_smooth_weight = 10000.0;
//	//}
//	//else if (flag == 2) //refine root only
//	//{
//	//	joint_rot_smooth_weight = 10000.0;
//	//}
//
//
//	EVecXf x = EVecXf::Zero(2 * joints_2d.size() + dof);
//
//	for (int j = 0; j < joints_2d.size(); j++)
//	{
//		x[2 * j + 0] = joint_2d_weight * joints_2d[j].x;
//		x[2 * j + 1] = joint_2d_weight * joints_2d[j].y;
//	}
//	for (int j = 0; j < dof; j++)
//	{
//		
//		//if (j < 3)
//		//	x[2 * joints_2d.size() + j] = 5 * root_tran_smooth_weight * p[j] * scale; // mm to m
//		//else if (j < 6)
//		//	x[2 * joints_2d.size() + j] = 10 * root_rot_smooth_weight * p[j] * 180 / M_PI; //rad to deg
//		//else
//		//	x[2 * joints_2d.size() + j] = 5 * joint_rot_smooth_weight * p[j] * 180 / M_PI; //rad to deg
//
//		if (j < 3)
//			x[2 * joints_2d.size() + j] = 5 * weights[j] * p[j] * scale; // mm to m
//		else if (j < 6)
//			x[2 * joints_2d.size() + j] = 10 * weights[j] * p[j] * 180 / M_PI; //rad to deg
//		else
//			x[2 * joints_2d.size() + j] = 5 * weights[j] * p[j] * 180 / M_PI; //rad to deg
//	}
//		
//
//	float opts[LM_OPTS_SZ], info[LM_INFO_SZ];
//	opts[0] = LM_INIT_MU; opts[1] = 1E-15; opts[2] = 1E-15; opts[3] = 1E-20; opts[4] = 1E-06;
//
//	levmar_UserData ext_data;
//	ext_data.sk_source = sk_source;
//	ext_data.joints_names = joints_names;
//	ext_data.joints_2d = joints_2d;
//	ext_data.view = cur_view;
//	ext_data.index_map = index_map;
//	ext_data.joint_rot_smooth_weight = joint_rot_smooth_weight;
//	ext_data.root_rot_smooth_weight = root_rot_smooth_weight;
//	ext_data.root_tran_smooth_weight = root_tran_smooth_weight;
//	ext_data.joint_2d_weight = joint_2d_weight;
//	ext_data.scale = scale;
//	ext_data.weights = weights;
//
//	auto start = std::chrono::high_resolution_clock::now();
//
//	cout << "before: " << p.transpose() << endl;
//	int ret = slevmar_der(disval_posesolver, jacos_posesolver, p.data(), x.data(), p.size(), x.size(), 1000, opts, info, nullptr, nullptr, &ext_data);
//	cout << "after: " << p.transpose() << endl;
//
//	for (int i = 0; i < LM_INFO_SZ; i++)
//		cout << "info " << i << " : " << info[i] << endl;
//
//	//update back to frame
//
//	for (unsigned int ii = 0; ii < sk_source->getJointNum(); ii++)
//	{
//		ZJoint::Ptr jnt = sk_source->getJointByJID(ii);
//		if (index_map.find(jnt->getName().get_str()) == index_map.end())
//		{
//			continue;
//		}
//
//		if (jnt->getDegrees() == 0)
//		{
//			continue;
//		}
//
//		unsigned int bone_index = index_map[jnt->getName()];
//		unsigned int degree_index = 0;
//
//		ZMath::ZVector3D rotate_vec = jnt->getRotate();
//		ZMath::ZVector3D translate_vec = jnt->getTranslation();
//
//
//
//		for (unsigned int jj = 0; jj < 3; jj++)
//		{
//			if (!jnt->get_locked(jj))
//			{
//				translate_vec(jj) = p[bone_index + degree_index];
//				degree_index++;
//			}
//		}
//
//		for (unsigned int jj = 3; jj < 6; jj++)
//		{
//			if (!jnt->get_locked(jj))
//			{
//
//				rotate_vec(jj - 3) = p[bone_index + degree_index];
//
//				degree_index++;
//			}
//		}
//		jnt->translate(translate_vec);
//		jnt->rotate(rotate_vec);
//	}
//	extractMSkelPoseToMap(sk_source, pose_map);
//}




void adjust_by_joints2d_mskel_levmar_v2(ZSkeleton::Ptr sk_source, map<string, EVecXf>& pose_map, const set<string>& bone_set, const GCameraPers& cur_view, const vector<cv::Point2i>& joints_2d, const vector<string>& joints_names, float scale, optimizeParam param)
{
	cout << "you are using the version created on 05/25/2022" << endl;
	std::map<ZName, unsigned int> index_map;

	//construct valid map
	unsigned int index_index = 0;
	for (unsigned int ii = 0; ii < sk_source->getJointNum(); ii++)
	{
		ZJoint::Ptr jnt = sk_source->getJointByName(sk_source->getJointByJID(ii)->getName());
		if (!jnt || jnt->getDegrees() == 0 || bone_set.find(jnt->getName().get_str()) == bone_set.end())
		{
			continue;
		}

		index_map[jnt->getName()] = index_index;
		index_index += 3;
		if (!jnt->getParent())
		{
			index_index += 3;
		}
	}


	int dof = 0; // input dimension
	for (unsigned int ii = 0; ii < sk_source->getJointNum(); ii++)
	{
		if (index_map.find(sk_source->getJointByJID(ii)->getName().get_str()) == index_map.end())
		{
			continue;
		}
		if (!sk_source->getJointByJID(ii)->getParent())
		{
			dof += 6;
		}
		else
		{
			dof += 3;
		}
	}



	EVecXf p = EVecXf::Zero(dof);
	setMSkelPoseByMap(sk_source, pose_map);
	for (unsigned int ii = 0; ii < sk_source->getJointNum(); ii++)
	{
		if (index_map.find(sk_source->getJointByJID(ii)->getName().get_str()) == index_map.end())
		{
			continue;
		}


		ZJoint::Ptr bone = sk_source->getJointByJID(ii);

		unsigned int jnt_index = index_map[bone->getName()];
		unsigned int degree_index = 0;
		if (!bone->getParent())
		{

			if (!bone->get_locked(0))
			{
				//p[jnt_index + degree_index] = sk_virtual->getTranslation()(0) * height_scale;
				p[jnt_index + degree_index] = bone->getTranslation()[0];
				//x[jnt_index + degree_index + n] = weight_reg_trans * new_root(0) * height_scale;
				degree_index++;
			}
			if (!bone->get_locked(1))
			{
				//p[jnt_index + degree_index] = sk_virtual->getTranslation()(1) * height_scale;
				p[jnt_index + degree_index] = bone->getTranslation()[1];
				//x[jnt_index + degree_index + n] = 0.0f * new_root(1) * height_scale;
				degree_index++;
			}
			if (!bone->get_locked(2))
			{
				//p[jnt_index + degree_index] = sk_virtual->getTranslation()(2) * height_scale;
				p[jnt_index + degree_index] = bone->getTranslation()[2];
				//x[jnt_index + degree_index + n] = weight_reg_trans * new_root(2) * height_scale;
				degree_index++;
			}
		}
		if (!bone->get_locked(3))
		{

			p[jnt_index + degree_index] = bone->getRotate()(0);
			//x[jnt_index + degree_index + n] = weight_ro * weight_mul_ro * vir_bone->getRotate()(0);

			degree_index++;
		}
		if (!bone->get_locked(4))
		{

			p[jnt_index + degree_index] = bone->getRotate()(1);
			//x[jnt_index + degree_index + n] = weight_ro * vir_bone->getRotate()(1);

			degree_index++;
		}
		if (!bone->get_locked(5))
		{

			p[jnt_index + degree_index] = bone->getRotate()(2);
			//x[jnt_index + degree_index + n] = weight_ro * vir_bone->getRotate()(2);
			degree_index++;
		}
	}



	float joint_rot_smooth_weight = param.joint_rot_smooth_weight;
	float root_rot_smooth_weight = param.root_rot_smooth_weight;
	float root_tran_smooth_weight = param.root_tran_smooth_weight;
	float joint_2d_weight = param.joint_2d_weight;


	vector<float> weights;
	weights.resize(dof, 0.0);

	if (bone_set.find(sk_source->getRoot()->getName().get_str()) != bone_set.end()) //root is in
	{
		for (int j = 0; j < dof; j++)
		{

			if (j < 3)
				weights[j] = root_tran_smooth_weight;
			else if (j < 6)
				weights[j] = root_rot_smooth_weight;
			else
				weights[j] = joint_rot_smooth_weight;
		}
	}
	else
	{
		for (int j = 0; j < dof; j++)
		{
			weights[j] = joint_rot_smooth_weight;
		}
	}
	

	

	/*for (int j = 0; j < weights.size(); j++)
		cout << weights[j] << " ";
	cout << endl;*/

	//if (flag == 1)//refine root pos only
	//{
	//	joint_rot_smooth_weight = 10000.0;
	//	root_rot_smooth_weight = 10000.0;
	//}
	//else if (flag == 2) //refine root only
	//{
	//	joint_rot_smooth_weight = 10000.0;
	//}


	EVecXf x = EVecXf::Zero(2 * joints_2d.size() + dof);

	for (int j = 0; j < joints_2d.size(); j++)
	{
		x[2 * j + 0] = joint_2d_weight * joints_2d[j].x;
		x[2 * j + 1] = joint_2d_weight * joints_2d[j].y;
	}

	if (bone_set.find(sk_source->getRoot()->getName().get_str()) != bone_set.end()) //root is in
	{
		for (int j = 0; j < dof; j++)
		{

			//if (j < 3)
			//	x[2 * joints_2d.size() + j] = 5 * root_tran_smooth_weight * p[j] * scale; // mm to m
			//else if (j < 6)
			//	x[2 * joints_2d.size() + j] = 10 * root_rot_smooth_weight * p[j] * 180 / M_PI; //rad to deg
			//else
			//	x[2 * joints_2d.size() + j] = 5 * joint_rot_smooth_weight * p[j] * 180 / M_PI; //rad to deg

			if (j < 3)
				x[2 * joints_2d.size() + j] = 5 * weights[j] * p[j] * scale; // mm to m
			else if (j < 6)
				x[2 * joints_2d.size() + j] = 10 * weights[j] * p[j] * 180 / M_PI; //rad to deg
			else
				x[2 * joints_2d.size() + j] = 5 * weights[j] * p[j] * 180 / M_PI; //rad to deg
		}
	}
	else
	{
		for (int j = 0; j < dof; j++)
		{

			//if (j < 3)
			//	x[2 * joints_2d.size() + j] = 5 * root_tran_smooth_weight * p[j] * scale; // mm to m
			//else if (j < 6)
			//	x[2 * joints_2d.size() + j] = 10 * root_rot_smooth_weight * p[j] * 180 / M_PI; //rad to deg
			//else
			//	x[2 * joints_2d.size() + j] = 5 * joint_rot_smooth_weight * p[j] * 180 / M_PI; //rad to deg

			
			x[2 * joints_2d.size() + j] = 5 * weights[j] * p[j] * 180 / M_PI; //rad to deg
		}
	}

	


	float opts[LM_OPTS_SZ], info[LM_INFO_SZ];
	opts[0] = LM_INIT_MU; opts[1] = 1E-15; opts[2] = 1E-15; opts[3] = 1E-20; opts[4] = 1E-06;

	levmar_UserData ext_data;
	ext_data.sk_source = sk_source;
	ext_data.joints_names = joints_names;
	ext_data.joints_2d = joints_2d;
	ext_data.view = cur_view;
	ext_data.index_map = index_map;
	ext_data.joint_rot_smooth_weight = joint_rot_smooth_weight;
	ext_data.root_rot_smooth_weight = root_rot_smooth_weight;
	ext_data.root_tran_smooth_weight = root_tran_smooth_weight;
	ext_data.joint_2d_weight = joint_2d_weight;
	ext_data.scale = scale;
	ext_data.weights = weights;
	ext_data.bone_set = bone_set;

	auto start = std::chrono::high_resolution_clock::now();

	cout << "before: " << p.transpose() << endl;
	int ret = slevmar_der(disval_posesolver_v2, jacos_posesolver_v2, p.data(), x.data(), p.size(), x.size(), 1000, opts, info, nullptr, nullptr, &ext_data);
	cout << "after: " << p.transpose() << endl;

	for (int i = 0; i < LM_INFO_SZ; i++)
		cout << "info " << i << " : " << info[i] << endl;

	//update back to frame

	for (unsigned int ii = 0; ii < sk_source->getJointNum(); ii++)
	{
		ZJoint::Ptr jnt = sk_source->getJointByJID(ii);
		if (index_map.find(jnt->getName().get_str()) == index_map.end())
		{
			continue;
		}

		if (jnt->getDegrees() == 0)
		{
			continue;
		}

		unsigned int bone_index = index_map[jnt->getName()];
		unsigned int degree_index = 0;

		ZMath::ZVector3D rotate_vec = jnt->getRotate();
		ZMath::ZVector3D translate_vec = jnt->getTranslation();



		for (unsigned int jj = 0; jj < 3; jj++)
		{
			if (!jnt->get_locked(jj))
			{
				translate_vec(jj) = p[bone_index + degree_index];
				degree_index++;
			}
		}

		for (unsigned int jj = 3; jj < 6; jj++)
		{
			if (!jnt->get_locked(jj))
			{

				rotate_vec(jj - 3) = p[bone_index + degree_index];

				degree_index++;
			}
		}
		jnt->translate(translate_vec);
		jnt->rotate(rotate_vec);
	}
	extractMSkelPoseToMap(sk_source, pose_map);
}


void readMayaCamera(string file, int width, int height, vector<MayaCamera>& cameras)
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

	float film_width_inch = stof(tokens[4]);
	float film_height_inch = stof(tokens[5]);

	float film_width = stof(tokens[4]) * 25.4; // inch to mm
	float film_height = stof(tokens[5]) * 25.4;



	for (int i = startFrame; i <= endFrame; i++)
	{
		MayaCamera mcam;
		mcam.width_pixel = width;
		mcam.height_pixel = height;
		mcam.film_height_inch = film_height_inch;
		mcam.film_width_inch = film_width_inch;


		getline(is, str);
		boost::split(tokens, str, boost::is_any_of(","));
		EVec3f pos(stof(tokens[1]), stof(tokens[2]), stof(tokens[3]));

		mcam.pos_cm = pos;



		EVec3f angle(stof(tokens[4]), stof(tokens[5]), stof(tokens[6]));

		mcam.angle = angle;

		float focal_mm = stof(tokens[7]);
		mcam.focal_mm = focal_mm;

		cameras.push_back(mcam);
	}

	is.close();
}

GCameraPers maycam2gcam(const MayaCamera &mcam)
{
	float film_width = mcam.film_width_inch * 25.4; // inch to mm
	float film_height = mcam.film_height_inch * 25.4;

	float focal_pixel = mcam.focal_mm / film_width * mcam.width_pixel;

	EVec3f pos = mcam.pos_cm * 0.01;
	EMat3f rot = getRot3X3fromAngle(mcam.angle);

	EVec3f target = pos + EVec3f(rot(0, 2), rot(1, 2), rot(2, 2)) * (-1);
	EVec3f up = EVec3f(rot(0, 1), rot(1, 1), rot(2, 1));

	GCameraPers camera_real(mcam.width_pixel, mcam.height_pixel, 0.1, 10.0, GCameraIntrinsic(focal_pixel, focal_pixel, mcam.width_pixel / 2, mcam.height_pixel / 2), GCameraInfo(GVector3(pos.x(), pos.y(), pos.z()), GVector3(target.x(), target.y(), target.z()), GVector3(up.x(), up.y(), up.z())));	return camera_real;
}

EMat3f getRot3X3fromAngle(const EVec3f& pose)
{
	Eigen::Isometry3f transZ(Eigen::AngleAxisf(pose[2] / 180 * M_PI, Eigen::Vector3f(0, 0, 1)));
	Eigen::Isometry3f transY(Eigen::AngleAxisf(pose[1] / 180 * M_PI, Eigen::Vector3f(0, 1, 0)));
	Eigen::Isometry3f transX(Eigen::AngleAxisf(pose[0] / 180 * M_PI, Eigen::Vector3f(1, 0, 0)));

	EMat3f rot = transZ.linear() * transY.linear() * transX.linear();

	return rot;
}