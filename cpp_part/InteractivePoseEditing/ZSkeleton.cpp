#include "pch.h"
#include "ZSkeleton.h"

ZSkeleton::ZSkeleton(ZName name) : ZTransform(name)
{
}

bool ZSkeleton::loadSK(const ZPath& path)
{
	if (path.get_extensions() == ".mskel")
	{
		return loadMSKel(path);
	}
	return true;
}

bool ZSkeleton::saveSK(const ZPath& path)
{
	if (path.get_extensions() == ".mskel")
	{
		return saveMSKel(path);
	}
	return true;
}

bool ZSkeleton::loadMotion(const ZPath& path)
{
	if (path.get_extensions() == ".sps")
	{
		return loadSPS(path);
	}

	return true;
}

void ZSkeleton::setRoot(unsigned int id)
{
	root = joint_list[id];
}

void ZSkeleton::addBone(ZJoint::Ptr joint)
{
	joint->setJID(joint_list.size());
	joint_list.emplace_back(joint);
	name_map[joint->getName()] = joint;
}

void ZSkeleton::setBoneParent(unsigned int jid_bone, unsigned int jid_parent)
{
	joint_list[jid_bone]->setParent(joint_list[jid_parent]);
	joint_list[jid_parent]->appendChild(joint_list[jid_bone]);
}

ZJoint::Ptr ZSkeleton::getRoot()
{
	return root;
}

ZJoint::Ptr ZSkeleton::getJointByJID(unsigned int index)
{
	if (index > joint_list.size() + 1)
	{
		return nullptr;
	}
	return joint_list.at(index);
}

size_t ZSkeleton::getJointNum()
{
	return joint_list.size();
}

ZJoint::Ptr ZSkeleton::getJointByName(ZName name)
{
	if (name_map.find(name) != name_map.end())
	{
		return name_map[name];
	}
	else
	{
		return nullptr;
	}
}

ZObjectType ZSkeleton::getObjectType()
{
	return ZObjectType::Skeleton;
}

void ZSkeleton::setKey(int key)
{
	for (unsigned int ii = 0; ii < joint_list.size(); ii++)
	{
		joint_list[ii]->setKey(key);
	}
	updateMatrix();
}

bool ZSkeleton::setPoseByJson(std::string json_str)
{
	using json = nlohmann::json;

	json j;

	try
	{
		j = json::parse(json_str);
	}
	catch (json::parse_error& e)
	{
		LOG(ERROR) << "paser json failed";
		return false;
	}

	if (j.find("CharPoses") == j.end())
	{
		LOG(ERROR) << "can not find key CharPoses";
		return false;
	}

	json char_pose = j["CharPoses"];
	for (auto& pose : char_pose)
	{
		if (pose.find("Subject") == pose.end())
		{
			LOG(ERROR) << "can not find key Subject";
			return false;
		}

		if (pose.find("Bone") == pose.end())
		{
			LOG(ERROR) << "can not find key Bone";
			return false;
		}

		if (pose["Subject"] == getName().get_str())
		{
			std::vector<std::string> joint_lists;
			std::string body_data = pose["Bone"].get<std::string>();
			boost::split(joint_lists, body_data, boost::is_any_of(";"), boost::token_compress_on);
			for (std::string joint_data : joint_lists)
			{
				std::vector<std::string> tr_data;
				boost::split(tr_data, joint_data, boost::is_any_of("\t "), boost::token_compress_on);

				ZJoint::Ptr joint = getJointByName(tr_data[0]);
				if (joint)
				{
					for (unsigned int ii = 1; ii < tr_data.size(); ii++)
					{
						std::vector<std::string> s_data;
						boost::split(s_data, tr_data[ii], boost::is_any_of(":"), boost::token_compress_on);
						ZMath::ZVector3D trans = joint->getTranslation();
						ZMath::ZVector3D rot = joint->getRotate();

						if (s_data[0] == "tx")
						{
							trans(0) = std::stof(s_data[1]);
						}
						if (s_data[0] == "ty")
						{
							trans(1) = std::stof(s_data[1]);
						}
						if (s_data[0] == "tz")
						{
							trans(2) = std::stof(s_data[1]);
						}
						if (s_data[0] == "rx")
						{
							rot(0) = ZMath::degree2Radian(std::stof(s_data[1]));
						}
						if (s_data[0] == "ry")
						{
							rot(1) = ZMath::degree2Radian(std::stof(s_data[1]));
						}
						if (s_data[0] == "rz")
						{
							rot(2) = ZMath::degree2Radian(std::stof(s_data[1]));
						}

						joint->translate(trans);
						joint->rotate(rot);
					}
				}

			}
			return true;
		}
	}

	return false;


}


bool ZSkeleton::setPoseByBoneStr(std::string bone_str)
{
	std::vector<std::string> joint_lists;
	std::string body_data = bone_str;
	boost::split(joint_lists, body_data, boost::is_any_of(";"), boost::token_compress_on);
	for (std::string joint_data : joint_lists)
	{
		std::vector<std::string> tr_data;
		boost::split(tr_data, joint_data, boost::is_any_of("\t "), boost::token_compress_on);

		ZJoint::Ptr joint = getJointByName(tr_data[0]);
		if (joint)
		{
			for (unsigned int ii = 1; ii < tr_data.size(); ii++)
			{
				std::vector<std::string> s_data;
				boost::split(s_data, tr_data[ii], boost::is_any_of(":"), boost::token_compress_on);
				ZMath::ZVector3D trans = joint->getTranslation();
				ZMath::ZVector3D rot = joint->getRotate();

				if (s_data[0] == "tx")
				{
					trans(0) = std::stof(s_data[1]) * trans_scale;
				}
				if (s_data[0] == "ty")
				{
					trans(1) = std::stof(s_data[1]) * trans_scale;
				}
				if (s_data[0] == "tz")
				{
					trans(2) = std::stof(s_data[1]) * trans_scale;
				}
				if (s_data[0] == "rx")
				{
					rot(0) = ZMath::degree2Radian(std::stof(s_data[1]));
				}
				if (s_data[0] == "ry")
				{
					rot(1) = ZMath::degree2Radian(std::stof(s_data[1]));
				}
				if (s_data[0] == "rz")
				{
					rot(2) = ZMath::degree2Radian(std::stof(s_data[1]));
				}

				joint->translate(trans);
				joint->rotate(rot);
			}
		}
	}
	return true;
}

bool ZSkeleton::writeSPoseToFile(std::ofstream& fout, int key)
{
	fout << key << std::endl;
	for (unsigned int ii = 0; ii < getJointNum(); ii++)
	{
		

		ZJoint::Ptr jnt = getJointByJID(ii);
		if (outputFilter_map.find(jnt->getName()) != outputFilter_map.end()) continue;
		fout << jnt->getName().get_str();

		/*if (!jnt->get_locked(0))
		{
			fout << " " << "tx:" << jnt->getTranslation()(0);
		}
		if (!jnt->get_locked(1))
		{
			fout << " " << "ty:" << jnt->getTranslation()(1);
		}
		if (!jnt->get_locked(2))
		{
			fout << " " << "tz:" << jnt->getTranslation()(2);
		}
		if (!jnt->get_locked(3))
		{
			fout << " " << "rx:" << ZMath::radian2Degree(jnt->getRotate()(0));
		}
		if (!jnt->get_locked(4))
		{
			fout << " " << "ry:" << ZMath::radian2Degree(jnt->getRotate()(1));
		}
		if (!jnt->get_locked(5))
		{
			fout << " " << "rz:" << ZMath::radian2Degree(jnt->getRotate()(2));
		}*/

		fout << " " << "tx:" << jnt->getTranslation()(0) * trans_scale;
		fout << " " << "ty:" << jnt->getTranslation()(1) * trans_scale;
		fout << " " << "tz:" << jnt->getTranslation()(2) * trans_scale;
		fout << " " << "rx:" << ZMath::radian2Degree(jnt->getRotate()(0));
		fout << " " << "ry:" << ZMath::radian2Degree(jnt->getRotate()(1));
		fout << " " << "rz:" << ZMath::radian2Degree(jnt->getRotate()(2));

		fout << std::endl;
	}

	return true;
}

void ZSkeleton::updateMatrix()
{
	ZMath::ZTransform3D parent_matrix = ZMath::ZIdentify4D();
	if (parent)
	{
		parent_matrix = parent->worldMatrix();
	}
	local_matrix = TM * RM * SM;
	world_matrix = parent_matrix * local_matrix;
	dirty = false;

	for (auto& child : childs)
	{
		child->updateMatrix();
	}

	root->updateMatrix();
}

std::string ZSkeleton::getPoseJsonStr()
{
	using json = nlohmann::json;
	json j;

	std::string bone;

	json char_pose;
	json bone_subject;
	bone_subject["Subject"] = getName().get_str();
	for (unsigned int ii = 0; ii < getJointNum(); ii++)
	{
		ZJoint::Ptr jnt = joint_list[ii];
		if (outputFilter_map.find(jnt->getName()) != outputFilter_map.end()) continue;
		if (jnt->getDegrees() == 0) continue;
		bone.append(jnt->getName().get_str());
		bone.append(" ");
		if (!jnt->get_locked(0))
		{
			bone.append("tx:");
			bone.append(std::to_string(jnt->getTranslation()(0) * trans_scale));
			bone.append(" ");
		}
		if (!jnt->get_locked(1))
		{
			bone.append("ty:");
			bone.append(std::to_string(jnt->getTranslation()(1) * trans_scale));
			bone.append(" ");
		}
		if (!jnt->get_locked(2))
		{
			bone.append("tz:");
			bone.append(std::to_string(jnt->getTranslation()(2) * trans_scale));
			bone.append(" ");
		}
		if (!jnt->get_locked(3))
		{
			bone.append("rx:");
			bone.append(std::to_string(ZMath::radian2Degree(jnt->getRotate()(0))));
			bone.append(" ");
		}
		if (!jnt->get_locked(4))
		{
			bone.append("ry:");
			bone.append(std::to_string(ZMath::radian2Degree(jnt->getRotate()(1))));
			bone.append(" ");
		}
		if (!jnt->get_locked(5))
		{
			bone.append("rz:");
			bone.append(std::to_string(ZMath::radian2Degree(jnt->getRotate()(2))));
			bone.append(" ");
		}
		bone.append(";");
	}
	bone_subject["Bone"] = bone;

	char_pose.emplace_back(bone_subject);

	j["CharPoses"] = char_pose;

	return j.dump();

}

std::string ZSkeleton::getUnrealJsonStr()
{
	using json = nlohmann::json;
	json j;
	for (unsigned int ii = 0; ii < getJointNum(); ii++)
	{
		ZJoint::Ptr jnt = joint_list[ii];
		if (outputFilter_map.find(jnt->getName()) != outputFilter_map.end()) continue;
		json bonepose;
		json boneTrans;
		json boneRotate;

		if (!jnt->getParent())
		{
			boneTrans.emplace_back(jnt->getTranslation()(0) * trans_scale);
			boneTrans.emplace_back(-jnt->getTranslation()(1) * trans_scale);
			boneTrans.emplace_back(jnt->getTranslation()(2) * trans_scale);
			bonepose["Translate"] = boneTrans;
		}

		ZMath::ZMatrix3D localTransform = jnt->localMatrix().matrix().block(0, 0, 3, 3);
		localTransform(0, 1) = -localTransform(0, 1);
		localTransform(1, 0) = -localTransform(1, 0);
		localTransform(1, 2) = -localTransform(1, 2);
		localTransform(2, 1) = -localTransform(2, 1);
		Eigen::Quaternion<ZScalar> q(localTransform);
		boneRotate.emplace_back(q.x());
		boneRotate.emplace_back(q.y());
		boneRotate.emplace_back(q.z());
		boneRotate.emplace_back(q.w());

		
		bonepose["Rotate"] = boneRotate;

		std::string boneName = jnt->getName().get_str();
		std::string replaceStr = "FBXASC032";
		std::string newStr = "-";
		size_t start_pos = boneName.find(replaceStr);
		while (start_pos != std::string::npos) {
			boneName.replace(start_pos, replaceStr.length(), newStr);
			start_pos = boneName.find(replaceStr);
		}

		j[boneName] = bonepose;

	}
	json j_out;
	j_out["skeleton_frame"] = j;
	return j_out.dump();
}

void ZSkeleton::appendFilterName(ZName appendSet)
{
	outputFilter_map.insert(appendSet);
}

void ZSkeleton::setOutputTransScale(ZScalar trans_scale)
{
	this->trans_scale = trans_scale;
}

void ZSkeleton::captureInitialHistory()
{
	initial_history = captureHistory("skeleton initial history");
}

void ZSkeleton::restoreFromInitialHistory()
{
	restoreFromHistory(initial_history);
}

ZHistoryRecord ZSkeleton::captureHistory(std::string history_info)
{
	ZHistoryRecord history_record;                                                                                                                                                                                                                                                                                                                                                                  
	ZSkeletonState skeleton_state = captureState();
	history_record.second.history_info = history_info;
	history_record.second.value = skeleton_state;
	return history_record;
}

void ZSkeleton::restoreFromHistory(const ZHistoryRecord& history_record)
{
	ZSkeletonState skeleton_state = boost::any_cast<ZSkeletonState>(history_record.second.value);
	restoreState(skeleton_state);
}

bool ZSkeleton::loadMSKel(const ZPath& path)
{
	std::ifstream fin(path.get_str());
	if (!fin.is_open())
	{
		LOG(ERROR) << path.get_str() << "does not exist";
		return false;
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
						LOG(ERROR) << path.get_str() << "parsing MSKel error at line:" << line_num;
						return false;
					}
					ZJoint::Ptr joint = std::make_shared<ZJoint>(split_joints[0]);
					joint->setJID(ii);
					joint_list.emplace_back(joint);
					name_map[joint->getName()] = joint;

					std::string rot_order = split_joints[1];
					boost::trim(rot_order);
					if (rot_order == "xyz")
					{
						joint->setRotOrder(ZMath::RotOrder::XYZ);
					}
					else if (rot_order == "xzy")
					{
						joint->setRotOrder(ZMath::RotOrder::XZY);
					}
					else if (rot_order == "yxz")
					{
						joint->setRotOrder(ZMath::RotOrder::YXZ);
					}
					else if (rot_order == "yzx")
					{
						joint->setRotOrder(ZMath::RotOrder::YZX);
					}
					else if (rot_order == "zyx")
					{
						joint->setRotOrder(ZMath::RotOrder::ZYX);
					}
					else if (rot_order == "zxy")
					{
						joint->setRotOrder(ZMath::RotOrder::ZXY);
					}
					
					joint->translate(ZMath::ZVector3D{ static_cast<ZScalar>(std::stod(split_joints[2])),
						static_cast<ZScalar>(std::stod(split_joints[3])),
						static_cast<ZScalar>(std::stod(split_joints[4])) });
					joint->setJointOrient(ZMath::ZVector3D{ ZMath::degree2Radian(static_cast<ZScalar>(std::stod(split_joints[5]))),
						ZMath::degree2Radian(static_cast<ZScalar>(std::stod(split_joints[6]))),
						ZMath::degree2Radian(static_cast<ZScalar>(std::stod(split_joints[7]))) });
					joint->rotate(ZMath::ZVector3D{ ZMath::degree2Radian(static_cast<ZScalar>(std::stod(split_joints[8]))),
						ZMath::degree2Radian(static_cast<ZScalar>(std::stod(split_joints[9]))),
						ZMath::degree2Radian(static_cast<ZScalar>(std::stod(split_joints[10]))) });
					joint->setRotateAxis(ZMath::ZVector3D{ ZMath::degree2Radian(static_cast<ZScalar>(std::stod(split_joints[11]))),
						ZMath::degree2Radian(static_cast<ZScalar>(std::stod(split_joints[12]))),
						ZMath::degree2Radian(static_cast<ZScalar>(std::stod(split_joints[13]))) });

					//todo : if scale

					//locked info:
					int locked_index = 0;
					for (unsigned int jj = 0; jj < 3; jj++)
					{
						bool is_locked = std::stoi(split_joints[14 + jj]);
						if (is_locked == 0)
						{
							joint->set_locked(jj, false);
							switch (jj)
							{
							case 0 :
								if (split_joints.size() > 21)
								{
									joint->set_x_trans_limit(std::stof(split_joints[20 + locked_index]), std::stof(split_joints[20 + locked_index + 1]));
									locked_index += 2;
								}
								break;
							case 1:
								if (split_joints.size() > 21)
								{
									joint->set_y_trans_limit(std::stof(split_joints[20 + locked_index]), std::stof(split_joints[20 + locked_index + 1]));
									locked_index += 2;
								}
								break;
							case 2:
								if (split_joints.size() > 21)
								{
									joint->set_z_trans_limit(std::stof(split_joints[20 + locked_index]), std::stof(split_joints[20 + locked_index + 1]));
									locked_index += 2;
								}
								break;
							default:
								break;
							}
						}
						else
						{
							joint->set_locked(jj, true);
						}
					}
					for (unsigned int kk = 0; kk < 3; kk++)
					{
						bool is_locked = std::stoi(split_joints[17 + kk]);
						if (is_locked == 0)
						{
							joint->set_locked(kk + 3, false);
							switch (kk)
							{
							case 0:
								if (split_joints.size() > 21)
								{
									joint->set_x_rotate_limit(std::stof(split_joints[20 + locked_index]), std::stof(split_joints[20 + locked_index + 1]));
									locked_index += 2;
								}
								break;
							case 1:
								if (split_joints.size() > 21)
								{
									joint->set_y_rotate_limit(std::stof(split_joints[20 + locked_index]), std::stof(split_joints[20 + locked_index + 1]));
									locked_index += 2;
								}
								break;
							case 2:
								if (split_joints.size() > 21)
								{
									joint->set_z_rotate_limit(std::stof(split_joints[20 + locked_index]), std::stof(split_joints[20 + locked_index + 1]));
									locked_index += 2;
								}
								break;
							default:
								break;
							}
						}
						else
						{
							joint->set_locked(kk + 3, true);
						}
					}



				}
				else
				{
					LOG(ERROR) << path.get_str() << " MSKel num not correct";
					return false;
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
					LOG(ERROR) << path.get_str() << "parsing MSKel error at line:" << line_num;
					return false;
				}
				if (split_hierachy[0] == "marker")
				{
					break;
				}
				ZName bone_name1 = split_hierachy[0];
				ZName bone_name2 = split_hierachy[1];

				if (name_map.find(bone_name1) == name_map.end())
				{
					LOG(ERROR) << path.get_str() << "can not find name :" << bone_name1 << " at line:" << line_num;
				}
				if (name_map.find(bone_name2) == name_map.end())
				{
					LOG(ERROR) << path.get_str() << "can not find name :" << bone_name2 << " at line:" << line_num;
				}

				name_map[bone_name1]->setParent(name_map[bone_name2]);
				name_map[bone_name2]->appendChild(name_map[bone_name1]);
			}
		}

		for (unsigned int ii = 0; ii < joint_list.size(); ii++)
		{
			if (!joint_list[ii]->getParent())
			{
				root = joint_list[ii];
				break;
			}
		}

		//root->setParent(std::dynamic_pointer_cast<ZITransform3D>(shared_from_this()));
	}

	captureInitialHistory();

	for (unsigned int ii = 0; ii < joint_list.size(); ii++)
	{
		joint_list[ii]->captureInitialHistory();
	}


	return true;
}

bool ZSkeleton::loadSPS(const ZPath& path)
{
	std::ifstream fin(path.get_str());
	if (!fin.is_open())
	{
		LOG(ERROR) << path.get_str() << "does not exist";
		return false;
	}
	//if file exist
	for (unsigned int ii = 0; ii < joint_list.size(); ii++)
	{
		joint_list[ii]->clearALLKeyFrame();
	}

	auto is_digit = [](std::string s)
	{
		return std::all_of(s.begin(), s.end(), ::isdigit);
	};

	std::string cur_line;
	//bool is_first = true;
	//int first_frame_num = 0;
	int cur_frame_num = 0;

	std::set<ZName> error_joint;

	while (std::getline(fin, cur_line))
	{
		boost::trim(cur_line);
		if (cur_line.empty() || cur_line[0] == '#')
		{
			continue;
		}
		std::vector<std::string> split_character;
		boost::split(split_character, cur_line, boost::is_any_of("\t "), boost::token_compress_on);
		if (split_character.size() == 1 && !is_digit(cur_line))
		{
			continue;
		}
		if (split_character.size() == 1 && is_digit(cur_line))//fist_pose begin
		{
			cur_frame_num = stoi(split_character[0]);
			//if (is_first)
			//{
			//	first_frame_num = cur_frame_num;
			//	is_first = true;
			//}
			continue;
		}

		if (split_character.size() > 1)
		{
			ZName joint_name = split_character[0];
			ZJoint::Ptr joint = name_map[joint_name];
			if (!joint)
			{
				if (error_joint.find(joint_name) == error_joint.end())
				{
					LOG(WARNING) << name << " can not find joint:" << joint_name;
				}
				else
				{
					error_joint.insert(joint_name);
				}
				continue;
			}
			else
			{
				for (unsigned int ii = 1; ii < split_character.size(); ii++)
				{
					std::vector<std::string> split_value;
					boost::split(split_value, split_character[ii], boost::is_any_of(":"), boost::token_compress_on);
					if (split_value[0] == "tx")
					{
						ZAnimationFrame frame;
						frame.value = std::stof(split_value[1]);
						joint->setKeyFrame(cur_frame_num, frame, 0);
					}
					if (split_value[0] == "ty")
					{
						ZAnimationFrame frame;
						frame.value = std::stof(split_value[1]);
						joint->setKeyFrame(cur_frame_num, frame, 1);
					}
					if (split_value[0] == "tz")
					{
						ZAnimationFrame frame;
						frame.value = std::stof(split_value[1]);
						joint->setKeyFrame(cur_frame_num, frame, 2);
					}
					if (split_value[0] == "rx")
					{
						ZAnimationFrame frame;
						frame.value = ZMath::degree2Radian(std::stof(split_value[1]));
						joint->setKeyFrame(cur_frame_num, frame, 3);
					}
					if (split_value[0] == "ry")
					{
						ZAnimationFrame frame;
						frame.value = ZMath::degree2Radian(std::stof(split_value[1]));
						joint->setKeyFrame(cur_frame_num, frame, 4);
					}
					if (split_value[0] == "rz")
					{
						ZAnimationFrame frame;
						frame.value = ZMath::degree2Radian(std::stof(split_value[1]));
						joint->setKeyFrame(cur_frame_num, frame, 5);
					}
					if (split_value[0] == "sx")
					{
						ZAnimationFrame frame;
						frame.value = std::stof(split_value[1]);
						joint->setKeyFrame(cur_frame_num, frame, 6);
					}
					if (split_value[0] == "sy")
					{
						ZAnimationFrame frame;
						frame.value = std::stof(split_value[1]);
						joint->setKeyFrame(cur_frame_num, frame, 7);
					}
					if (split_value[0] == "sx")
					{
						ZAnimationFrame frame;
						frame.value = std::stof(split_value[1]);
						joint->setKeyFrame(cur_frame_num, frame, 8);
					}
				}
			}
		}
	}

	return true;
}

bool ZSkeleton::saveMSKel(const ZPath& path)
{
	using namespace ZMath;
	std::ofstream fout(path.get_str(), 'w');
	if (!fout.is_open()) {
		LOG(ERROR) << "error open path :" << path.get_str();
		return false;
	}
	fout << "mskel" << std::endl;
	fout << "joint " << joint_list.size() << std::endl;
	for (unsigned int ii = 0; ii < getJointNum(); ii++)
	{
		ZJoint::Ptr jnt = joint_list[ii];
		fout << jnt->getName().get_str();
		fout << " ";
		switch (rot_order)
		{
		case RotOrder::XYZ:
			fout << "xyz";
			break;
		case RotOrder::XZY:
			fout << "xzy";
			break;
		case RotOrder::YXZ:
			fout << "yxz";
			break;
		case RotOrder::YZX:
			fout << "yzx";
			break;
		case RotOrder::ZYX:
			fout << "zyx";
			break;
		case RotOrder::ZXY:
			fout << "zxy";
			break;
		default:
			fout << "xyz";
			break;
		}

		ZVector3D trans = jnt->getTranslation();
		fout << " " << trans(0) << " " << trans(1) << " " << trans(2);
		ZVector3D jo = jnt->getJointOrient();
		fout << " " << radian2Degree(jo(0)) << " " << radian2Degree(jo(1)) << " " << radian2Degree(jo(2));
		ZVector3D r = jnt->getRotate();
		fout << " " << radian2Degree(r(0)) << " " << radian2Degree(r(1)) << " " << radian2Degree(r(2));
		ZVector3D ro = jnt->getRotateAxis();
		fout << " " << radian2Degree(ro(0)) << " " << radian2Degree(ro(1)) << " " << radian2Degree(ro(2));

		for (unsigned int ii = 0; ii < 6; ii++)
		{
			if (jnt->get_locked(ii))
			{
				fout << " " << 1;
			}
			else
			{
				fout << " " << 0;
			}
		}
		if (!jnt->get_locked(0))
		{
			fout << " " << jnt->get_x_trans_limit().min_limit << " " << jnt->get_x_trans_limit().max_limit;
		}
		if (!jnt->get_locked(1))
		{
			fout << " " << jnt->get_y_trans_limit().min_limit << " " << jnt->get_y_trans_limit().max_limit;
		}
		if (!jnt->get_locked(2))
		{
			fout << " " << jnt->get_z_trans_limit().min_limit << " " << jnt->get_z_trans_limit().max_limit;
		}
		if (!jnt->get_locked(3))
		{
			fout << " " << -360 << " " << 360;
		}
		if (!jnt->get_locked(4))
		{
			fout << " " << -360 << " " << 360;
		}
		if (!jnt->get_locked(5))
		{
			fout << " " << -360 << " " << 360;
		}
		/*if (!jnt->get_locked(0))
		{
			fout << " " << jnt->get_x_trans_limit().min_limit << " " << jnt->get_x_trans_limit().max_limit;
		}
		if (!jnt->get_locked(1))
		{
			fout << " " << jnt->get_y_trans_limit().min_limit << " " << jnt->get_y_trans_limit().max_limit;
		}
		if (!jnt->get_locked(2))
		{
			fout << " " << jnt->get_z_trans_limit().min_limit << " " << jnt->get_z_trans_limit().max_limit;
		}
		if (!jnt->get_locked(3))
		{
			fout << " " << jnt->get_x_rotate_limit().min_limit << " " << jnt->get_x_rotate_limit().max_limit;
		}
		if (!jnt->get_locked(4))
		{
			fout << " " << jnt->get_y_rotate_limit().min_limit << " " << jnt->get_y_rotate_limit().max_limit;
		}
		if (!jnt->get_locked(5))
		{
			fout << " " << jnt->get_z_rotate_limit().min_limit << " " << jnt->get_z_rotate_limit().max_limit;
		}*/
		fout << std::endl;
	}

	fout << "hierarchy" << std::endl;

	for (unsigned int ii = 0; ii < getJointNum(); ii++)
	{
		ZJoint::Ptr jnt = joint_list[ii];
		for (auto child : jnt->getChilds())
		{
			fout << child->getName().get_str() << " " << jnt->getName().get_str() << std::endl;
		}
	}

	fout.close();
}

ZSkeletonState ZSkeleton::captureState()
{
	ZSkeletonState state;
	state.transform_state.name = name;
	state.transform_state.inherit_transform = inherit_parent_transform;
	state.transform_state.rot_order = rot_order;
	state.transform_state.rotation = rotation;
	state.transform_state.scaling = scaling;
	state.transform_state.childs = childs;
	state.transform_state.parent = parent;

	state.joint_list = joint_list;
	state.name_map = name_map;
	state.root = root;

	return state;
}

void ZSkeleton::restoreState(const ZSkeletonState& state)
{
	name = state.transform_state.name;
	inherit_parent_transform = state.transform_state.inherit_transform;
	rot_order = state.transform_state.rot_order;
	translate(state.transform_state.translation);
	rotate(state.transform_state.rotation);
	scale(state.transform_state.scaling);
	childs = state.transform_state.childs;
	parent = state.transform_state.parent;

	joint_list = state.joint_list;
	name_map = state.name_map;
	root = state.root;

	updateMatrix();
}
