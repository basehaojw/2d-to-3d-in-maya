#pragma once
#include "ZJoint.h"
#include <nlohmann/json.hpp>

struct ZSkeletonState
{
	ZTransformState transform_state;
	ZJoint::Ptr root;
	ZJoint::PtrVec joint_list;
	std::map<ZName, ZJoint::Ptr> name_map;
};

class ZSkeleton : public ZTransform
{
public:
	ZDEFINE_PTR(ZSkeleton)
	ZDEFINE_VECPTR(ZSkeleton)

	ZSkeleton(ZName name);

	virtual bool loadSK(const ZPath& path);

	virtual bool saveSK(const ZPath& path);

	virtual bool loadMotion(const ZPath& path);

	virtual void setRoot(unsigned int id);

	virtual void addBone(ZJoint::Ptr joint);

	virtual void setBoneParent(unsigned int jid_bone, unsigned int jid_parent);

	virtual ZJoint::Ptr getRoot();

	virtual ZJoint::Ptr getJointByJID(unsigned int index);

	virtual size_t getJointNum();

	virtual ZJoint::Ptr getJointByName(ZName name);

	virtual ZObjectType getObjectType() override;

	virtual void setKey(int key) override;

	virtual bool setPoseByJson(std::string json_str);

	virtual bool setPoseByBoneStr(std::string json_str);

	virtual bool writeSPoseToFile(std::ofstream& fout,int key);

	virtual void updateMatrix() override;

	virtual std::string getPoseJsonStr();

	virtual std::string getUnrealJsonStr();

	virtual void appendFilterName(ZName);
	virtual void setOutputTransScale(ZScalar trans_scale);

	virtual void captureInitialHistory();
	virtual void restoreFromInitialHistory();

	virtual ZHistoryRecord captureHistory(std::string history_info) override;
	virtual void restoreFromHistory(const ZHistoryRecord& history_record) override;

	ZScalar getTransScale() const {
		return trans_scale;
	};

private:

	ZJoint::Ptr root;

	ZJoint::PtrVec joint_list;

	std::map<ZName, ZJoint::Ptr> name_map;

	ZScalar trans_scale = 1.0f;

	std::set<ZName> outputFilter_map;

	//capture initial record
	ZHistoryRecord initial_history;

	bool loadMSKel(const ZPath& path);

	bool loadSPS(const ZPath& path);

	bool saveMSKel(const ZPath& path);

	ZSkeletonState captureState();

	void restoreState(const ZSkeletonState& state);
};