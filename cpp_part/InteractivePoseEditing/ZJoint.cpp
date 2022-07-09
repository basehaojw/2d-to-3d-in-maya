#include "pch.h"
#include "ZJoint.h"

using namespace ZMath;

ZJoint::ZJoint(ZName name) : ZTransform(name)
{
    jid = -1;
    joint_orient = ZUnit3D();
    rotate_axis = ZUnit3D();
    JOM = ZIdentify4D();
    ROM = ZIdentify4D();

    for (unsigned ii = 0; ii < 9; ii++)
    {
        locked[ii] = false;
    }

}

ZID ZJoint::getJID()
{
    return jid;
}

void ZJoint::setJID(ZID id)
{
    jid = id;
}

//http://help.autodesk.com/view/MAYAUL/2018/CHS/?guid=__Nodes_index_html
void ZJoint::updateMatrix()
{
    ZTransform3D parent_world_matrix = ZIdentify4D();
    ZTransform3D ISM = ZIdentify4D();
    if (parent)
    {
        parent_world_matrix = parent->worldMatrix();
        ISM = parent->SMatrix().inverse();
    }
    
    local_matrix = TM * ISM * JOM * RM * ROM * SM;
    world_matrix = parent_world_matrix * local_matrix;
    dirty = false;

    for (auto& child : childs)
    {
        child->updateMatrix();
    }
}

ZMath::ZVector3D ZJoint::getJointOrient()
{
    return joint_orient;
}

ZMath::ZVector3D ZJoint::getRotateAxis()
{
    return rotate_axis;
}

const ZTransform3D& ZJoint::JOMatrix()
{
    return JOM;
}

const ZTransform3D& ZJoint::ROMatrix()
{
    return ROM;
}

const ZMath::ZTransform3D& ZJoint::ISMatrix()
{
    if (dirty) updateMatrix();
    if (parent)
    {
        ISM = parent->SMatrix().inverse();
    }
    else
    {
        ISM = ZIdentify4D();
    }
    return ISM;
}

void ZJoint::setJointOrient(const ZMath::ZVector3D& joint_orient)
{
    this->joint_orient = joint_orient;
    JOM = ZRotation(this->joint_orient, RotOrder::XYZ);
    dirty = true;
}

void ZJoint::setRotateAxis(const ZMath::ZVector3D& rotate_axis)
{
    this->rotate_axis = rotate_axis;
    ROM = ZRotation(this->rotate_axis, RotOrder::XYZ);
    dirty = true;
}

void ZJoint::captureInitialHistory()
{
    initial_history = captureHistory(name.get_str() + " initial history");
}

void ZJoint::restoreFromInitialHistory()
{
    restoreFromHistory(initial_history);
}

ZObjectType ZJoint::getObjectType()
{
    return ZObjectType::Joint;
}

ZHistoryRecord ZJoint::captureHistory(std::string history_info)
{
    ZHistoryRecord history_record;
    ZJointState joint_state = captureState();
    history_record.second.history_info = history_info;
    history_record.second.value = joint_state;
    return history_record;
}

void ZJoint::restoreFromHistory(const ZHistoryRecord& history_record)
{
    ZJointState joint_state = boost::any_cast<ZJointState>(history_record.second.value);

    restoreState(joint_state);
}

unsigned int ZJoint::getDegrees()
{
    unsigned int degrees = 0;
    for (unsigned int ii = 0; ii < 6; ii++)
    {
        if (!locked[ii])
        {
            degrees++;
        }
    }
    return degrees;
}

ZJointState ZJoint::captureState()
{
    ZJointState joint_state;
    joint_state.name = name;
    joint_state.jid = jid;
    joint_state.inherit_transform = inherit_parent_transform;
    joint_state.rot_order = rot_order;
    joint_state.translation = translation;
    joint_state.joint_orient = joint_orient;
    joint_state.rotation = rotation;
    joint_state.rotate_axis = rotate_axis;
    joint_state.scaling = scaling;
    joint_state.childs = childs;
    joint_state.parent = parent;

    return joint_state;
}

void ZJoint::restoreState(const ZJointState& joint_state)
{
    name = joint_state.name;
    inherit_parent_transform = joint_state.inherit_transform;
    rot_order = joint_state.rot_order;
    translate(joint_state.translation);
    setJointOrient(joint_state.joint_orient);
    rotate(joint_state.rotation);
    setRotateAxis(joint_state.rotate_axis);
    scale(joint_state.scaling);
    childs = joint_state.childs;
    parent = joint_state.parent;
    jid = joint_state.jid;

    updateMatrix();
}
