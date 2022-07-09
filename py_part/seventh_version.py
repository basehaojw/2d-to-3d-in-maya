import re
import os
import sys
from operator import add

from PySide2 import QtWidgets, QtGui, QtCore
import maya.OpenMayaUI as omui
import shiboken2
import maya.cmds as cmds
import pymel.core as pm
import math

sys.path.append("C:/Users/admin/haojw/GitLab/maya/PLE-22602/target_tool/")
from re_package import re_packaging as c_api

mm_cm_m = 1


def getMayaMainWindow():
    windowPtr = omui.MQtUtil.mainWindow()
    return shiboken2.wrapInstance(long(windowPtr), QtWidgets.QWidget)


class MainWindow(QtWidgets.QWidget):
    def __init__(self):

        super(MainWindow, self).__init__()
        parent = getMayaMainWindow()
        self.setParent(parent)
        self.setWindowFlags(QtCore.Qt.Window)
        self.setWindowTitle('Projection&Optimization')
        self.setMinimumSize(100, 120)
        self.main_lay = QtWidgets.QHBoxLayout()

        top_lay = QtWidgets.QHBoxLayout()

        self.project_btn = QtWidgets.QPushButton('Project 2d joints')
        self.project_btn.setMinimumSize(130, 40)
        top_lay.addWidget(self.project_btn)

        optimize_btn = QtWidgets.QPushButton('Optimize')
        optimize_btn.setMinimumSize(130, 40)

        undo_btn = QtWidgets.QPushButton('Undo')
        undo_btn.setMinimumSize(130, 40)
        self.main_lay.addLayout(top_lay)
        self.main_lay.addWidget(optimize_btn)
        # self.main_lay.addWidget(undo_btn)

        self.setLayout(self.main_lay)

        self.project_btn.clicked.connect(self.generate_2d_joints)
        optimize_btn.clicked.connect(self.optimize)
        undo_btn.clicked.connect(self.undo)

        self.maya_position = {}
        self.cam_info = []
        # self.pose_map = {}
        self.root = get_post_map_test()[0]
        # self.pose_map.update(self.root)
        # self.pose_map.update(branch)

    def generate_2d_joints(self):
        joints_dict = get_skl_data()
        sphare_item = []
        for i in joints_dict:
            for j in joints_dict:
                if j in joints_dict[i]:
                    print j
                    sphare_item.append(j)
        for i in sphare_item:
            joints_dict.pop(i)

        joints_2d_percentage_data, self.cam_info = get_2d_joint_data(joints_dict)
        print "percentage dict: ", joints_2d_percentage_data
        top_right, top_left, bottom_left = get_coordinates()[:3]
        self.maya_position = calculate_3d_pos(top_right, top_left, bottom_left, joints_2d_percentage_data)
        print "maya_position: ", self.maya_position
        image_depth = get_coordinates()[-2]
        cam_angle = self.cam_info[0]
        create_2d_joints(cam_angle, joints_dict, self.maya_position, image_depth)

        cmds.select(cl=True)

        if not cmds.ls('joints_2d', type="transform"):
            cmds.group(em=True, name="joints_2d")
        grp = cmds.ls('joints_2d', type="transform")
        if cmds.listRelatives(grp):
            for i in cmds.listRelatives(grp, ad=1):
                cmds.delete(i)
        for i in joints_dict.keys():
            cmds.parent(i + '_2d', grp)

    def optimize(self):
        print 'optimizing...'
        current_2d_jnts_trans = get_current_2d_jnts_trans()
        moved_jnts = {}
        for i in current_2d_jnts_trans:
            for j in self.maya_position:
                if re.match(j+"_2d", i):
                    compare_result = compare_xyz(list(current_2d_jnts_trans[i]), list(self.maya_position[j]))
                    if not compare_result:
                        # print i, "position has changed from {} to {}".format(list(self.maya_position[j]),
                        #                                                      current_2d_jnts_trans[i])
                        moved_jnts[i] = current_2d_jnts_trans[i]
        if not moved_jnts:
            print "nothing need to optimized!!!"
            return
        print "moved_jnts: ", moved_jnts
        new_2d_projected_pos = c_api.project_joint(mm_cm_m, moved_jnts.values(), moved_jnts.keys(), self.cam_info)
        print "new_2d_projected_pos: ", new_2d_projected_pos
        result_2d_pos = {}
        for i in self.maya_position:
            for j in new_2d_projected_pos:
                if re.match(i+'_2d', j):
                    result_2d_pos[i] = [new_2d_projected_pos[i+'_2d_x'], new_2d_projected_pos[i+'_2d_y']]
        print "result_2d_pos: ", result_2d_pos

        pose_map = get_post_map_test()[1]
        print "pose_map: ", pose_map

        cur_mskel_bones = get_cur_mskel_bones(self.root.keys()[0])
        print "cur_mskel_bones: ", cur_mskel_bones
        result = c_api.optimize(pose_map, mm_cm_m, result_2d_pos.values(),
                                result_2d_pos.keys(), cur_mskel_bones, self.cam_info)
        print "Final result: ", result

        for i in result:
            if i in cur_mskel_bones:
                if len(result[i]) > 3:
                    print "before    joint: ", i, "  value: ", pose_map[i]
                    print "optimized joint: ", i, "  value: ", result[i]
                    # pm.PyNode(i).translate.set(*result[i][:3])
                    result[i] = [math.degrees(j) for j in result[i]]
                    print "root- test--", result[i]
                    pm.PyNode(i).rotate.set(*result[i][-3:])
                else:
                    print "before    joint: ", i, "  value: ", pose_map[i][-3:]
                    print "optimized joint: ", i, "  value: ", result[i][-3:]
                    result[i] = [math.degrees(j) for j in result[i]]
                    print "other- test--", result[i]
                    pm.PyNode(i).rotate.set(*result[i][-3:])
        # self.generate_2d_joints()

    def undo(self):
        pass


def get_cur_mskel_bones(root_bones):
    mskel_jnts = pm.ls(sl=1)
    if not mskel_jnts:
        cmds.confirmDialog(title='Confirm', message='Please select joints need to be optimized ',  cancelButton='OK',
                           dismissString='OK')
        return
    for i in mskel_jnts:
        if root_bones not in i.longName():
            cmds.confirmDialog(title='Warning', message='Select joints in Wrong place, please check!',
                               cancelButton='OK', dismissString='OK')
            return
    result = [i.shortName() for i in mskel_jnts]

    return result


def get_current_2d_jnts_trans():
    joints_2d_grp = pm.ls("joints_2d", type="transform")
    if not joints_2d_grp:
        print "Please generate 2d joints first"
        return
    joints_2d_grp = joints_2d_grp[0]
    joints_data = {}
    for i in joints_2d_grp.getChildren():
        joints_data[i.shortName()] = {}

    def get_children(iter_dict):
        if not iter_dict:
            return
        for root in iter_dict:
            if cmds.listRelatives(root):
                iter_dict[root] = {i: {} for i in cmds.listRelatives(root)}
                get_children(iter_dict[root])

    get_children(joints_data)

    current_joints_translation = get_skl_translation(joints_data)

    return current_joints_translation


def get_post_map_test():
    root_jnt = "Chest_M"
    root_jnt = cmds.ls("Chest_M", type="joint")
    if len(root_jnt) > 1:
        raise Exception("More than one root joint found, please check !!!")
    root_jnt = root_jnt[0]
    children = cmds.listRelatives(root_jnt, ad=1)
    p_trans = list(pm.PyNode(root_jnt).getTranslation())
    p_rot = list(pm.PyNode(root_jnt).getRotation())
    p_rot = [math.radians(i) for i in p_rot]
    root = {root_jnt: reduce(add, (p_trans, p_rot))}

    branch = {i: [math.radians(j) for j in list(pm.PyNode(i).getRotation())] for i in children}
    result = dict()
    result.update(root)
    result.update(branch)

    return root, result


def get_pose_map_info(jnt, root=False):
    if root:
        result = reduce(add, (list(pm.PyNode(jnt).getTranslation(space="world")), list(pm.PyNode(jnt).getRotation())))
    else:
        result = list(pm.PyNode(jnt).getRotation())

    return result


def get_skl_data():
    joints_data = {}
    root_system = cmds.ls(sl=1)
    for i in root_system:
        joints_data[i] = {}

    def get_children(iter_dict):
        if not iter_dict:
            return
        for root in iter_dict:
            if cmds.listRelatives(root):
                iter_dict[root] = {i: {} for i in cmds.listRelatives(root)}
                get_children(iter_dict[root])

    get_children(joints_data)
    return joints_data


def create_2d_joints(cam_rotation, skl_dict, position_dict, image_depth, skl_parent=None):
    rot = cam_rotation
    for k, v in skl_dict.items():
        if "_2d" not in k:
            joint_name = k + "_2d"
        else:
            joint_name = k
        if cmds.ls(joint_name):
            for i in cmds.ls(joint_name):
                cmds.delete(i)

        position = position_dict[k]
        if skl_parent:
            cmds.select(skl_parent, add=0)
        else:
            cmds.select(cl=1)
        #0.003 * image_depth
        cmds.joint(name=joint_name, rad=0.002 * image_depth, p=position, o=rot)
        # cmds.setAttr('{}.translateZ'.format(joint_name), l=1)
        # cmds.setAttr('{}.rotateZ'.format(joint_name), l=1)
        cmds.setAttr('{}.overrideEnabled'.format(joint_name), 1)
        cmds.setAttr(".ovc", 6)
        rot = [0, 0, 0]
        if v:
            parent_jnt = joint_name
            create_2d_joints(rot, v, position_dict, image_depth, parent_jnt)


def get_2d_joint_data(selected_joints):

    cam_info = get_coordinates()[-1]
    print "cam_info: ", cam_info
    select_joints_info = get_skl_translation(selected_joints)
    selected_joints_name = select_joints_info.keys()
    selected_joints_coord = select_joints_info.values()

    print "======================================================="
    print "select_joints_info: ", selected_joints_name, "---", selected_joints_coord
    print "======================================================="
    new_coord_info = c_api.project_joint(mm_cm_m, selected_joints_coord, selected_joints_name, cam_info)
    print "position returned from C: ", new_coord_info

    new_coord_info = sorted(new_coord_info.items(), key=lambda x: x[0])

    result = {}
    for i in selected_joints_name:
        result[i] = []
    for i in new_coord_info:
        for j in selected_joints_name:
            if re.match(j, i[0]):
                result[j].append(i[1])
    print "======================================================="
    print "result: ", result
    print "======================================================="
    percentage_result = get_percentage(cam_info[4], cam_info[5], result)

    return percentage_result, cam_info


def get_percentage(resolution_x, resolution_y, coordinate_2d):
    result_dict = {}
    for k, v in coordinate_2d.items():
        x_percentage = v[0] / resolution_x
        y_percentage = v[1] / resolution_y
        result_dict[k] = [x_percentage, y_percentage]

    return result_dict


def get_skl_translation(skl_dict):
    translation_dict = {}

    def get_translation(target_dict):
        for k, v in target_dict.items():
            translation = cmds.xform(k, q=1, ws=1, rp=1)
            translation_dict[k] = translation
            if v:
                get_translation(v)

    get_translation(skl_dict)
    return translation_dict


def get_current_mskel_file(project='', char_name='', real_char_name=''):
    template = "T:/projects/{}/asset/char/{}/Tpose/{}/{}_Tpose.mskel".format(project, char_name, real_char_name,
                                                                             char_name)

    return "C:/Users/admin/haojw/GitLab/maya/PLE-22602/target_tool/mskel/Ling_BNRF_Tpose.mskel"


def get_camera_image_plane():
    camera_list = pm.ls("*cam*", type="camera")
    image_plane_list = pm.ls("*imagePlane*", type="imagePlane")
    if len(camera_list) == 1 and len(image_plane_list) == 1 and \
            camera_list[0].getChildren()[0] == image_plane_list[0].getParent():
        return camera_list[0], image_plane_list[0]
    else:
        for camera in camera_list:
            for image_plane in image_plane_list:
                if camera.name() in image_plane.name():
                    return camera, image_plane


def get_coordinates():
    cam, cam_image_plane = get_camera_image_plane()
    cam = cam.getParent()
    image_depth = cam_image_plane.depth.get()

    cam_m = cam.getMatrix(ws=1)

    cam_z_vector = pm.dt.Vector(cam_m[2][:3])
    cam_y_vector = pm.dt.Vector(cam_m[1][:3])
    cam_x_vector = pm.dt.Vector(cam_m[0][:3])
    cam_z_vector.normalize()
    cam_y_vector.normalize()
    cam_x_vector.normalize()

    image_p = cam.getTranslation(space="world") + (cam_z_vector * -image_depth)

    fl = cam.focalLength.get()
    apv = cam.verticalFilmAperture.get()
    aph = cam.horizontalFilmAperture.get()

    fov = math.atan((0.5 * aph) / (fl * 0.03937))

    x = math.tan(fov) * image_depth
    y = x * (apv / aph)

    top_right = image_p + cam_x_vector * x + cam_y_vector * y
    top_left = image_p + cam_x_vector * -x + cam_y_vector * y
    bottom_right = image_p + cam_x_vector * x + cam_y_vector * -y
    bottom_left = image_p + cam_x_vector * -x + cam_y_vector * -y

    # camera para: angle, pos_cm, film_width_inch, film_height_inch, width_pixel, height_pixel, focal_mm
    pos_cm = cam.getTranslation(space="world")
    angle = cam.getRotation()
    width_pixel = cam_image_plane.coverageX.get()
    height_pixel = cam_image_plane.coverageY.get()
    camera_para = [list(angle), list(pos_cm), aph, apv, width_pixel, height_pixel, fl]

    return top_right, top_left, bottom_left, bottom_right, image_depth, camera_para


def calculate_3d_pos(top_right, top_left, bottom_left, percentage):
    result_dict = {}
    top_right_p = pm.dt.Point(top_right)
    top_left_p = pm.dt.Point(top_left)
    bottom_left_p = pm.dt.Point(bottom_left)

    width = top_left_p.distanceTo(top_right_p)
    height = top_left_p.distanceTo(bottom_left_p)
    for k, v in percentage.items():
        p_width = width * v[0]
        p_height = height * v[1]

        vector_x = top_right_p - top_left_p
        vector_x.normalize()
        vector_y = bottom_left_p - top_left_p
        vector_y.normalize()

        result_dict[k] = top_left_p + vector_y * p_height + vector_x * p_width
    return result_dict


def compare_xyz(coord_1, coord_2):
    coord_1 = [round(i, 6) for i in coord_1]
    coord_2 = [round(i, 6) for i in coord_2]
    if coord_1 != coord_2:
        # print coord_1, "---", coord_2
        return False
    else:
        return True


if __name__ == '__main__':
    mskel_file = get_current_mskel_file()
    c_api.test_init_video_maker_haojw(mskel_file)
    test = getMayaMainWindow()
    print dir(test)
    ui = MainWindow()
    ui.show()
    # sys.exit(app.exec_())
