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

sys.path.append("S:/users/haojiawei/head_change_test/FormalTool")
from re_package import re_packaging as c_api
from re_package.config import PROJECTION_ICON, OPT_ICON, VIEW_ICON, JOINT_ICON

mm_cm_m = 1


def get_maya_main_window():
    window_ptr = omui.MQtUtil.mainWindow()
    return shiboken2.wrapInstance(long(window_ptr), QtWidgets.QWidget)


def check_mult_window(maya_window):
    for i in maya_window.children():
        if i.objectName() == "2d_editing_tool":
            i.close()


class MainWindow(QtWidgets.QWidget):
    def __init__(self):
        open_file_dialog = QtWidgets.QDialog()
        super(MainWindow, self).__init__()
        parent = get_maya_main_window()
        check_mult_window(parent)
        self.setParent(parent)
        self.setWindowFlags(QtCore.Qt.Window)
        self.setWindowTitle('Projection&Optimization')
        self.setObjectName("2d_editing_tool")
        self.setMinimumSize(100, 120)
        self.main_lay = QtWidgets.QVBoxLayout()

        top_lay = QtWidgets.QHBoxLayout()

        mskel_label = QtWidgets.QLabel('Mskel file: ')
        self.mskel_line = MyLineEdit("")
        self.mskel_line.setToolTip("you can drop file here")
        self.mskel_line.setAcceptDrops(True)
        self.mskel_line.returnPressed.connect(self.init_video_maker)

        self.chose_btn = QtWidgets.QToolButton(open_file_dialog)
        self.chose_btn.clicked.connect(self._open_file_dialog)

        top_lay.addWidget(mskel_label)
        top_lay.addWidget(self.mskel_line)
        top_lay.addWidget(self.chose_btn)

        mid_lay = QtWidgets.QHBoxLayout()

        select_view_btn = QtWidgets.QPushButton('Select cam')
        select_view_btn.clicked.connect(self.set_current_cam)
        select_view_btn.setIcon(QtGui.QIcon(VIEW_ICON))
        self.selected_cam_label = QtWidgets.QLabel("")
        mid_lay.addWidget(select_view_btn)
        mid_lay.addWidget(self.selected_cam_label)

        mid_btm_lay = QtWidgets.QHBoxLayout()

        select_root_joint_btn = QtWidgets.QPushButton('Select root joint')
        select_root_joint_btn.setIcon(QtGui.QIcon(JOINT_ICON))
        select_root_joint_btn.clicked.connect(self.set_root_joint)
        self.select_root_label = QtWidgets.QLabel("")
        mid_btm_lay.addWidget(select_root_joint_btn)
        mid_btm_lay.addWidget(self.select_root_label)

        btm_lay = QtWidgets.QHBoxLayout()
        self.project_btn = QtWidgets.QPushButton('Project 2d joints')
        self.project_btn.setIcon(QtGui.QIcon(PROJECTION_ICON))
        self.project_btn.setMinimumSize(130, 40)

        optimize_btn = QtWidgets.QPushButton('Optimize')
        optimize_btn.setIcon(QtGui.QIcon(OPT_ICON))
        optimize_btn.setMinimumSize(130, 40)
        btm_lay.addWidget(self.project_btn)
        btm_lay.addWidget(optimize_btn)

        self.main_lay.addLayout(top_lay)
        self.main_lay.addLayout(mid_lay)
        self.main_lay.addLayout(mid_btm_lay)
        self.main_lay.addLayout(btm_lay)

        self.setLayout(self.main_lay)

        self.project_btn.clicked.connect(self.generate_2d_joints)
        optimize_btn.clicked.connect(self.optimize)

        self.mskel_line.setStyleSheet('''QLineEdit{
                                            border:1px solid gray;
                                            width:300px;
                                            border-radius:7px;
                                            padding:2px 4px;
                                    }''')

        self.retranslate_ui(open_file_dialog)
        QtCore.QMetaObject.connectSlotsByName(open_file_dialog)

        self.maya_position = {}
        self.cam = ""
        self.cam_info = []
        self.root = ""
        self.init_video_maker_status = False
        self.current_frame = cmds.currentTime(q=1)
        self.namespace = ''

        self.test_set_default()

    def test_set_default(self):
        if cmds.ls("Chest_M"):
            self.select_root_label.setText("Chest_M")
            self.root = "Chest_M"
        self.mskel_line.setText("S:/users/haojiawei/head_change_test/mskel/Ling_BNRF_Tpose.mskel")

    def retranslate_ui(self, TestQFileDialog):
        _translate = QtCore.QCoreApplication.translate
        TestQFileDialog.setWindowTitle(_translate("TestQFileDialog", "Dialog"))
        self.chose_btn.setText(_translate("TestQFileDialog", "..."))

    def _open_file_dialog(self):
        # directory = str(QtWidgets.QFileDialog.getExistingDirectory())

        _target_file = QtWidgets.QFileDialog.getOpenFileName(None, "getOpenFileName",
                                                             "T:/project/Huantou_TestYudi/asser/char/Ling_BNRF/Tpose/yuxuan",
                                                             "All Files (*);;Text Files (*.mskel)")
        print _target_file, type(_target_file)
        self.mskel_line.setText('{}'.format(_target_file[0]))

    def set_current_cam(self):

        current_cam_node = pm.ls(sl=1)
        if not current_cam_node:
            confirm_dialog(1, "Selected nothing! Please select something relate camera")

        if len(current_cam_node) > 1:
            confirm_dialog(1, "Got more than one object selected, select the first one by default")

        current_cam_node = current_cam_node[0]
        cam_type = ["transform", "camera"]
        if current_cam_node.nodeType() not in cam_type:
            confirm_dialog(1, 'Current select object {} nothing to do with camera ,please select camera object!'.
                           format(current_cam_node.name()))

            print "please select object about cam or view!!"

        if current_cam_node.nodeType() == "camera":
            current_transform_cam = current_cam_node.getParent()
            print "Selected view name: ", current_transform_cam.name()

            self.cam = current_transform_cam.name()
            self.selected_cam_label.setText(self.cam)
        elif current_cam_node.nodeType() == "transform" and has_cam_child(current_cam_node):
            print "Selected view name: ", current_cam_node.name()
            self.cam = has_cam_child(current_cam_node).getParent().name()
            self.selected_cam_label.setText(self.cam)
        else:
            print "please select object about cam or view!!"
            confirm_dialog(1, "Current select object {} nothing to do with camera,please select camera or camera "
                              "relative object!".format(current_cam_node.name()))

    def set_root_joint(self):
        root_joint = cmds.ls(sl=1, type="joint")
        if not root_joint:
            confirm_dialog(1, "Please select the root joint")
            return
        if len(root_joint) > 1:
            confirm_dialog(1, "Root joint should be 1 {} got".format(len(root_joint)))
            return
        root_joint = root_joint[0]
        parent = cmds.listRelatives(root_joint, parent=1)[0]
        if pm.PyNode(parent).nodeType() == "joint":
            confirm_dialog(1, "Please Select the right root joint,current selected is [{}] !".format(root_joint))
            return
        self.root = root_joint
        self.select_root_label.setText(self.root)
        print "set root joint: ", self.root
        if pm.PyNode(self.root).namespace():
            self.namespace = pm.PyNode(self.root).namespace()
            print "get namespace: ", self.namespace

    def init_video_maker(self):
        mskel_file = self.mskel_line.text()
        if not mskel_file:
            confirm_dialog(1, "Please input mskel file !")
            return
        elif not os.path.exists(mskel_file):
            confirm_dialog(1, "Mskel file not exist please check !")
            return
        c_api.test_init_video_maker_haojw(mskel_file)
        self.init_video_maker_status = True
        print "init video maker done!!!"

    def generate_2d_joints(self):
        self.current_frame = cmds.currentTime(q=1)
        joints_dict = get_skl_data()
        if not joints_dict:
            return
        sphare_item = []
        for i in joints_dict:
            for j in joints_dict:
                if j in joints_dict[i]:
                    print j
                    sphare_item.append(j)
        for i in sphare_item:
            joints_dict.pop(i)

        joints_2d_percentage_data, self.cam_info = get_2d_joint_data(joints_dict, self.cam)

        print "percentage dict: ", joints_2d_percentage_data
        top_right, top_left, bottom_left = get_coordinates(self.cam)[:3]
        if not top_right or not top_left or not bottom_left:
            return
        self.maya_position = calculate_3d_pos(top_right, top_left, bottom_left, joints_2d_percentage_data)
        print "maya_position: ", self.maya_position
        image_depth = get_coordinates(self.cam)[-2]
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
        if not self.init_video_maker_status:
            self.init_video_maker()
        if cmds.currentTime(q=1) != self.current_frame:
            print "Frame changed! Please do projection again than do optimization"
            confirm_dialog(1, "Frame changed! Please do projection again than do optimization")

            return
        print 'optimizing...'
        current_2d_jnts_trans = get_current_2d_jnts_trans()
        moved_jnts = {}
        for i in current_2d_jnts_trans:
            for j in self.maya_position:
                if re.match(j + "_2d", i):
                    compare_result = compare_xyz(list(current_2d_jnts_trans[i]), list(self.maya_position[j]))
                    if not compare_result:
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
                if re.match(i + '_2d', j):
                    result_2d_pos[i] = [new_2d_projected_pos[i + '_2d_x'], new_2d_projected_pos[i + '_2d_y']]
        result_2d_pos = {k: v for k, v in result_2d_pos.items() if pm.PyNode(k).nodeType() == "joint"}
        pose_map = get_post_map_test(self.root)

        cur_mskel_bones = get_cur_mskel_bones(self.root)
        print "==========================================="
        print "pose_map: ", pose_map
        print "\n"
        print "result_2d_pos: ", result_2d_pos
        print "\n"
        print "cur_mskel_bones: ", cur_mskel_bones
        print "==========================================="
        if self.namespace:
            no_ns_pose_map = {k.replace(self.namespace, ""): v for k, v in pose_map.items()}
            no_ns_result_2d_pos = {k.replace(self.namespace, ""): v for k, v in result_2d_pos.items()}
            no_ns_cur_mskel_bones = [i.replace(self.namespace, "") for i in cur_mskel_bones]
            no_ns_result = c_api.optimize(no_ns_pose_map, mm_cm_m, no_ns_result_2d_pos.values(),
                                          no_ns_result_2d_pos.keys(), no_ns_cur_mskel_bones, self.cam_info)
            result = {self.namespace + k: v for k, v in no_ns_result.items()}
        else:
            result = c_api.optimize(pose_map, mm_cm_m, result_2d_pos.values(),
                                    result_2d_pos.keys(), cur_mskel_bones, self.cam_info)
            print "Final result: ", result

        for i in result:
            # if i in cur_mskel_bones:
            if len(result[i]) > 3:
                print "before    joint: ", i, "  value: ", pose_map[i]
                print "optimized joint: ", i, "  value: ", result[i]
                # pm.PyNode(i).translate.set(*result[i][:3])
                result[i] = [math.degrees(j) for j in result[i]]
                try:
                    pm.PyNode(i).rotate.set(*result[i][-3:])
                except Exception as e:
                    print e
                    confirm_dialog(2, "{} Can not set rotation attr".format(i))
            else:
                print "before    joint: ", i, "  value: ", pose_map[i][-3:]
                print "optimized joint: ", i, "  value: ", result[i][-3:]
                result[i] = [math.degrees(j) for j in result[i]]
                try:
                    pm.PyNode(i).rotate.set(*result[i][-3:])
                except Exception as e:
                    print e
                    confirm_dialog(2, "{} Can not set rotation attr".format(i))


def get_cur_mskel_bones(root_bones):
    mskel_jnts = pm.ls(sl=1)
    if not mskel_jnts:
        confirm_dialog(1, 'Please select joints need to be optimized ')
        return
    for i in mskel_jnts:
        if root_bones not in i.longName():
            confirm_dialog(1, 'Select joints in Wrong place, please check!')
            return
    result = [i.shortName() for i in mskel_jnts]
    return result


def get_current_2d_jnts_trans():
    joints_2d_grp = pm.ls("joints_2d", type="transform")
    if not joints_2d_grp:
        print "Please generate 2d joints first"
        confirm_dialog(1, 'Please generate 2d joints first')

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


def get_post_map_test(root_jnt_name):
    root_jnt = cmds.ls(root_jnt_name, type="joint")

    if not root_jnt:
        print "Can not get the root joint {}".format(root_jnt_name)
        raise Exception("Can not get the root joint {}".format(root_jnt_name))
    if len(root_jnt) > 1:
        raise Exception("More than one root joint found, please check !!!")
    root_jnt = root_jnt[0]
    children = cmds.listRelatives(root_jnt, ad=1)
    children = [i for i in children if pm.PyNode(i).nodeType() == "joint"]
    p_trans = list(pm.PyNode(root_jnt).getTranslation())
    p_rot = list(pm.PyNode(root_jnt).getRotation())
    p_rot = [math.radians(i) for i in p_rot]
    root = {root_jnt: reduce(add, (p_trans, p_rot))}
    branch = {i: [math.radians(j) for j in list(pm.PyNode(i).getRotation())] for i in children}
    result = dict()
    result.update(root)
    result.update(branch)

    return result


def get_skl_data():
    joints_data = {}
    root_system = cmds.ls(sl=1)
    if not root_system:
        print "got nothing selected, please check..."
        confirm_dialog(1, 'got nothing selected, please check...')

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
        # 0.003 * image_depth
        cmds.joint(name=joint_name, rad=0.002 * image_depth, p=position, o=rot)
        # cmds.setAttr('{}.translateZ'.format(joint_name), l=1)
        # cmds.setAttr('{}.rotateZ'.format(joint_name), l=1)
        cmds.setAttr('{}.overrideEnabled'.format(joint_name), 1)
        cmds.setAttr(".ovc", 6)
        rot = [0, 0, 0]
        if v:
            parent_jnt = joint_name
            create_2d_joints(rot, v, position_dict, image_depth, parent_jnt)


def get_2d_joint_data(selected_joints, cam):
    cam_info = get_coordinates(cam)[-1]
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


# def get_current_mskel_file(project='', char_name='', real_char_name=''):
#     template = "T:/projects/{}/asset/char/{}/Tpose/{}/{}_Tpose.mskel".format(project, char_name, real_char_name,
#                                                                              char_name)
#
#     return "S:/users/haojiawei/head_change_test/DEBUG/PLE-22602/tool/tool-ui/Ling_BNRF_Tpose.mskel"

def has_cam_child(obj_node):
    for i in obj_node.listRelatives(ad=1):
        if i.nodeType() == "camera":
            return i
    return False


def get_camera_image_plane_v1(current_cam_name):
    current_perspective_name = current_cam_name

    current_image_plane = get_child_image_plane(current_perspective_name)

    if not current_image_plane:
        confirm_dialog(1, 'Can not get image plane in {},Please check!!!'.format(current_perspective_name))

        return
    current_image_plane = current_image_plane[0]
    return pm.PyNode(current_perspective_name), pm.PyNode(current_image_plane)


def get_child_image_plane(cam):
    result = []

    def get_image_plane(root):
        # print "root: ", root
        for i in cmds.listRelatives(root):
            if cmds.ls(i, type="imagePlane"):
                # print "child image plane is: ", i
                result.append(str(i))
                break
            else:
                get_image_plane(i)
        return result

    try:
        result = get_image_plane(cam)
    except Exception as e:
        print e
        result = []
    return result


# def get_current_persp():

# pan = pm.getPanel(withFocus=True)
# try:
#     current_perspective_name = pm.windows.modelPanel(pan, query=True, camera=True)
# except Exception as e:
#     print e
#     current_perspective_name = pm.ls("*cam*", type="camera")[0]
#     current_perspective_name = current_perspective_name.getParent().shortName()
# print "View name: ", current_perspective_name
# return current_perspective_name


def get_coordinates(cam):
    if not cam:
        confirm_dialog(1, 'Can not get camera! Please make you have choose camera!')
        return
    cam, cam_image_plane = get_camera_image_plane_v1(cam)
    if not cam_image_plane:
        return
    # cam = cam.getParent()
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


def confirm_dialog(level=1, message=""):
    if level == 0:
        title = "Info"
    elif level == 1:
        title = "Warning"
    else:
        title = "Error"
    cmds.confirmDialog(title=title, message=message,
                       cancelButton='OK', dismissString='OK')


class MyLineEdit(QtWidgets.QLineEdit):
    def __init__(self, title):
        super(MyLineEdit, self).__init__(title)
        self.setAcceptDrops(True)

    def dragEnterEvent(self, e):
        if e.mimeData().hasText():
            e.accept()
        else:
            e.ignore()

    def dropEvent(self, e):
        file_path_list = e.mimeData().text()
        file_path = file_path_list.split('\n')[0]
        file_path = file_path.replace('file:///', '', 1)
        self.setText(file_path)


if __name__ == '__main__':
    # mskel_file = get_current_mskel_file()
    # c_api.test_init_video_maker_haojw(mskel_file)

    ui = MainWindow()
    ui.show()

    # sys.exit(app.exec_())
