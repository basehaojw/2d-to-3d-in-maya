# coding=utf-8
import ctypes
import os
import json
from config import INTERFACE_DLL, TBB_DLL, ENV_PATH

os.environ['PATH'] = os.environ['PATH'] + os.pathsep + ENV_PATH

ctypes.CDLL(TBB_DLL)
interact_lib = ctypes.CDLL(INTERFACE_DLL)
inited = False


class CPose(ctypes.Structure):
    _fields_ = [('p0', ctypes.c_float),
                ('p1', ctypes.c_float),
                ('p2', ctypes.c_float),
                ('p3', ctypes.c_float),
                ('p4', ctypes.c_float),
                ('p5', ctypes.c_float),
                ]


# CPose test[10];
class ReturnPoseHaojw(ctypes.Structure):
    _fields_ = [
        ('joint', CPose),
    ]


class ReturnPoseTest(ctypes.Structure):
    _fields_ = [
        ('result', ctypes.c_char_p),
    ]


class MayaParam(ctypes.Structure):
    _fields_ = [
        ("joints", ctypes.c_char_p),
        ("selected_joints_name", ctypes.c_char_p),
        ("cur_mskel_bones_name", ctypes.c_char_p),
        ("angle", ctypes.c_char_p),
        ("pos_cm", ctypes.c_char_p),
        ("film_width_inch", ctypes.c_float),
        ("film_height_inch", ctypes.c_float),
        ("width_pixel", ctypes.c_int),
        ("height_pixel", ctypes.c_int),
        ("focal_mm", ctypes.c_float),
    ]


interact_lib.test_init_video_maker_haojw.argtypes = [ctypes.c_char_p]
interact_lib.test_init_video_maker_haojw.restype = ctypes.c_void_p

# ==========================================================

interact_lib.test_opt_haojw.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.POINTER(MayaParam),
                                        ctypes.POINTER(ReturnPoseTest)]
interact_lib.test_opt_haojw.restype = ctypes.c_void_p

# ==========================================================

interact_lib.test_projection_haojw.argtypes = [ctypes.c_int, ctypes.POINTER(MayaParam), ctypes.POINTER(ReturnPoseTest)]
interact_lib.test_projection_haojw.restype = ctypes.c_void_p


def test_c_interface():
    interact_lib.test_py_arguments.argtypes = [ctypes.POINTER(CPose), ctypes.c_char_p]
    interact_lib.test_py_arguments.restype = ctypes.c_void_p
    cpose = CPose(0.15, 0.26, 0.37, 0.48, 0.59, 3.141596)
    interact_lib.test_py_arguments(cpose, "hello world")
    return


def init_video_maker(mskel_path):
    global interact_lib
    global inited
    if inited:
        return
    interact_lib.init_video_maker(mskel_path.encode())


def test_init_video_maker_haojw(yaml_path):
    global interact_lib
    global inited
    if inited:
        return
    interact_lib.test_init_video_maker_haojw(yaml_path.encode())
    inited = True


def project_joint(mm_cm_m, joints, selected_joints_name, camera_aram):
    """

    :rtype: object
    """
    joints_str = ";".join(["{},{},{}".format(i[0], i[1], i[2]) for i in joints])
    selected_joints_name_str = ",".join(selected_joints_name)
    angle_str = ",".join(
        ["{}".format(camera_aram[0][0]), "{}".format(camera_aram[0][1]), "{}".format(camera_aram[0][2])])
    pos_cm_str = ",".join(
        ["{}".format(camera_aram[1][0]), "{}".format(camera_aram[1][1]), "{}".format(camera_aram[1][2])])
    film_width_inch, film_height_inch, width_pixel, height_pixel, focal_mm = camera_aram[2:]

    maya_param = MayaParam(joints_str.encode(), selected_joints_name_str.encode(), "",
                           angle_str.encode(), pos_cm_str.encode(), film_width_inch, film_height_inch,
                           width_pixel, height_pixel, focal_mm,
                           )
    return_haojw = ReturnPoseTest()
    interact_lib.test_projection_haojw(mm_cm_m, maya_param, return_haojw)
    result_dict = json.loads(return_haojw.result)
    return result_dict


def optimize(target_joint, mm_cm_m, joints, selected_joints_name, cur_mskel_bones_name, camera_aram):

    # =============================
    joints_str = ";".join(["{},{}".format(i[0], i[1]) for i in joints])
    selected_joints_name_str = ",".join(selected_joints_name)
    cur_mskel_bones_name_str = ",".join(cur_mskel_bones_name)

    angle_str = ",".join(
        ["{}".format(camera_aram[0][0]), "{}".format(camera_aram[0][1]), "{}".format(camera_aram[0][2])])
    pos_cm_str = ",".join(
        ["{}".format(camera_aram[1][0]), "{}".format(camera_aram[1][1]), "{}".format(camera_aram[1][2])])
    film_width_inch, film_height_inch, width_pixel, height_pixel, focal_mm = camera_aram[2:]

    maya_param = MayaParam(joints_str.encode(), selected_joints_name_str.encode(), cur_mskel_bones_name_str.encode(),
                           angle_str.encode(), pos_cm_str.encode(), film_width_inch, film_height_inch,
                           width_pixel, height_pixel, focal_mm,
                           )
    return_haojw = ReturnPoseTest()
    interact_lib.test_opt_haojw(json.dumps(target_joint), mm_cm_m, maya_param, return_haojw)

    result_dict = json.loads(return_haojw.result)

    return result_dict


# this func only for test
def test_run_opt():
    # yaml_path = 'C:/Users/admin/haojw/GitLab/maya/PLE-22602/InteractivePoseEditing/test_c_interface/joint2d_mskel.yaml'
    # init_video_maker(yaml_path)
    mskel_file = 'C:/Users/admin/haojw/GitLab/maya/PLE-22602/target_tool/mskel/Ling_BNRF_Tpose.mskel'
    # test_mskel_file = "S:/users/jianjie/HeadChange/HeadChange_interface/sk_amc_dir/yuxuan_ok.mskel"
    test_init_video_maker_haojw(mskel_file)
    #
    target_joint = {"Chest_M": [0.0350713, -0.036958, 0.0577193, 0, 0, 0], "Neck_M": [17.243, -4.015, 0, 0, 0, 0]}

    # maya arges
    joints = [[529.8584219528263, 123.69765078377536, -112.48958952646794]]
    selected_joints_name = ["Head_M"]
    cur_mskel_bones_name = ["Neck_M", "Neck1_M", "Head_M"]

    # maya camera
    angle = (-2.742, -21.093, -2.879)
    pos_cm = (161.953, 149.341, 744.923)
    film_width_inch = 0.9877110803749809
    film_height_inch = 0.5555874827109267
    width_pixel = 1920
    height_pixel = 1080
    focal_mm = 65.967
    camera_param = [angle, pos_cm, film_width_inch, film_height_inch, width_pixel, height_pixel, focal_mm]

    mm_cm_m = 1

    # test_result = project_joint(mm_cm_m, joints, selected_joints_name, camera_param)
    # print test_result
    result = optimize(target_joint, mm_cm_m, joints, selected_joints_name, cur_mskel_bones_name, camera_param)
    #
    for k, v in result.items():
        print k, '-->', v


# test_c_interface()

# run_opt()

