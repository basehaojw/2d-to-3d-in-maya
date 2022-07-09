# coding=utf-8
import ctypes
import os
interface_path = 'S:/users/lingyiwang/InteractivePoseEditing/c_interface.dll'
env_path = 'S:/users/jianjie/interactiveEditingDLL'

os.environ['PATH'] = os.environ['PATH'] + env_path

tbb = 'S:/users/jianjie/interactiveEditingDLL/tbb.dll'
ctypes.CDLL(tbb)
interact_lib = ctypes.CDLL(interface_path)

inited = False

# Struct定义中的变量顺序要和.h中的一致!
class CPose(ctypes.Structure):

    _fields_ = [('p0', ctypes.c_float),
                ('p1', ctypes.c_float),
                ('p2', ctypes.c_float),
                ('p3', ctypes.c_float),
                ('p4', ctypes.c_float),
                ('p5', ctypes.c_float),
                ]


class ReturnPose(ctypes.Structure):

    _fields_ = [("head", CPose),
               ("neck1", CPose),
               ("neck2", CPose),
               ("root", CPose),
               ("spline1", CPose),
               ("spline2", CPose),
               ("spline3", CPose),
               ]


class MayaParam(ctypes.Structure):
    _fields_ = [("joints", ctypes.c_char_p),
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


interact_lib.init_video_maker.argtypes = [ctypes.c_char_p]
interact_lib.init_video_maker.restype = ctypes.c_void_p
# ===================
interact_lib.optimize_frame.argtypes = [ctypes.POINTER(CPose), ctypes.POINTER(CPose),
                                        ctypes.POINTER(CPose), ctypes.POINTER(CPose),
                                        ctypes.POINTER(CPose), ctypes.POINTER(CPose), ctypes.POINTER(CPose),
                                        ctypes.c_int, ctypes.POINTER(MayaParam), ctypes.POINTER(ReturnPose),
                                        ]
interact_lib.optimize_frame.restype = ctypes.c_void_p



def test_interactive_c_struct():
    lib = ctypes.CDLL('c_interface')
    lib.test_py_arguments.argtypes = [ctypes.POINTER(CPose), ctypes.c_char_p]
    lib.test_py_arguments.restype = ctypes.c_void_p
    cpose = CPose(1, 2, 3, 4, 5, 6)
    lib.test_py_arguments(ctypes.byref(cpose), b"yaml path")
    return


def optimization_init_helper(path=r"joint2d_mskel.yaml"):
    global interact_lib
    global inited
    if inited:
        return
    interact_lib.init_video_maker(path.encode())
    inited = True
    return


def optimization_helper(head: list, neck1: list, neck2: list,
                        root: list, spline1: list, spline2: list, spline3: list, mm_cm_m: int,
                        joints: list, selected_joints_name: list, cur_mskel_bones_name: list, camera_aram: list) -> dict:
    global interact_lib
    # ================= pose 赋值
    head_pose = CPose(head[0], head[1], head[2], 0, 0, 0)
    neck1_pose = CPose(neck1[0], neck1[1], neck1[2], 0, 0, 0)
    neck2_pose = CPose(neck2[0], neck2[1], neck2[2], 0, 0, 0)
    root_pose = CPose(root[0], root[1], root[2], root[3], root[4], root[5])
    spline1_pose = CPose(spline1[0], spline1[1], spline1[2], 0, 0, 0)
    spline2_pose = CPose(spline2[0], spline2[1], spline2[2], 0, 0, 0)
    spline3_pose = CPose(spline3[0], spline3[1], spline3[2], 0, 0, 0)
    # ================ maya
    joints_str = ";".join([f"{i[0]},{i[1]}" for i in joints])
    selected_joints_name_str = ",".join(selected_joints_name)
    cur_mskel_bones_name_str = ",".join(cur_mskel_bones_name)

    angle_str = ",".join([f"{camera_aram[0][0]}", f"{camera_aram[0][1]}", f"{camera_aram[0][2]}"])
    pos_cm_str = ",".join([f"{camera_aram[1][0]}", f"{camera_aram[1][1]}", f"{camera_aram[1][2]}"])
    film_width_inch, film_height_inch, width_pixel, height_pixel, focal_mm = camera_aram[2:]
    maya_param = MayaParam(joints_str.encode(), selected_joints_name_str.encode(), cur_mskel_bones_name_str.encode(),
                           angle_str.encode(), pos_cm_str.encode(), film_width_inch, film_height_inch,
                           width_pixel, height_pixel, focal_mm,
                           )
    # ==================
    resp_pose = ReturnPose()
    # =================
    interact_lib.optimize_frame(ctypes.byref(head_pose), ctypes.byref(neck1_pose),
                                ctypes.byref(neck2_pose), ctypes.byref(root_pose),
                                ctypes.byref(spline1_pose), ctypes.byref(spline2_pose), ctypes.byref(spline3_pose),
                                mm_cm_m, maya_param, ctypes.byref(resp_pose))
    res = dict()
    res["head"] = [resp_pose.head.p0, resp_pose.head.p1, resp_pose.head.p2]
    res["neck1"] = [resp_pose.neck1.p0, resp_pose.neck1.p1, resp_pose.neck1.p2]
    res["neck2"] = [resp_pose.neck2.p0, resp_pose.neck2.p1, resp_pose.neck2.p2]
    res["root"] = [resp_pose.root.p0, resp_pose.root.p1, resp_pose.root.p2, resp_pose.root.p3, resp_pose.root.p4, resp_pose.root.p5]
    res["spline1"] = [resp_pose.spline1.p0, resp_pose.spline1.p1, resp_pose.spline1.p2]
    res["spline2"] = [resp_pose.spline2.p0, resp_pose.spline2.p1, resp_pose.spline2.p2]
    res["spline3"] = [resp_pose.spline3.p0, resp_pose.spline3.p1, resp_pose.spline3.p2]
    return res

def test_helper():
    # 初始化
    optimization_init_helper(r"C:/Users/admin/haojw/GitLab/maya/PLE-22602/InteractivePoseEditing/Interface/joint2d_mskel.yaml")

    # 赋值pose
    head = [0.0350713, -0.036958, 0.0577193]
    neck1 = [0.0482044, -0.0763037, 0.0150297]
    neck2 = [0.0486032, -0.0563124, 0.0310278]
    root = [4713.64, 1479.32, -5983.26, -2.35504, -0.00843069, 0.0394872]
    spline1 = [-0.248062, -0.0715346, 0.109785]
    spline2 = [-0.258886, -0.0721044, 0.085626]
    spline3 = [-0.269955, -0.0755426, 0.0464903]
    # maya 参数
    joints = [[897, 440], [661, 479]]
    selected_joints_name = ["lhumerus", "rhumerus"]
    cur_mskel_bones_name = ["root", "spline1", "spline2", "spline3"]
    # maya 相机参数
    angle = [177.859, 37.5894, 179.642]
    pos_cm = [629.537, 192.878, -835.797]
    film_width_inch = 0.925197
    film_height_inch = 0.520423
    width_pixel = 1920
    height_pixel = 1080
    focal_mm = 35
    camera_param = [angle, pos_cm, film_width_inch, film_height_inch, width_pixel, height_pixel, focal_mm]
    # 0表示mm, scale为0.001, 1表示cm, scale为0.01, 2表示m, scale为1.0
    mm_cm_m = 0
    # 调用
    res = optimization_helper(head, neck1, neck2, root, spline1, spline2, spline3, mm_cm_m,
                              joints, selected_joints_name, cur_mskel_bones_name, camera_param)
    # res = {head: [pose1, pose2, pose3], neck1: [pose1, pose2, pose3], ...}
    for key, value in res.items():
        print(key, value)
    return


def main():
    # test_interactive_c_struct()
    # test_optimization()
    test_helper()
    return


if __name__ == "__main__":
    main()
