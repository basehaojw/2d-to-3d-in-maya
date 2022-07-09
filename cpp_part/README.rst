
nuget依赖
=================


facecnn.1.0.11是比较老的包，包含了比较旧的gbasic, 你需要从S:\\virtualx-tool\\vcpkg\\installed\\x64-windows中复制gbasic的include和lib到本项目的nuget中

同时这个nuget包不包含levmar, 所以需要复制levmarReleasex64.lib到本地的nuget的包中

或者打一个新的nuget包



c_interface
=========================

为python生成的一个c dll, 编译好的dll放在S:\\users\\lingyiwang\\InteractivePoseEditing, 运行环境需要安装有VCRUNTIME140_1.dll(也包含在该目录中)

python的函数封装在该项目中interacitve_cstruct.py中

.. code-block:: python

    import interacitve_cstruct

    # 初始化
    interacitve_cstruct.optimization_init_helper(r"E:\InteractivePoseEditing\x64\Release\joint2d_mskel.yaml")

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
    res = interacitve_cstruct.optimization_helper(head, neck1, neck2, root, spline1, spline2, spline3, mm_cm_m,
                                                  joints, selected_joints_name, cur_mskel_bones_name, camera_param)
    # res = {head: [pose1, pose2, pose3], neck1: [pose1, pose2, pose3], ...}
    for key, value in res.items():
        print(key, value)


