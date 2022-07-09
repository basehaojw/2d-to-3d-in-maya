import pymel.core as pm
import math

cam = pm.PyNode("trk_cam")
cam_imagPlan = pm.PyNode("imagePlaneShape1")

imag_depth = cam_imagPlan.depth.get()

cam_m = cam.getMatrix(ws=1)

cam_z_vector = pm.dt.Vector(cam_m[2][:3])
cam_y_vector = pm.dt.Vector(cam_m[1][:3])
cam_x_vector = pm.dt.Vector(cam_m[0][:3])
cam_z_vector.normalize()
cam_y_vector.normalize()
cam_x_vector.normalize()

image_p = cam.getTranslation(space="world") + (cam_z_vector * -imag_depth)

fl = cam.focalLength.get()
apv = cam.verticalFilmAperture.get()
aph = cam.horizontalFilmAperture.get()

fov = math.atan((0.5 * aph) / (fl * 0.03937))

x = math.tan(fov) * imag_depth
y = x * (apv / aph)

print(x, y)
