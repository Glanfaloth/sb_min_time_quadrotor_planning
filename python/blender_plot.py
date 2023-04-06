import bpy
import csv
from mathutils import Vector

# Set the file paths
mesh_path = "/local/home/yeltao/thesis_ws/sb_min_time_quadrotor_planning/blender/random_columns.obj"
csv_path = "/local/home/yeltao/thesis_ws/sb_min_time_quadrotor_planning/path.csv"

# Load the mesh
bpy.ops.import_scene.obj(filepath=mesh_path)

# Load the path data
with open(csv_path) as csv_file:
    reader = csv.DictReader(csv_file)
    path_data = [row for row in reader]

# Create a curve to visualize the path
curve = bpy.data.curves.new(name="PathCurve", type='CURVE')
curve.dimensions = '3D'
spline = curve.splines.new('POLY')
spline.points.add(len(path_data) - 1)
for i, row in enumerate(path_data):
    point = Vector((float(row['p_x']), float(row['p_y']), float(row['p_z'])))
    spline.points[i].co = (point.x, point.y, point.z, 1)

# Create an object to display the curve
curve_obj = bpy.data.objects.new("PathObject", curve)
bpy.context.scene.collection.objects.link(curve_obj)

# Set the curve object to be in wireframe mode
curve_obj.display_type = 'WIRE'

# Select the mesh and curve objects
for obj in bpy.data.objects:
    obj.select_set(True)
curve_obj.select_set(True)

# Set the 3D cursor to the origin and move the objects there
bpy.context.scene.cursor.location = (0, 0, 0)
bpy.ops.object.origin_set(type='ORIGIN_CURSOR')
bpy.context.area.type = 'VIEW_3D'
bpy.ops.view3d.snap_cursor_to_center()
bpy.ops.object.location_clear()

# Center the view on the objects
bpy.ops.view3d.camera_to_view_selected()

# Set the 3D viewport to show both objects
bpy.ops.view3d.view_selected()
