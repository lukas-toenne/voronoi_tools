# ##### BEGIN MIT LICENSE BLOCK #####
#
# Copyright (c) 2020 Lukas Toenne
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# ##### END MIT LICENSE BLOCK #####

# <pep8 compliant>

import bpy
from bpy_types import Operator
from bpy.props import BoolProperty, EnumProperty, FloatProperty, FloatVectorProperty
from .triangulator import Triangulator, InputPoint


def get_points_from_children(context):
    obj = context.active_object
    depsgraph = context.evaluated_depsgraph_get()

    matinv = obj.matrix_world.inverted()

    points = []
    for child in obj.children:
        child_eval = child.evaluated_get(depsgraph)
        mesh_from_eval = child_eval.to_mesh()

        mat = matinv @ child_eval.matrix_world

        if mesh_from_eval:
            points.extend([InputPoint(mat @ v.co) for v in mesh_from_eval.vertices])

    return points


def project_points(points):
    for i in range(len(points)):
        points[i].co.z = 0.0


def limit_points(points, bounds_min, bounds_max):
    points[:] = [p for p in points if p.co.x >= bounds_min[0] and p.co.x <= bounds_max[0] and p.co.y >= bounds_min[1] and p.co.y <= bounds_max[1]]


class AddVoronoiCells(Operator):
    """Generate Voronoi cells on a surface."""
    bl_idname = 'mesh.voronoi_add'
    bl_label = 'Add Voronoi Cells'
    bl_options = {'UNDO', 'REGISTER'}

    output_graph : EnumProperty(
        name="Output Graph",
        description="Type of graph to generate from point data",
        items={
            ('VORONOI', "Voronoi", "Divides space into cells around the closest point"),
            ('DELAUNAY', "Delaunay", "Triangulation with maximised angles"),
            },
        default='VORONOI',
        )

    bounds_mode : EnumProperty(
        name="Bounds Mode",
        description="Bounds behavior mode",
        items={
            ('NONE', "None", "Use all points"),
            ('LIMIT', "Limit", "Use only points within limits"),
            ('REPEAT', "Repeat", "Repeat points outside bounds"),
            },
        default='NONE',
        )

    bounds_min : FloatVectorProperty(
        name="Bounds Minimum",
        description="Minimum bounds value",
        size=2,
        default=(0.0, 0.0),
        )

    bounds_max : FloatVectorProperty(
        name="Bounds Maximum",
        description="Maximum bounds value",
        size=2,
        default=(1.0, 1.0),
        )

    generate_debug_meshes : BoolProperty(
        name="Generate Debug Meshes",
        description="Generate a collection of debug meshes for each step",
        default=False,
        )

    @classmethod
    def poll(cls, context):
        active_object = context.active_object
        return active_object and active_object.type == 'MESH'

    def generate_mesh(self, context):
        obj = context.active_object

        points = get_points_from_children(context)
        project_points(points)
        if self.bounds_mode != 'NONE':
            limit_points(points, self.bounds_min, self.bounds_max)

        triangulator = Triangulator()
        if self.generate_debug_meshes:
            self.setup_debugging(context, obj, triangulator)

        if self.output_graph == 'DELAUNAY':
            del_bm = triangulator.construct_delaunay(points)

            del_bm.to_mesh(obj.data)
            del_bm.free()

        elif self.output_graph == 'VORONOI':
            del_bm = triangulator.construct_delaunay(points)
            voro_bm = triangulator.construct_voronoi(del_bm)

            voro_bm.to_mesh(obj.data)
            del_bm.free()
            voro_bm.free()

        obj.data.update()

        return True

    def execute(self, context):
        active_object = context.active_object
        orig_mode = active_object.mode

        bpy.ops.object.mode_set(mode='OBJECT')
        try:
            if not self.generate_mesh(context):
                return {'CANCELLED'}
        finally:
            bpy.ops.object.mode_set(mode=orig_mode)

        return {'FINISHED'}

    def setup_debugging(self, context, obj, triangulator):
        scene = context.scene

        # Add a collection to group all the debug meshes in
        collection = bpy.data.collections.get("VoronoiToolsDebug")
        if collection:
            # Clear existing collection
            old_objects = collection.objects[:]
            for old_obj in old_objects:
                old_data = old_obj.data
                bpy.data.objects.remove(old_obj)
                bpy.data.meshes.remove(old_data)
        else:
            collection = bpy.data.collections.new("VoronoiToolsDebug")
            scene.collection.children.link(collection)

        # Implementation of intermediate mesh debugging
        def add_debug_mesh(bm, name):
            # Store current bmesh state in a new mesh datablock
            debug_mesh = bpy.data.meshes.new("VoronoiToolsDebug.{}".format(name))
            bm.to_mesh(debug_mesh)
            debug_mesh.update()

            # Object to link the mesh in the scene
            debug_obj = bpy.data.objects.new("VoronoiToolsDebug.{}".format(name), debug_mesh)
            # Copy transform and other attributes from the main object
            debug_obj.location = obj.location
            debug_obj.rotation_euler = obj.rotation_euler
            debug_obj.rotation_quaternion = obj.rotation_quaternion
            debug_obj.rotation_axis_angle = obj.rotation_axis_angle
            debug_obj.scale = obj.scale
            # Useful visualization settings for debugging
            debug_obj.show_wire = True
            debug_obj.show_all_edges = True
            # Add keyframes to show the object only on one frame
            # This allows scrubbing the timeline to see the steps of the algorithm
            frame = len(collection.objects) + 1
            debug_obj.hide_viewport = True
            debug_obj.keyframe_insert("hide_viewport", frame=(frame - 1))
            debug_obj.hide_viewport = False
            debug_obj.keyframe_insert("hide_viewport", frame=(frame))
            debug_obj.hide_viewport = True
            debug_obj.keyframe_insert("hide_viewport", frame=(frame + 1))
            # Set scene frame range to cover all debug objects
            scene.frame_start = 1
            scene.frame_end = frame

            # Put debug object into the debug collection
            collection.objects.link(debug_obj)

        # Replace the dummy method in the triangulator for debug mesh generation
        triangulator.add_debug_mesh = add_debug_mesh


def register():
    bpy.utils.register_class(AddVoronoiCells)

def unregister():
    bpy.utils.unregister_class(AddVoronoiCells)
