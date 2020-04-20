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
import bmesh
from bpy_types import Operator
from bpy.props import BoolProperty
from mathutils import Vector


class InputPoint:
    co : Vector((0, 0, 0))

    def __init__(self, co):
        self.co = co


def get_points_from_children(context, obj):
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


class AddVoronoiCells(Operator):
    """Generate Voronoi cells on a surface."""
    bl_idname = 'mesh.voronoi_add'
    bl_label = 'Add Voronoi Cells'
    bl_options = {'UNDO', 'REGISTER'}

    clear_mesh_data: BoolProperty(
        name        = "Clear Mesh",
        description = "Clear existing mesh data",
        default     = True)

    @classmethod
    def poll(cls, context):
        active_object = context.active_object
        return active_object and active_object.type == 'MESH'

    def gen_spheronoids(self, context):
        obj = context.active_object

        points = get_points_from_children(context, obj)

        bm = bmesh.new()
        bm.from_mesh(obj.data)

        if self.clear_mesh_data:
            bm.clear()

        for pt in points:
            bm.verts.new(pt.co)

        # edges = get_edge_rings(bm, keep_caps = True)
        # if not edges:
        #     self.report({'WARNING'}, "No suitable selection found")
        #     return False

        # result = bmesh.ops.subdivide_edges(
        #     bm,
        #     edges = edges,
        #     cuts = int(self.num_cuts),
        #     use_grid_fill = bool(self.use_grid_fill),
        #     use_single_edge = bool(self.use_single_edge),
        #     quad_corner_type = str(self.corner_type))

        # bpy.ops.mesh.select_all(action='DESELECT')
        # bm.select_mode = {'EDGE'}

        # inner = result['geom_inner']
        # for edge in filter(lambda e: isinstance(e, bmesh.types.BMEdge), inner):
        #     edge.select = True

        bm.to_mesh(obj.data)
        bm.free()

        obj.data.update()

        return True

    def execute(self, context):
        active_object = context.active_object
        orig_mode = active_object.mode

        bpy.ops.object.mode_set(mode='OBJECT')
        try:
            if not self.gen_spheronoids(context):
                return {'CANCELLED'}
        finally:
            bpy.ops.object.mode_set(mode=orig_mode)

        return {'FINISHED'}


def register():
    bpy.utils.register_class(AddVoronoiCells)

def unregister():
    bpy.utils.unregister_class(AddVoronoiCells)
