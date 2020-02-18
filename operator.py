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


def edit_mode_out():
    bpy.ops.object.mode_set(mode='OBJECT')


def edit_mode_in():
    bpy.ops.object.mode_set(mode='EDIT')


class AddSpheronoids(Operator):
    """Generate a random set of points and Voronoi cells on a unit sphere."""
    bl_idname = 'mesh.spheronoids_add'
    bl_label = 'Add Spheronoids'

    clear_mesh_data: BoolProperty(
        name        = "Clear Mesh",
        description = "Clear existing mesh data",
        default     = True)

    @classmethod
    def poll(cls, context):
        active_object = context.active_object
        return active_object and active_object.type == 'MESH' and active_object.mode == 'EDIT'

    def gen_spheronoids(self, context):
        edit_mode_out()

        try:
            object = context.active_object
            bm = bmesh.new()
            bm.from_mesh(object.data)

            if self.clear_mesh_data:
                bm.clear()

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

            bm.to_mesh(object.data)
            bm.free()

        finally:
            edit_mode_in()

        return True

    def execute(self, context):
        if not self.gen_spheronoids(context):
            return {'CANCELLED'}

        return {'FINISHED'}


def register():
    print("I REGISTERED!")
    bpy.utils.register_class(AddSpheronoids)

def unregister():
    print("I UNREGISTERED!")
    bpy.utils.unregister_class(AddSpheronoids)
