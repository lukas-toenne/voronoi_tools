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
import math
from bpy_types import Operator
from bpy.props import BoolProperty
from mathutils import Vector


class InputPoint:
    co : Vector((0, 0, 0))

    def __init__(self, co):
        self.co = co


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


def radial_sort_points(points):
    center = Vector((0, 0, 0))
    for pt in points:
        center += pt.co
    center /= len(points)
    points.sort(key = lambda pt: (pt.co - center).length_squared)


def do_sweephull(bm, points):
    project_points(points)
    if len(points) < 3:
        return

    radial_sort_points(points)

    # Create vertices for all points
    point_verts = []
    for pt in points:
        point_verts.append(bm.verts.new(pt.co))
    bm.verts.ensure_lookup_table()

    def is_ccw_winding(i, j, k):
        a = points[i].co
        b = points[j].co
        c = points[k].co
        return (b - a).cross(c - a).z > 0

    def add_point_to_convex_hull(convex_hull, point_index):
        new_convex_hull = []

        # Returns true if the point is on the outside of the convex hull edge (ci, cj).
        # Indices are in the convex_hull array!
        def is_edge_visible(ci, cj):
            return not is_ccw_winding(point_index, convex_hull[ci], convex_hull[cj])

        was_visible = is_edge_visible(-1, 0) # visibility of last edge
        for c in range(len(convex_hull)):
            next_c = (c + 1) % len(convex_hull)
            # Is the convex hull edge visible from the new point?
            is_visible = is_edge_visible(c, next_c)

            hull_index = convex_hull[c]
            hull_index_next = convex_hull[next_c]

            # Connect to visible edges
            if is_visible:
                bm.faces.new((point_verts[point_index], point_verts[hull_index_next], point_verts[hull_index]))

            # Update the convex hull

            # Discard vertex if both edge are visible from the new point
            if not (was_visible and is_visible):
                new_convex_hull.append(hull_index)

            # Insert new point at start of visible section
            if (not was_visible) and is_visible:
                new_convex_hull.append(point_index)

            was_visible = is_visible

        convex_hull[:] = new_convex_hull

    # Add first 3 verts as the first triangle
    bm.faces.new((point_verts[0], point_verts[1], point_verts[2]))
    convex_hull = [0, 1, 2] if is_ccw_winding(0, 1, 2) else [2, 1, 0]

    for i in range(3, len(points)):
        add_point_to_convex_hull(convex_hull, i)


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

    def generate_mesh(self, context):
        obj = context.active_object

        bm = bmesh.new()
        bm.from_mesh(obj.data)

        if self.clear_mesh_data:
            bm.clear()

        do_sweephull(bm, get_points_from_children(context))

        bm.to_mesh(obj.data)
        bm.free()

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


def register():
    bpy.utils.register_class(AddVoronoiCells)

def unregister():
    bpy.utils.unregister_class(AddVoronoiCells)
