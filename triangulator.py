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
from mathutils import Matrix, Vector

class InputPoint:
    co : Vector((0, 0, 0))

    def __init__(self, co):
        self.co = co

# Get the adjacent and opposing vertices for an edge diagonal.
# Returns vertices (a, b, c, d), where (a, c) is the edge and b, d are opposing vertices,
# as well as the associated edges (in CCW order).
def get_quad_verts(edge):
    va = edge.verts[0]
    vb = None
    vc = edge.verts[1]
    vd = None

    ea = None
    eb = None
    ec = None
    ed = None

    for loop in va.link_loops:
        prev_vert = loop.link_loop_prev.vert
        next_vert = loop.link_loop_next.vert
        if prev_vert == vc:
            vb = next_vert
            ea = loop.edge
            eb = loop.link_loop_next.edge
        if next_vert == vc:
            vd = prev_vert
            ed = loop.link_loop_prev.edge
            ec = loop.link_loop_prev.link_loop_prev.edge

    assert(va is not None)
    assert(vb is not None)
    assert(vc is not None)
    assert(vd is not None)
    return (va, vb, vc, vd), (ea, eb, ec, ed)

# Vertices must form a non-overlapping polygon (can be concave).
def is_delaunay(verts):
    a = verts[0].co
    b = verts[1].co
    c = verts[2].co
    d = verts[3].co
    M = Matrix((
        (a.x, a.y, a.length_squared, 1),
        (b.x, b.y, b.length_squared, 1),
        (c.x, c.y, c.length_squared, 1),
        (d.x, d.y, d.length_squared, 1),
        ))
    return M.determinant() <= 0

class Triangulator:
    def add_debug_mesh(self, bm, name):
        pass

    def radial_sort_points(self, points):
        center = Vector((0, 0, 0))
        for pt in points:
            center += pt.co
        center /= len(points)
        points.sort(key = lambda pt: (pt.co - center).length_squared)

    # Construct a triangle mesh using the sweephull method.
    def do_sweephull(self, bm, points):
        if len(points) < 3:
            return

        self.radial_sort_points(points)

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

        # Add first 3 verts as the first triangle
        if is_ccw_winding(0, 1, 2):
            bm.faces.new((point_verts[0], point_verts[1], point_verts[2]))
            convex_hull = [0, 1, 2]
        else:
            bm.faces.new((point_verts[2], point_verts[1], point_verts[0]))
            convex_hull = [2, 1, 0]

        for point_index in range(3, len(points)):
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

            self.add_debug_mesh(bm, "SweepHull")

    def do_edgeflip(self, bm):
        import collections

        flipstack = collections.deque(maxlen=len(bm.edges))
        for edge in bm.edges:
            if not edge.is_boundary:
                flipstack.append(edge)
                edge.tag = True

        while flipstack:
            diag = flipstack.pop()
            diag.tag = False

            verts, edges = get_quad_verts(diag)
            if not is_delaunay(verts):
                bm.edges.remove(diag)
                bm.faces.new((verts[0], verts[1], verts[3]))
                bm.faces.new((verts[2], verts[3], verts[1]))

                for edge in edges:
                    if not edge.tag and not edge.is_boundary:
                        flipstack.append(edge)
                        edge.tag = True

                self.add_debug_mesh(bm, "EdgeFlip")

    def construct_delaunay(self, points):
        del_bm = bmesh.new()
        self.do_sweephull(del_bm, points)
        self.do_edgeflip(del_bm)
        return del_bm

    def construct_voronoi(self, del_bm, triangulate_cells=True):
        import collections

        voro_bm = bmesh.new()

        del_bm.faces.index_update()
        voro_verts = collections.deque(maxlen=len(del_bm.faces))

        # Add vertices for circumcenters
        for face in del_bm.faces:
            a = face.verts[0].co
            b = face.verts[1].co
            c = face.verts[2].co

            La = a.length_squared
            Lb = b.length_squared
            Lc = c.length_squared
            Sx = 0.5 * Matrix(((La, a.y, 1), (Lb, b.y, 1), (Lc, c.y, 1))).determinant()
            Sy = 0.5 * Matrix(((a.x, La, 1), (b.x, Lb, 1), (c.x, Lc, 1))).determinant()
            norm = Matrix(((a.x, a.y, 1), (b.x, b.y, 1), (c.x, c.y, 1))).determinant()
            r0 = Matrix(((a.x, a.y, La), (b.x, b.y, Lb), (c.x, c.y, Lc))).determinant()
            if norm != 0:
                co = Vector((Sx, Sy, 0)) / norm
                # r0 = math.sqrt(r0/norm + co.length_squared)

                voro_verts.append(voro_bm.verts.new(co))

        del_bm.verts.index_update() # TODO REMOVE
        voro_loop = collections.deque()

        for vert in del_bm.verts:
            if len(vert.link_loops) < 3:
                continue

            # Gather circumcenter vertices of adjacent faces
            voro_loop.clear()

            # Find the beginning of the loop fan, in case there is a boundary edge
            loop_start = vert.link_loops[0] # arbitrary start if there is no boundary edge
            loop_end = loop_start
            for loop in vert.link_loops:
                # Boundary edge means the loop points at itself in the radial list
                if loop.link_loop_radial_next == loop:
                    loop_start = loop

                # Loop on the next edge of the fan in ccw direction
                next_edge_loop = loop.link_loop_prev
                if next_edge_loop.link_loop_radial_next == next_edge_loop:
                    loop_end = next_edge_loop

            loop = loop_start
            while True:
                voro_loop.append(voro_verts[loop.face.index])

                # Loop on the next edge of the fan in ccw direction
                next_edge_loop = loop.link_loop_prev
                loop = next_edge_loop.link_loop_radial_next
                if loop == loop_end:
                    break

            voro_bm.faces.new(voro_loop)

            self.add_debug_mesh(voro_bm, "VoronoiMesh")

        return voro_bm
