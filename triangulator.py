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
    """Location of the point in object space."""
    co : Vector((0, 0, 0))

    """
    One of:
    - ORIGINAL: Main vertex from the original point set
    - REPEAT_POS: Point copy shifted in positive direction
    - REPEAT_NEG: Point copy shifted in negative direction
    
    Only edges between original points and original and positive points
    will be kept, all other are removed.
    """
    type : 'ORIGINAL'

    def __init__(self, co, type):
        self.co = co
        self.type = type

degenerate_epsilon = 1.0e-9

def is_degenerate_triangle(co0, co1, co2):
    return abs((co1 - co0).cross(co2 - co0).z) < degenerate_epsilon

"""True if the direction of edges between the 3 points is turning counter-clockwise."""
def is_ccw_safe(co0, co1, co2):
    return (co1 - co0).cross(co2 - co0).z > degenerate_epsilon

"""True if the direction of edges between the 3 points is turning clockwise."""
def is_cw_safe(co0, co1, co2):
    return (co1 - co0).cross(co2 - co0).z < -degenerate_epsilon

# Get the adjacent and opposing vertices for an edge diagonal.
# Returns vertices (a, b, c, d), where (a, c) is the edge and b, d are opposing vertices,
# as well as the associated edges (in CCW order).
def get_quad_verts(edge):
    # Only valid for internal edges with two adjacent faces
    assert(len(edge.link_loops) == 2)

    # Edge verts order does not follow winding order!
    # Loops have to be used for getting vertices in the correct order.
    # The start of the vertex list is arbitrary though, so can just pick the first loop as starting point.

    loop = edge.link_loops[0]
    assert(loop.link_loop_next.vert == edge.link_loops[1].vert)
    va = loop.vert
    vd = loop.link_loop_prev.vert
    ed = loop.link_loop_prev.edge
    ec = loop.link_loop_prev.link_loop_prev.edge

    loop = edge.link_loops[1]
    assert(loop.link_loop_next.vert == edge.link_loops[0].vert)
    vc = loop.vert
    vb = loop.link_loop_prev.vert
    eb = loop.link_loop_prev.edge
    ea = loop.link_loop_prev.link_loop_prev.edge

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
    def prepare_object(self, obj):
        def ensure_vgroup(name):
            vg = obj.vertex_groups.get(name)
            if vg is None:
                vg = obj.vertex_groups.new(name=name)
            return vg

        # Create vertex groups for point type
        self.vg_original = ensure_vgroup("OriginalPoints").index
        self.vg_repeat_pos = ensure_vgroup("RepeatedPointsPos").index
        self.vg_repeat_neg = ensure_vgroup("RepeatedPointsNeg").index

    def add_debug_mesh(self, bm, name):
        pass

    def radial_sort_points(self, points):
        center = Vector((0, 0, 0))
        for pt in points:
            center += pt.co
        center /= len(points)
        points.sort(key = lambda pt: (pt.co - center).length_squared)

    def add_point_vertices(self, bm, points):
        if len(points) < 3:
            return

        self.radial_sort_points(points)

        dvert_lay = bm.verts.layers.deform.verify()

        # Create vertices for all points
        for pt in points:
            vert = bm.verts.new(pt.co)
            dvert = vert[dvert_lay]

            if pt.type == 'ORIGINAL':
                dvert[self.vg_original] = 1.0
            if pt.type == 'REPEAT_POS':
                dvert[self.vg_repeat_pos] = 1.0
            if pt.type == 'REPEAT_NEG':
                dvert[self.vg_repeat_neg] = 1.0

    # Construct a triangle mesh using the sweephull method.
    def do_sweephull(self, bm, points):
        if len(points) < 3:
            return

        bm.verts.ensure_lookup_table()

        def is_ccw_safe_points(i, j, k):
            return is_ccw_safe(points[i].co, points[j].co, points[k].co)

        def is_cw_safe_points(i, j, k):
            return is_cw_safe(points[i].co, points[j].co, points[k].co)

        # Add first 3 verts as the first triangle
        start_index = None
        for i in range(len(points) - 2):
            j = i + 1
            k = i + 2
            if is_ccw_safe_points(i, j, k):
                bm.faces.new((bm.verts[i], bm.verts[j], bm.verts[k]))
                convex_hull = [i, j, k]
                start_index = i + 3
                break
            elif is_ccw_safe_points(k, j, i):
                bm.faces.new((bm.verts[k], bm.verts[j], bm.verts[i]))
                convex_hull = [k, j, i]
                start_index = i + 3
                break

        for point_index in range(start_index, len(points)):
            new_convex_hull = []

            # Returns true if the point is on the outside of the convex hull edge (ci, cj).
            # Indices are in the convex_hull array!
            def is_edge_visible(ci, cj):
                return is_cw_safe_points(point_index, convex_hull[ci], convex_hull[cj])

            was_visible = is_edge_visible(-1, 0) # visibility of last edge
            for c in range(len(convex_hull)):
                next_c = (c + 1) % len(convex_hull)
                # Is the convex hull edge visible from the new point?
                is_visible = is_edge_visible(c, next_c)

                hull_index = convex_hull[c]
                hull_index_next = convex_hull[next_c]

                # Connect to visible edges
                if is_visible:
                    bm.faces.new((bm.verts[point_index], bm.verts[hull_index_next], bm.verts[hull_index]))

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

            # Flipping degenerate triangles can cause overlap
            if is_degenerate_triangle(verts[0].co, verts[2].co, verts[3].co) or is_degenerate_triangle(verts[2].co, verts[0].co, verts[1].co):
                continue

            if not is_delaunay(verts):
                bm.edges.remove(diag)
                bm.faces.new((verts[0], verts[1], verts[3]))
                bm.faces.new((verts[2], verts[3], verts[1]))

                for edge in edges:
                    if not edge.tag and not edge.is_boundary:
                        flipstack.append(edge)
                        edge.tag = True

                self.add_debug_mesh(bm, "EdgeFlip")

    """
    Find and delete duplicate faces in the Delaunay triangulation.
    Faces are duplicate if any vertex is in the negative repetition or all vertices are in the position repetition.
    """
    def prune_duplicate_faces(self, bm, points):
        bm.verts.index_update()

        duplicate_faces = []
        for face in bm.faces:
            has_original = False
            has_repeat_neg = False

            for vert in face.verts:
                point = points[vert.index]
                if point.type == 'ORIGINAL':
                    has_original = True
                if point.type == 'REPEAT_NEG':
                    has_repeat_neg = True
            if (not has_original) or has_repeat_neg:
                duplicate_faces.append(face)

        bmesh.ops.delete(bm, geom=duplicate_faces, context='FACES')

    def construct_delaunay(self, points, prune):
        del_bm = bmesh.new()

        self.add_point_vertices(del_bm, points)
        self.do_sweephull(del_bm, points)
        self.do_edgeflip(del_bm)

        if prune:
            self.prune_duplicate_faces(del_bm, points)

        return del_bm

    def construct_voronoi(self, points, del_bm, triangulate_cells=True):
        import collections

        voro_bm = bmesh.new()

        del_bm.verts.index_update()
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
            else:
                voro_verts.append(None)

        voro_loop = collections.deque()

        for vert in del_bm.verts:
            # Avoid duplicate faces
            point = points[vert.index]
            if point.type != 'ORIGINAL':
                continue

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
                center_vert = voro_verts[loop.face.index]
                if center_vert:
                    voro_loop.append(center_vert)

                # Loop on the next edge of the fan in ccw direction
                next_edge_loop = loop.link_loop_prev
                loop = next_edge_loop.link_loop_radial_next
                if loop == loop_end:
                    break

            # Can still get a loop with <3 verts in corner cases (colinear vertices)
            if len(voro_loop) >= 3:
                voro_bm.faces.new(voro_loop)

            self.add_debug_mesh(voro_bm, "VoronoiMesh")

        return voro_bm
