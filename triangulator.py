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
from sys import float_info
from . import props

"""
Input point for the triangulator.
"""
class InputPoint:
    """
    Location of the point in object space.
    """
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

"""
True if vertices are colinear or coinciding within the epsilon value.
"""
def is_degenerate_triangle(co0, co1, co2):
    return abs((co1 - co0).cross(co2 - co0).z) < degenerate_epsilon

"""
True if the direction of edges between the 3 points is turning counter-clockwise.
"""
def is_ccw_safe(co0, co1, co2):
    return (co1 - co0).cross(co2 - co0).z > degenerate_epsilon

"""
True if the direction of edges between the 3 points is turning clockwise.
"""
def is_cw_safe(co0, co1, co2):
    return (co1 - co0).cross(co2 - co0).z < -degenerate_epsilon

"""
Get the adjacent and opposing vertices for an edge diagonal.
Returns vertices (a, b, c, d), where (a, c) is the edge and b, d are opposing vertices,
as well as the associated edges (in CCW order).
"""
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

"""
Returns True if the Delaunay condition is satisfied:
Each vertex is outside the circumcircle for the other three.
Vertices must form a non-overlapping polygon (can be concave).
"""
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

"""
Utility class for generating a triangle mesh that satisfies the Delaunay condition.
Voronoi diagrams can be constructed from the Delaunay mesh.
"""
class Triangulator:
    """
    UV Layers to generate in the output mesh.
    """
    uv_layers = set()

    """
    If True then Voronoi cells will be created with a triangle fan instead of ngons.
    """
    triangulate_cells = False

    def __init__(self, uv_layers=set(), data_layers=set(), triangulate_cells=False, bounds_min=None, bounds_max=None):
        self.uv_layers = uv_layers
        self.data_layers = data_layers
        self.triangulate_cells = triangulate_cells
        self.bounds_min = bounds_min
        self.bounds_max = bounds_max

    """
    Set up object ID block data, such as vertex groups.
    This must be done so internal bmesh data layers are preserved in the Mesh data block.
    """
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

    """
    Sort the points list by distance from the center in preparation for the sweephull algorithm.
    """
    def radial_sort_points(self, points):
        center = Vector((0, 0, 0))
        for pt in points:
            center += pt.co
        center /= len(points)
        points.sort(key = lambda pt: (pt.co - center).length_squared)

    """
    Add vertices to the Delaunay triangulation mesh based on input points.
    """
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

    """
    Construct a triangle mesh using the sweephull method.
    The resulting mesh is non-overlapping, but does not satisfy the Delaunay condition yet.
    """
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

    """
    Iteratively apply the edge flipping method to ensure the Delaunay condition is satisfied for each edge.
    The input mesh must be non-overlapping.
    """
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

    def add_data_layers(self, bm, points):
        loop_layer_map = dict()
        for uv_layer_id in self.uv_layers:
            uv_layer_name = props.find_enum_name(props.output_uv_layers_items, uv_layer_id)
            loop_layer_map[uv_layer_id] = bm.loops.layers.uv.new(uv_layer_name)
        for data_layer_id in self.data_layers:
            data_layer_name = props.find_enum_name(props.output_data_layers_items, data_layer_id)
            if (data_layer_id == 'POINT_INDEX'):
                loop_layer_map[data_layer_id] = bm.loops.layers.uv.new(data_layer_name)

        use_minmax = ('POLYGON' in loop_layer_map)

        if self.bounds_min is None or self.bounds_max is None:
            bounds_loc = Vector((0, 0))
            bounds_mat = Matrix(((1, 0), (0, 1)))
        else:
            extent = Vector(self.bounds_max[:]) - Vector(self.bounds_min[:])
            bounds_loc = Vector(self.bounds_min[:])
            bounds_mat = Matrix(((1.0/extent.x if extent.x > degenerate_epsilon else 1, 0), (0, 1.0/extent.y if extent.y > degenerate_epsilon else 1)))

        for face in bm.faces:
            if use_minmax:
                # Determine bounds of the polygon
                xmin = float_info.max
                ymin = float_info.max
                xmax = float_info.min
                ymax = float_info.min
                for vert in face.verts:
                    xmin = min(xmin, vert.co.x)
                    xmax = max(xmax, vert.co.x)
                    ymin = min(ymin, vert.co.y)
                    ymax = max(ymax, vert.co.y)
                scale_x = 1.0/(xmax - xmin) if (xmax - xmin) > degenerate_epsilon else 1.0
                scale_y = 1.0/(ymax - ymin) if (ymax - ymin) > degenerate_epsilon else 1.0
                polygon_loc = Vector((xmin, ymin))
                polygon_mat = Matrix(((scale_x, 0), (0, scale_y)))

            for layer_id, layer in loop_layer_map.items():
                if layer_id == 'POLYGON':
                    for loop in face.loops:
                        loop[layer].uv = polygon_mat @ (loop.vert.co.xy - polygon_loc)
                elif layer_id == 'BOUNDS':
                    for loop in face.loops:
                        loop[layer].uv = bounds_mat @ (loop.vert.co.xy - bounds_loc)
                elif layer_id == 'CIRCUM_CIRCLE':
                    for loop in face.loops:
                        loop[layer].uv = bounds_mat @ (loop.vert.co.xy - bounds_loc)
                elif layer_id == 'POINT_INDEX':
                    for loop in face.loops:
                        loop[layer].uv = Vector((, 0))

    """
    Constructs a triangle mesh that satisfies the Delaunay condition based on the input points.
    If prune is True, redundant mesh elements resulting from repetition will be removed.
    """
    def construct_delaunay(self, points, prune):
        del_bm = bmesh.new()

        self.add_point_vertices(del_bm, points)
        self.do_sweephull(del_bm, points)
        self.do_edgeflip(del_bm)

        if prune:
            self.prune_duplicate_faces(del_bm, points)

        self.add_data_layers(del_bm, points)

        return del_bm


    """
    Add vertices for circumcenters of the Delaunay triangles.
    Returns a list with a vertex for each face (can be None if the face is degenerate).
    """
    def create_cell_center_verts(self, del_bm, voro_bm):
        center_verts = collections.deque(maxlen=len(del_bm.faces))

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

                center_verts.append(voro_bm.verts.new(co))
            else:
                center_verts.append(None)

        return center_verts

    """
    Creates a face for each cell of the Voronoi graph.
    """
    def create_cell_faces(self, del_bm, voro_bm, points, center_verts):
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
                center_vert = center_verts[loop.face.index]
                if center_vert:
                    voro_loop.append(center_vert)

                # Loop on the next edge of the fan in ccw direction
                next_edge_loop = loop.link_loop_prev
                loop = next_edge_loop.link_loop_radial_next
                if loop == loop_end:
                    break

            # Can still get a loop with <3 verts in corner cases (colinear vertices)
            if len(voro_loop) >= 3:
                if self.triangulate_cells:
                    center_vert = voro_bm.verts.new(vert.co, vert)
                    for i in range(len(voro_loop) - 1):
                        voro_bm.faces.new((voro_loop[i], voro_loop[i + 1], center_vert))
                    voro_bm.faces.new((voro_loop[len(voro_loop) - 1], voro_loop[0], center_vert))
                else:
                    voro_bm.faces.new(voro_loop)

            self.add_debug_mesh(voro_bm, "VoronoiMesh")

    """
    Constructs a Voronoi mesh based on input points a triangulated mesh.
    The del_bm mesh must be a valid Delaunay triangulation.
    """
    def construct_voronoi(self, points, del_bm):
        import collections

        del_bm.verts.index_update()
        del_bm.faces.index_update()

        voro_bm = bmesh.new()
        center_verts = self.create_cell_center_verts(del_bm, voro_bm)
        self.create_cell_faces(del_bm, voro_bm, points, center_verts)

        self.add_data_layers(voro_bm, points)

        return voro_bm

    """
    Debug function for recording intermediate mesh results.
    Default implementation is a dummy, must be replaced externally.
    """
    def add_debug_mesh(self, bm, name):
        pass
