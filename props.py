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
from bpy.props import BoolProperty, EnumProperty, FloatProperty, FloatVectorProperty


output_uv_layers_items = {
    ('POLYGON', "Polygon", "Each polygon filling out UV space (-1..1)"),
    ('BOUNDS', "Bounds", "Position within the input bounds (Local space coordinates if bounds are disabled)"),
    ('CIRCUM_CIRCLE', "Circum Circle", "Position within the circumscribed circle of the triangle (Delaunay only)"),
    ('CELL_CENTERED', "Cell Centered", "Local space coordinates with cell center at origin (Voronoi only)"),
    ('EDGE_DISTANCE', "Edge Distance", "Local space distance from the cell edge (Voronoi only, needs triangulated cells)"),
}


output_data_layers_items = {
    ('POINT_INDEX', "Point Index", "Contiguous index of input point for vertex (Delaunay) or cell face (Voronoi)"),
}


def find_enum_name(items, identifier):
    for item in items:
        if item[0] == identifier:
            return item[1]


def find_enum_description(items, identifier):
    for item in items:
        if item[0] == identifier:
            return item[2]


class VoronoiToolProps():
    #
    # Input settings

    use_vertex_sources : BoolProperty(
        name="Use Mesh Sources",
        description="Use vertices from selected objects as input points",
        default=True,
        )

    use_particle_sources : BoolProperty(
        name="Use Particle Sources",
        description="Use particles from selected objects as input points",
        default=False,
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

    #
    # Output settings

    output_graph : EnumProperty(
        name="Output Graph",
        description="Type of graph to generate from point data",
        items={
            ('VORONOI', "Voronoi", "Divides space into cells around the closest point"),
            ('DELAUNAY', "Delaunay", "Triangulation with maximised angles"),
            },
        default='DELAUNAY',
        )

    output_uv_layers : EnumProperty(
        name="Output UV Layers",
        description="UV layers to generate in the output mesh",
        items=output_uv_layers_items,
        default={'POLYGON'},
        options={'ENUM_FLAG'},
        )

    output_data_layers : EnumProperty(
        name="Output Data Layers",
        description="Data layers to generate in the output mesh",
        items=output_data_layers_items,
        default={'POINT_INDEX'},
        options={'ENUM_FLAG'},
        )

    #
    # Delaunay specific settings

    #
    # Voronoi specific settings

    triangulate_cells : BoolProperty(
        name="Triangulate Cells",
        description="Create triangle fans for Voronoi cells instead of ngons",
        default=False,
        )
