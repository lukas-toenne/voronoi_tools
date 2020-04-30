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
from mathutils import Vector
from .props import VoronoiToolProps
from .triangulator import Triangulator, InputPoint


def get_points_from_meshes(context):
    obj = context.active_object
    depsgraph = context.evaluated_depsgraph_get()

    matinv = obj.matrix_world.inverted()

    points = []
    for source_obj in context.selected_objects:
        if source_obj == obj:
            continue

        source_eval = source_obj.evaluated_get(depsgraph)
        mat = matinv @ source_eval.matrix_world

        mesh_from_eval = source_eval.to_mesh()
        if mesh_from_eval:
            points.extend([InputPoint(i, mat @ v.co, 'ORIGINAL') for i, v in enumerate(mesh_from_eval.vertices)])

    return points


def get_points_from_particles(context):
    obj = context.active_object
    depsgraph = context.evaluated_depsgraph_get()

    matinv = obj.matrix_world.inverted()

    points = []
    for source_obj in context.selected_objects:
        if source_obj == obj:
            continue

        source_eval = source_obj.evaluated_get(depsgraph)
        # Particles are stored in world space, no need to apply the emitter object matrix
        mat = matinv

        for psys in source_eval.particle_systems:
            points.extend([InputPoint(i, mat @ p.location, 'ORIGINAL') for i, p in enumerate(psys.particles)])

    return points


class AddVoronoiCells(VoronoiToolProps, Operator):
    """Generate Voronoi cells on a surface."""
    bl_idname = 'mesh.voronoi_add'
    bl_label = 'Add Voronoi Cells'
    bl_options = {'UNDO', 'REGISTER'}

    #
    # Debug settings

    generate_debug_meshes : BoolProperty(
        name="Generate Debug Meshes",
        description="Generate a collection of debug meshes for each step",
        default=False,
        )

    ################

    @classmethod
    def poll(cls, context):
        active_object = context.active_object
        return active_object and active_object.type == 'MESH'

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

    def project_points(self, points):
        for i in range(len(points)):
            points[i].co.z = 0.0

    def apply_bounds(self, points):
        if self.bounds_mode != 'NONE':
            points[:] = [p for p in points if p.co.x >= self.bounds_min[0] and p.co.x <= self.bounds_max[0] and p.co.y >= self.bounds_min[1] and p.co.y <= self.bounds_max[1]]

        if self.bounds_mode == 'REPEAT':
            def offset_points(offset_x, offset_y):
                dx = self.bounds_max[0] - self.bounds_min[0]
                dy = self.bounds_max[1] - self.bounds_min[1]
                type = 'ORIGINAL'
                if offset_x < 0:
                    type = 'REPEAT_NEG'
                elif offset_x > 0:
                    type = 'REPEAT_POS'
                elif offset_y < 0:
                    type = 'REPEAT_NEG'
                elif offset_y > 0:
                    type = 'REPEAT_POS'
                return [InputPoint(p.id, p.co + Vector((offset_x * dx, offset_y * dy, 0)), type) for p in points]
            points[:] = offset_points(-1, -1) + offset_points(0, -1) + offset_points(1, -1) + offset_points(-1, 0) + offset_points(0, 0) + offset_points(1, 0) + offset_points(-1, 1) + offset_points(0, 1) + offset_points(1, 1)

    def generate_mesh(self, context):
        obj = context.active_object

        points = []
        if self.use_vertex_sources:
            points += get_points_from_meshes(context)
        if self.use_particle_sources:
            points += get_points_from_particles(context)

        self.project_points(points)
        self.apply_bounds(points)

        with Triangulator(
                uv_layers=self.output_uv_layers,
                triangulate_cells=self.triangulate_cells,
                bounds_min=None if self.bounds_mode == 'NONE' else self.bounds_min,
                bounds_max=None if self.bounds_mode == 'NONE' else self.bounds_max,
                ) as triangulator:
            triangulator.prepare_object(obj)

            if self.generate_debug_meshes:
                self.setup_debugging(context, obj, triangulator)

            if self.output_graph == 'DELAUNAY':
                triangulator.construct_delaunay(points, prune=True)
                triangulator.triangulation_bm.to_mesh(obj.data)

            elif self.output_graph == 'VORONOI':
                triangulator.construct_delaunay(points, prune=False)
                triangulator.construct_voronoi()
                triangulator.voronoi_bm.to_mesh(obj.data)

        obj.data.update()

        return True

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
            # Make a copy to avoid modifying the intermediate mesh
            debug_bm = bm.copy()

            if name == "SweepHull":
                triangulator._add_data_layers(debug_bm, graph_type='DELAUNAY')
            elif name == "EdgeFlip":
                triangulator._add_data_layers(debug_bm, graph_type='DELAUNAY')
            elif name == "VoronoiMesh":
                triangulator._add_data_layers(debug_bm, graph_type='VORONOI')
            else:
                raise Exception("Unknown debug name {}, cannot determine graph type".format(name))

            # Store current bmesh state in a new mesh datablock
            debug_mesh = bpy.data.meshes.new("VoronoiToolsDebug.{}".format(name))
            debug_bm.to_mesh(debug_mesh)
            debug_bm.free()
            # Assign material
            mat = bpy.data.materials.get("VoronoiToolsDebug.{}".format(name))
            if mat is None:
                mat = bpy.data.materials.new("VoronoiToolsDebug.{}".format(name))
            debug_mesh.materials.append(mat)
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

            debug_obj.hide_render = True
            debug_obj.keyframe_insert("hide_render", frame=(frame - 1))
            debug_obj.hide_render = False
            debug_obj.keyframe_insert("hide_render", frame=(frame))
            debug_obj.hide_render = True
            debug_obj.keyframe_insert("hide_render", frame=(frame + 1))
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
