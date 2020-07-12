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
import threading
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


class VoronoiToolsOperatorBase(VoronoiToolProps):
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

    def get_input_points(self, context):
        points = []
        if self.use_vertex_sources:
            points += get_points_from_meshes(context)
        if self.use_particle_sources:
            points += get_points_from_particles(context)

        self.project_points(points)
        self.apply_bounds(points)

        return points

    def get_triangulator(self):
        return Triangulator(
                uv_layers=self.output_uv_layers,
                triangulate_cells=self.triangulate_cells,
                bounds_min=None if self.bounds_mode == 'NONE' else self.bounds_min,
                bounds_max=None if self.bounds_mode == 'NONE' else self.bounds_max,
                )

    def generate_bmesh(self, triangulator, points):
        if self.output_graph == 'DELAUNAY':
            return triangulator.construct_delaunay(points)
        elif self.output_graph == 'VORONOI':
            return triangulator.construct_voronoi(points)


class AddVoronoiCells(VoronoiToolsOperatorBase, Operator):
    """Generate Voronoi cells on a surface."""
    bl_idname = 'mesh.voronoi_add'
    bl_label = 'Add Voronoi Cells'
    bl_options = {'UNDO', 'REGISTER'}

    @classmethod
    def poll(cls, context):
        active_object = context.active_object
        return active_object and active_object.type == 'MESH'

    def execute(self, context):
        active_object = context.active_object
        orig_mode = active_object.mode

        bpy.ops.object.mode_set(mode='OBJECT')
        try:
            points = self.get_input_points(context)
            with self.get_triangulator() as triangulator:
                obj = context.active_object
                triangulator.prepare_object(obj)
                bm = self.generate_bmesh(triangulator, points)
                bm.to_mesh(obj.data)
                obj.data.update()

        finally:
            bpy.ops.object.mode_set(mode=orig_mode)

        return {'FINISHED'}


class DebugTriangulator(VoronoiToolsOperatorBase, Operator):
    """Export Alebic cache of Triangulation and Voronoi steps."""
    bl_idname = 'voronoi_tools.debug_triangulator'
    bl_label = 'Debug Triangulator'
    bl_options = {'UNDO', 'REGISTER'}

    compute_frame_range : BoolProperty(
        name="Compute Frame Range",
        description="Perform a dry run to compute the full frame range needed for all debug steps",
        default=True,
        )

    @classmethod
    def poll(cls, context):
        active_object = context.active_object
        return active_object and active_object.type == 'MESH'

    def execute(self, context):
        active_object = context.active_object
        orig_mode = active_object.mode

        bpy.ops.object.mode_set(mode='OBJECT')
        try:
            points = self.get_input_points(context)
            frame_start, frame_end = self.get_frame_range(context, points)
            print("Debug output frame range: {}..{}".format(frame_start, frame_end))

            # Condition indicating that the frame has changed and the next debug step should be constructed.
            # The triangulator waits for this condition before each debug step.
            step_cond = threading.Condition()

            # Condition indicating that the debug mesh is ready for export.
            # The exporter frame handler waits for this condition after notifying the triangulator.
            export_cond = threading.Condition()

            # Start triangulator as a concurrent task.
            with self.get_triangulator() as triangulator:
                debug_obj = self.add_debug_object(context)
                # XXX Alembic cache bug: crashes when trying to re-use an existing cache file,
                # remove existing operator to avoid the problem.
                self.remove_mesh_sequence_cache(debug_obj)
                triangulator.prepare_object(debug_obj)

                thread = threading.Thread(target=self.triangulator_thread, name="TriangulatorThread", kwargs={
                    "step_cond":step_cond, "export_cond":export_cond, "triangulator":triangulator, "points":points, "debug_obj":debug_obj
                    })
                thread.start()

                export_filepath = bpy.path.abspath("//VoronoiToolsDebug.abc")

                self.run_exporter(
                    step_cond=step_cond,
                    export_cond=export_cond,
                    frame_start=frame_start,
                    frame_end=frame_end,
                    export_objects=[debug_obj],
                    export_filepath=export_filepath,
                    )

                if thread.is_alive():
                    self.cancel_triangulator(step_cond, triangulator)
                    # Cancel triangulator if not all frames are exported
                thread.join()
                print("[Exporter] Finished")

                self.create_mesh_sequence_cache(context, debug_obj, export_filepath)

        finally:
            bpy.ops.object.mode_set(mode=orig_mode)

        return {'FINISHED'}

    def get_frame_range(self, context, points):
        # Dry run to count the number of frames required
        if self.compute_frame_range:
            with self.get_triangulator() as triangulator:
                numsteps = 0
                def __count_debug_steps(bm, name):
                    nonlocal numsteps
                    numsteps += 1
                triangulator.add_debug_mesh = __count_debug_steps

                self.generate_bmesh(triangulator, points)
            return 1, numsteps
        else:
            return context.scene.frame_start, context.scene.frame_end

    # Run the exporter on the main thread
    def run_exporter(self, step_cond, export_cond, frame_start, frame_end, export_objects, export_filepath):
        # XXX Bug in Alembic exporter: will freeze if frame_start > frame_end
        if frame_start > frame_end:
            return

        # Frame change handler to wake up the triangulator and wait for the next debug mesh
        def frame_change_handler(scene):
            print("[Exporter] Frame Change: {}".format(scene.frame_current))
            # Wait for notification from the triangulator before exporting
            with step_cond:
                step_cond.notify_all()
            export_cond.wait()

        bpy.app.handlers.frame_change_post.append(frame_change_handler)

        # Hold lock in the main thread for the exporter
        with export_cond:
            # XXX Bug in Alembic exporter: context override has no effect, have to change object selection
            # override = bpy.context.copy()
            # override['selected_objects'] = export_objects
            scene = bpy.context.scene
            orig_selected = set(obj for obj in scene.objects if obj.select_get())
            for obj in scene.objects:
                obj.select_set(obj in export_objects)

            # Hack: Alembic exporter has to consider the object "animated" in order to export
            # topology on every frame. Simple way to do this is to add a dummy modifier other than subsurf,
            # which is considered mesh animation by the exporter.
            for obj in export_objects:
                self.create_dummy_animation_modifier(obj)

            bpy.ops.wm.alembic_export(
                filepath=export_filepath,
                check_existing=False,
                selected=True,
                start=frame_start,
                end=frame_end)

            # Clean up dummy modifiers
            for obj in export_objects:
                self.remove_dummy_animation_modifier(obj)
            # Restore original selection
            for obj in scene.objects:
                obj.select_set(obj in orig_selected)

        bpy.app.handlers.frame_change_post.remove(frame_change_handler)

    def triangulator_thread(self, step_cond, export_cond, triangulator, points, debug_obj):
        # Debug steps will be skipped when this flag becomes False.
        # Condition lock must be used when reading or writing this flag!
        triangulator.debug_running = True

        # Triangulator debug handler will wait until frame change before updating mesh.
        # Note that the worker thread holds a general lock on the condition, released temporarily while waiting.
        def add_debug_mesh(bm, name):
            if triangulator.debug_running:
                print("[Triangulator] Debug Mesh: {}".format(name))
                # Make a copy to avoid modifying the intermediate mesh
                debug_bm = bm.copy()

                if name == "SweepHull":
                    triangulator._finalize_faces(debug_bm, graph_type='DELAUNAY')
                elif name == "EdgeFlip":
                    triangulator._finalize_faces(debug_bm, graph_type='DELAUNAY')
                elif name == "VoronoiMesh":
                    triangulator._finalize_faces(debug_bm, graph_type='VORONOI')
                else:
                    raise Exception("Unknown debug name {}, cannot determine graph type".format(name))

                # Store current bmesh state in the mesh datablock
                debug_bm.to_mesh(debug_obj.data)
                debug_bm.free()
                debug_obj.data.update()

                # Notify the exporter that the mesh is ready
                with export_cond:
                    export_cond.notify_all()
                # Wait for the next frame
                step_cond.wait()
        triangulator.add_debug_mesh = add_debug_mesh

        # Hold lock in the worker thread
        with step_cond:
            # Wait for the initial frame change before updating the mesh
            step_cond.wait()

            if triangulator.debug_running:
                print("[Triangulator] Starting triangulator ...")
                # triangulator.prepare_object(obj)
                bm = self.generate_bmesh(triangulator, points)
                # bm.to_mesh(obj.data)
                # obj.data.update()

            while triangulator.debug_running:
                print("[Triangulator] skipped frame, debug_running={}".format(triangulator.debug_running))
                with export_cond:
                    export_cond.notify_all()
                step_cond.wait()

    def cancel_triangulator(self, step_cond, triangulator):
        with step_cond:
            triangulator.debug_running = False
            step_cond.notify_all()

    """
    Add a object to store intermediate meshes and display the final mesh sequence.
    """
    def add_debug_object(self, context):
        obj = context.active_object
        scene = context.scene

        # Mesh datablock
        debug_mesh = bpy.data.meshes.get("VoronoiToolsDebug")
        if debug_mesh is None:
            debug_mesh = bpy.data.meshes.new("VoronoiToolsDebug")
        # Assign material
        mat = bpy.data.materials.get("VoronoiToolsDebug")
        if mat is None:
            mat = bpy.data.materials.new("VoronoiToolsDebug")
        debug_mesh.materials.append(mat)
        debug_mesh.update()

        # Object to link the mesh in the scene
        debug_obj = bpy.data.objects.get("VoronoiToolsDebug")
        if debug_obj is None:
            debug_obj = bpy.data.objects.new("VoronoiToolsDebug", debug_mesh)
        # Copy transform and other attributes from the main object
        debug_obj.location = obj.location
        debug_obj.rotation_euler = obj.rotation_euler
        debug_obj.rotation_quaternion = obj.rotation_quaternion
        debug_obj.rotation_axis_angle = obj.rotation_axis_angle
        debug_obj.scale = obj.scale
        # Useful visualization settings for debugging
        debug_obj.show_wire = True
        debug_obj.show_all_edges = True

        # Put debug object into the scene collection
        if debug_obj.name not in scene.collection.objects:
            scene.collection.objects.link(debug_obj)

        return debug_obj

    """
    Add a dummy modifier to force the Alembic exporter to consider the mesh "animated".
    """
    def create_dummy_animation_modifier(self, obj):
        mod = obj.modifiers.get("DummyAnimationModifier")
        if mod is None:
            mod = obj.modifiers.new("DummyAnimationModifier", 'DISPLACE')
            mod.strength = 0 # modifier has no effect, we only need it to trick the exporter

    """
    Remove the dummy modifier.
    """
    def remove_dummy_animation_modifier(self, obj):
        mod = obj.modifiers.get("DummyAnimationModifier")
        if mod is not None:
            obj.modifiers.remove(mod)

    """
    Add a mesh sequence cache modifier to re-import the Alembic data.
    """
    def create_mesh_sequence_cache(self, context, obj, export_filepath):
        import os

        mod = obj.modifiers.get("MeshSequenceCache")
        if mod is None:
            mod = obj.modifiers.new("MeshSequenceCache", 'MESH_SEQUENCE_CACHE')

        cache_name = os.path.basename(export_filepath)
        mod.cache_file = bpy.data.cache_files.get(cache_name)
        # XXX Cache file bug: running the operator to load an existing cache file will crash,
        # instead have to re-use and reload the existing cache file ID block.
        if mod.cache_file is None:
            self.report({'INFO'}, "Loading Alembic cache file {}".format(export_filepath))
            overrides = context.copy()
            overrides['modifier'] = mod 
            bpy.ops.cachefile.open(overrides, filepath=export_filepath)
            mod.cache_file = bpy.data.cache_files.get(cache_name)
        else:
            self.report({'INFO'}, "Reloading Alembic cache file {}".format(mod.cache_file.filepath))
            overrides = context.copy()
            overrides['modifier'] = mod 
            bpy.ops.cachefile.reload(overrides)

        object_path = "/{}/{}Shape".format(obj.name, obj.name)
        if object_path in mod.cache_file.object_paths:
            mod.object_path = object_path
        else:
            self.report({'WARNING'}, "Object path {} not found in cache file".format(object_path))

        mod.read_data = {'VERT', 'POLY', 'UV', 'COLOR'}

    """
    Remove the mesh sequence cache modifier.
    """
    def remove_mesh_sequence_cache(self, obj):
        mod = obj.modifiers.get("MeshSequenceCache")
        if mod is not None:
            obj.modifiers.remove(mod)


def register():
    bpy.utils.register_class(AddVoronoiCells)
    bpy.utils.register_class(DebugTriangulator)

def unregister():
    bpy.utils.unregister_class(AddVoronoiCells)
    bpy.utils.unregister_class(DebugTriangulator)
