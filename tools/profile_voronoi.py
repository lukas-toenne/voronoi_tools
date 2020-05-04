import bpy
import cProfile
import pstats
import io


pr = cProfile.Profile()
pr.enable()

bpy.ops.mesh.voronoi_add(use_vertex_sources=False, use_particle_sources=True, bounds_mode='REPEAT')

pr.disable()

s = io.StringIO()
ps = pstats.Stats(pr, stream=s).sort_stats("cumulative")
ps.print_stats()

print(s.getvalue())
