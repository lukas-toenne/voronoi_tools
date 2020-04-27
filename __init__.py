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

bl_info = {
    'name': 'Voronoi Tools',
    'author': 'Lukas Toenne',
    'version': (1, 0, 0),
    'blender': (2, 82, 0),
    'location': '3D View',
    'description':
        'Tools for generating Voronoi pattern meshes',
    "wiki_url": "",
    'category': 'Add Mesh',
    'support': 'COMMUNITY',
}

# Runtime script reload
if "bpy" in locals():
    import importlib

    from . import operator, props, triangulator
    importlib.reload(props)
    importlib.reload(triangulator)
    importlib.reload(operator)

import bpy
from . import operator


def register():
    operator.register()


def unregister():
    operator.unregister()


if __name__ == '__main__':
    register()
