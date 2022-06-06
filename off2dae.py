# Slight modification of https://gist.github.com/AngryLoki/3800594
# Converts Object File Format (OFF) files to OBJ files in batch 
# Allows the usage of OFF files in Gazebo Ignition
# blender --background --python3.7 off2obj.py -- input_dir output_dir i.e. blender --background --python off2obj.py -- off_meshes/ ./obj_meshes/
# Tested with Blender 2.82 and OFF Add-on https://github.com/3Descape/blender-off-addon/tree/patch-1
# Sizes all objects down to 10,10,10
import os
import sys
import glob
import bpy
import time
from mathutils import Vector
import bmesh

scale = 0.01
if len(sys.argv) != 7:
    print("Must provide input and output path")
else:
    for infile in glob.glob(os.path.join(sys.argv[5], '*.off')):
        override = bpy.context.copy()
        override['selected_objects'] = list(bpy.context.scene.objects)
        bpy.ops.object.delete(override)
        obj = bpy.ops.import_mesh.off(filepath=infile)
        bpy.ops.transform.resize(value=(scale,scale,scale))
        bpy.context.view_layer.update()
        context = bpy.context
        ob = context.edit_object
        bpy.ops.object.select_all(action='TOGGLE')

        for ob in bpy.data.objects:
            
            x,y,z = ob.dimensions
            ob.location = (0,0,0)
            ob.scale = (10/x,10/y,10/z)
            bpy.context.view_layer.update()

            me = ob.data
            if me.is_editmode:
                bm = bmesh.from_edit_mesh(me)
            else:
                bm = bmesh.new()
                bm.from_mesh(me)

            f = bm.faces.active
            if f:
                o = f.calc_center_median()
                bmesh.ops.translate(bm,
                       verts = bm.verts,
                       vec = -o,
                       )
                if bm.is_wrapped:
                    bmesh.update_edit_mesh(me, False, False)
                else:
                    bm.to_mesh(me)
                    me.update()

                #bmesh.update_edit_mesh(me)
                #me.update()           
                # move the object globally to reflect
                mw = ob.matrix_world
                t = mw @ o - mw @ Vector()
                mw.translation += t
            
            print(ob.location)
            #ob.lock_location(True,True,True)
            bpy.context.view_layer.update()
            #ob.location = (0,0,0)
            #ob.rotation_euler = (0,0,0)

        outfilename = os.path.splitext(os.path.split(infile)[1])[0] + ".dae"
        bpy.ops.wm.collada_export(filepath=os.path.join(sys.argv[6], outfilename))
        time.sleep(0.1)