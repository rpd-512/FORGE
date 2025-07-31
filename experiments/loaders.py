import yaml
import json
import algorithm_modules_cpp as forge

def load_dh_params(file):
    dh_param_list = []
    with open(file, 'r') as f:
        for d in yaml.safe_load(f)['dh_parameters']:
            dh_param_list.append(forge.dh_param(
                d['a'],
                d['d'],
                d['alpha']
            ))
    return dh_param_list

def load_scene(filename):
    scene_objects = []

    with open(filename, 'r') as f:
        data = json.load(f)
    for obj in data:
        if "type" not in obj or "position" not in obj:
            print("Invalid object in JSON (missing type or position).")
            continue

        obj_type = obj["type"]
        pos = obj["position"]
        base_pos = forge.position3D(pos[0], pos[1], pos[2])

        scene = forge.SceneObject()
        scene.type = obj_type
        data_struct = forge.SceneObjectData()

        if obj_type == "cube" and "size" in obj:
            size = obj["size"]
            box = forge.BoxObject()
            box.min_corner = base_pos
            box.max_corner = forge.position3D(
                base_pos.x + size[0],
                base_pos.y + size[1],
                base_pos.z + size[2]
            )
            data_struct.box = box
        elif obj_type == "cylinder" and "radius" in obj and "height" in obj:
            cyl = forge.CylinderObject()
            cyl.base_center = base_pos
            cyl.radius = obj["radius"]
            cyl.height = obj["height"]
            data_struct.cylinder = cyl
        elif obj_type == "sphere" and "radius" in obj:
            sph = forge.SphereObject()
            sph.center = base_pos
            sph.radius = obj["radius"]
            data_struct.sphere = sph
        else:
            print(f"Unknown or incomplete object type: {obj_type}")
            continue

        scene.data = data_struct
        scene_objects.append(scene)
    return scene_objects
