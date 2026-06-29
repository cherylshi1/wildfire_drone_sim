import json
import sys
from pathlib import Path

import bpy


DEFAULT_OUTPUT_DIR = (
    "/Users/rhyls/Desktop/Summer26 FSC/human trust w auto sys/real sim/models/fire_baked_mesh_frames"
)
DEFAULT_VDB_DIR = "/Users/rhyls/Downloads/vdb"
DEFAULT_FRAME_STEP = 8
DEFAULT_VOXEL_SIZE = 1.6
DEFAULT_THRESHOLD = 0.15
DEFAULT_ADAPTIVITY = 0.5
DEFAULT_SOURCE_FRAME_RATE = 24.0


def parse_args():
    argv = sys.argv
    if "--" in argv:
        argv = argv[argv.index("--") + 1 :]
    else:
        argv = sys.argv[1:]

    output_dir = Path(argv[0]) if len(argv) >= 1 else Path(DEFAULT_OUTPUT_DIR)
    vdb_dir = Path(argv[1]) if len(argv) >= 2 else Path(DEFAULT_VDB_DIR)
    frame_step = int(argv[2]) if len(argv) >= 3 else DEFAULT_FRAME_STEP
    return output_dir, vdb_dir, frame_step


def list_vdb_frames(vdb_dir):
    frame_paths = sorted(vdb_dir.glob("Frame_*.vdb"))
    if not frame_paths:
        raise FileNotFoundError(f"No VDB frames found in {vdb_dir}")
    return frame_paths


def clear_scene():
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    for mesh in list(bpy.data.meshes):
        if mesh.users == 0:
            bpy.data.meshes.remove(mesh)
    for node_group in list(bpy.data.node_groups):
        if node_group.users == 0:
            bpy.data.node_groups.remove(node_group)


def clear_existing_outputs(output_dir):
    output_dir.mkdir(parents=True, exist_ok=True)
    for path in output_dir.iterdir():
        if path.is_file():
            path.unlink()


def build_mesh_host(volume_object):
    bpy.ops.mesh.primitive_plane_add()
    host = bpy.context.active_object

    modifier = host.modifiers.new("VolumeToMesh", "NODES")
    node_group = bpy.data.node_groups.new("VolumeToMeshSequence", "GeometryNodeTree")
    modifier.node_group = node_group

    nodes = node_group.nodes
    links = node_group.links
    group_input = nodes.new("NodeGroupInput")
    group_output = nodes.new("NodeGroupOutput")
    node_group.interface.new_socket(
        name="Geometry",
        in_out="INPUT",
        socket_type="NodeSocketGeometry",
    )
    node_group.interface.new_socket(
        name="Geometry",
        in_out="OUTPUT",
        socket_type="NodeSocketGeometry",
    )

    object_info = nodes.new("GeometryNodeObjectInfo")
    object_info.transform_space = "RELATIVE"
    object_info.inputs["Object"].default_value = volume_object

    volume_to_mesh = nodes.new("GeometryNodeVolumeToMesh")
    volume_to_mesh.inputs["Resolution Mode"].default_value = "Size"
    volume_to_mesh.inputs["Voxel Size"].default_value = DEFAULT_VOXEL_SIZE
    volume_to_mesh.inputs["Threshold"].default_value = DEFAULT_THRESHOLD
    volume_to_mesh.inputs["Adaptivity"].default_value = DEFAULT_ADAPTIVITY

    links.new(object_info.outputs["Geometry"], volume_to_mesh.inputs["Volume"])
    links.new(volume_to_mesh.outputs["Mesh"], group_output.inputs[0])
    return host


def export_mesh_frame(frame_path, output_path):
    clear_scene()

    bpy.ops.object.volume_import(
        filepath=str(frame_path),
        directory=str(frame_path.parent),
        files=[{"name": frame_path.name}],
        relative_path=False,
        use_sequence_detection=False,
    )
    volume_object = next(obj for obj in bpy.data.objects if obj.type == "VOLUME")
    host = build_mesh_host(volume_object)

    depsgraph = bpy.context.evaluated_depsgraph_get()
    host_eval = host.evaluated_get(depsgraph)
    mesh = bpy.data.meshes.new_from_object(host_eval, depsgraph=depsgraph)
    export_object = bpy.data.objects.new("FireMeshExport", mesh)
    bpy.context.collection.objects.link(export_object)
    bpy.context.view_layer.objects.active = export_object

    for obj in bpy.context.selected_objects:
        obj.select_set(False)
    export_object.select_set(True)

    bpy.ops.wm.obj_export(
        filepath=str(output_path),
        export_selected_objects=True,
        export_uv=False,
        export_normals=True,
        export_materials=False,
        apply_modifiers=False,
    )

    polygon_count = len(mesh.polygons)
    vertex_count = len(mesh.vertices)
    bpy.data.meshes.remove(mesh)
    return vertex_count, polygon_count


def write_metadata(output_dir, frame_count, frame_step):
    metadata = {
        "frame_count": frame_count,
        "frame_rate": DEFAULT_SOURCE_FRAME_RATE / max(1, frame_step),
        "source_frame_step": frame_step,
        "voxel_size": DEFAULT_VOXEL_SIZE,
        "threshold": DEFAULT_THRESHOLD,
        "adaptivity": DEFAULT_ADAPTIVITY,
    }
    (output_dir / "metadata.json").write_text(json.dumps(metadata, indent=2))


def main():
    output_dir, vdb_dir, frame_step = parse_args()
    if frame_step <= 0:
        raise ValueError("frame_step must be >= 1")

    clear_existing_outputs(output_dir)
    frame_paths = list_vdb_frames(vdb_dir)[::frame_step]

    print(
        f"Baking {len(frame_paths)} mesh frames from {vdb_dir} to {output_dir}"
    )
    for rendered_index, frame_path in enumerate(frame_paths):
        output_path = output_dir / f"frame_{rendered_index:04d}.obj"
        vertex_count, polygon_count = export_mesh_frame(frame_path, output_path)
        if rendered_index % 5 == 0:
            print(
                f"Rendered {rendered_index + 1}/{len(frame_paths)}: "
                f"{output_path.name} verts={vertex_count} polys={polygon_count}"
            )

    write_metadata(output_dir, len(frame_paths), frame_step)
    print(
        f"Completed mesh bake: {len(frame_paths)} frames at "
        f"{DEFAULT_SOURCE_FRAME_RATE / frame_step:.2f} fps"
    )


if __name__ == "__main__":
    main()
