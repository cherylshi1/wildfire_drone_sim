import json
import math
import struct
import sys
import zlib
from pathlib import Path

import numpy as np

try:
    import openvdb
except ModuleNotFoundError as exc:  # pragma: no cover - runtime guard
    raise SystemExit(
        "openvdb is unavailable. Run this script with Blender's bundled Python."
    ) from exc


DEFAULT_OUTPUT_DIR = (
    "/Users/rhyls/Desktop/Summer26 FSC/human trust w auto sys/real sim/models/fire_baked_frames"
)
DEFAULT_VDB_DIR = "/Users/rhyls/Downloads/vdb"
DEFAULT_FRAME_STEP = 1
DEFAULT_RESOLUTION = 256
DEFAULT_FRAME_RATE = 24.0
DEFAULT_PROJECTION_AXIS = 1
DEFAULT_VERTICAL_STRETCH = 2.35


def parse_args():
    argv = sys.argv[1:]
    output_dir = Path(argv[0]) if len(argv) >= 1 else Path(DEFAULT_OUTPUT_DIR)
    vdb_dir = Path(argv[1]) if len(argv) >= 2 else Path(DEFAULT_VDB_DIR)
    frame_step = int(argv[2]) if len(argv) >= 3 else DEFAULT_FRAME_STEP
    resolution = int(argv[3]) if len(argv) >= 4 else DEFAULT_RESOLUTION
    return output_dir, vdb_dir, frame_step, resolution


def list_vdb_frames(vdb_dir):
    frame_paths = sorted(vdb_dir.glob("Frame_*.vdb"))
    if not frame_paths:
        raise FileNotFoundError(f"No VDB frames found in {vdb_dir}")
    return frame_paths


def load_flame_grid(frame_path):
    return openvdb.read(str(frame_path), "flames")


def grid_bbox_to_lists(bbox):
    return [int(value) for value in bbox[0]], [int(value) for value in bbox[1]]


def collect_global_bbox(frame_paths):
    global_min = None
    global_max = None

    for frame_path in frame_paths:
        grid = load_flame_grid(frame_path)
        bbox = grid.evalActiveVoxelBoundingBox()
        bbox_min, bbox_max = grid_bbox_to_lists(bbox)
        if global_min is None:
            global_min = bbox_min
            global_max = bbox_max
            continue
        global_min = [min(a, b) for a, b in zip(global_min, bbox_min)]
        global_max = [max(a, b) for a, b in zip(global_max, bbox_max)]

    if global_min is None or global_max is None:
        raise RuntimeError("Unable to determine a global VDB bounding box")

    global_shape = tuple((hi - lo) + 1 for lo, hi in zip(global_min, global_max))
    return tuple(global_min), tuple(global_max), global_shape


def volume_to_array(grid, global_min, global_shape):
    volume = np.zeros(global_shape, dtype=np.float32)
    grid.copyToArray(volume, ijk=global_min)
    return volume


def soften_image(image):
    padded = np.pad(image, ((1, 1), (1, 1)), mode="edge")
    softened = (
        padded[1:-1, 1:-1] * 0.44
        + (padded[:-2, 1:-1] + padded[2:, 1:-1]) * 0.14
        + (padded[1:-1, :-2] + padded[1:-1, 2:]) * 0.14
        + (
            padded[:-2, :-2]
            + padded[:-2, 2:]
            + padded[2:, :-2]
            + padded[2:, 2:]
        )
        * 0.035
    )
    return softened.astype(np.float32, copy=False)


def project_heat(volume):
    clipped = np.clip(volume, 0.0, None)
    peak = clipped.max(axis=DEFAULT_PROJECTION_AXIS)
    integrated = np.power(clipped, 0.9).sum(axis=DEFAULT_PROJECTION_AXIS)
    peak_gate = np.clip((peak - 0.05) / 0.24, 0.0, 1.0)
    heat = (
        np.power(peak, 0.74) * 6.2
        + np.power(np.maximum(integrated, 0.0), 0.62) * 0.32
    ) * peak_gate
    heat = np.transpose(heat, (1, 0))[::-1, :]
    heat = soften_image(heat)
    return heat


def build_analysis(frames, global_min, global_shape):
    crop_min_x = None
    crop_min_y = None
    crop_max_x = None
    crop_max_y = None
    percentile_samples = []

    for frame_path in frames:
        heat = project_heat(volume_to_array(load_flame_grid(frame_path), global_min, global_shape))
        nonzero = heat[heat > 0.0]
        if nonzero.size == 0:
            continue

        frame_peak = float(np.percentile(nonzero, 99.6))
        percentile_samples.append(frame_peak)
        threshold = frame_peak * 0.12
        mask = heat >= threshold
        if not np.any(mask):
            continue

        y_coords, x_coords = np.nonzero(mask)
        frame_min_x = int(x_coords.min())
        frame_max_x = int(x_coords.max())
        frame_min_y = int(y_coords.min())
        frame_max_y = int(y_coords.max())

        crop_min_x = frame_min_x if crop_min_x is None else min(crop_min_x, frame_min_x)
        crop_max_x = frame_max_x if crop_max_x is None else max(crop_max_x, frame_max_x)
        crop_min_y = frame_min_y if crop_min_y is None else min(crop_min_y, frame_min_y)
        crop_max_y = frame_max_y if crop_max_y is None else max(crop_max_y, frame_max_y)

    if not percentile_samples:
        raise RuntimeError("The VDB sequence did not produce any visible fire data")

    if None in (crop_min_x, crop_min_y, crop_max_x, crop_max_y):
        raise RuntimeError("Unable to determine a stable fire crop region")

    crop_padding_x = 8
    crop_padding_y_top = 8
    crop_padding_y_bottom = 4
    crop_box = (
        max(0, crop_min_x - crop_padding_x),
        max(0, crop_min_y - crop_padding_y_top),
        min(global_shape[0] - 1, crop_max_x + crop_padding_x),
        min(global_shape[2] - 1, crop_max_y + crop_padding_y_bottom),
    )

    heat_scale = float(np.percentile(np.array(percentile_samples, dtype=np.float32), 88.0))
    return crop_box, max(heat_scale, 1e-5)


def resize_bilinear(image, target_width, target_height):
    if image.shape[1] == target_width and image.shape[0] == target_height:
        return image.astype(np.float32, copy=False)

    src_height, src_width = image.shape
    x_positions = np.linspace(0.0, src_width - 1, target_width, dtype=np.float32)
    y_positions = np.linspace(0.0, src_height - 1, target_height, dtype=np.float32)

    x0 = np.floor(x_positions).astype(np.int32)
    y0 = np.floor(y_positions).astype(np.int32)
    x1 = np.clip(x0 + 1, 0, src_width - 1)
    y1 = np.clip(y0 + 1, 0, src_height - 1)

    x_weight = (x_positions - x0).reshape(1, -1)
    y_weight = (y_positions - y0).reshape(-1, 1)

    top_left = image[y0[:, None], x0[None, :]]
    top_right = image[y0[:, None], x1[None, :]]
    bottom_left = image[y1[:, None], x0[None, :]]
    bottom_right = image[y1[:, None], x1[None, :]]

    top = (top_left * (1.0 - x_weight)) + (top_right * x_weight)
    bottom = (bottom_left * (1.0 - x_weight)) + (bottom_right * x_weight)
    return (top * (1.0 - y_weight)) + (bottom * y_weight)


def colorize_fire(alpha, heat_norm):
    height, width = alpha.shape
    vertical = np.linspace(0.0, 1.0, height, dtype=np.float32).reshape(-1, 1)

    ember = np.array((0.33, 0.03, 0.0), dtype=np.float32)
    orange = np.array((0.98, 0.24, 0.03), dtype=np.float32)
    gold = np.array((1.0, 0.66, 0.13), dtype=np.float32)
    white = np.array((1.0, 0.97, 0.82), dtype=np.float32)

    orange_mix = np.clip(heat_norm * 1.9 + (1.0 - vertical) * 0.35, 0.0, 1.0)
    gold_mix = np.clip((heat_norm - 0.12) * 1.45 + vertical * 0.22, 0.0, 1.0)
    white_mix = np.clip((heat_norm - 0.48) * 1.55 + vertical * 0.28, 0.0, 1.0)

    rgb = ember + ((orange - ember) * orange_mix[..., None])
    rgb = rgb + ((gold - rgb) * gold_mix[..., None])
    rgb = rgb + ((white - rgb) * white_mix[..., None])

    edge_glow = np.clip(alpha * 1.35, 0.0, 1.0)
    rgb[..., 0] += edge_glow * 0.08
    rgb[..., 1] += edge_glow * 0.05
    rgb[..., 2] += heat_norm * 0.01
    return np.clip(rgb, 0.0, 1.0)


def render_rgba_frame(heat, crop_box, heat_scale, resolution):
    min_x, min_y, max_x, max_y = crop_box
    cropped = heat[min_y : max_y + 1, min_x : max_x + 1]
    normalized = np.clip(cropped / heat_scale, 0.0, 1.35)
    normalized = np.maximum(0.0, normalized - 0.09)
    alpha = np.clip(normalized / 0.91, 0.0, 1.0)
    alpha = np.power(alpha, 0.72)
    alpha = np.where(alpha >= 0.03, alpha, 0.0)

    crop_height, crop_width = alpha.shape
    fit_width = int(round(resolution * 0.92))
    fit_height = int(round(resolution * 0.92))
    scale = min(
        fit_width / max(1, crop_width),
        fit_height / max(1, int(round(crop_height * DEFAULT_VERTICAL_STRETCH))),
    )
    target_width = max(1, int(round(crop_width * scale)))
    target_height = max(
        1,
        min(
            fit_height,
            int(round(crop_height * scale * DEFAULT_VERTICAL_STRETCH)),
        ),
    )

    resized_alpha = resize_bilinear(alpha, target_width, target_height)
    resized_heat_norm = resize_bilinear(
        np.clip(normalized / 0.91, 0.0, 1.0),
        target_width,
        target_height,
    )
    rgba = np.zeros((resolution, resolution, 4), dtype=np.uint8)

    rgb = colorize_fire(resized_alpha, resized_heat_norm)
    fire_rgba = np.zeros((target_height, target_width, 4), dtype=np.uint8)
    fire_rgba[..., :3] = np.round(rgb * 255.0).astype(np.uint8)
    fire_rgba[..., 3] = np.round(np.clip(resized_alpha, 0.0, 1.0) * 255.0).astype(np.uint8)

    offset_x = (resolution - target_width) // 2
    offset_y = resolution - target_height - int(round(resolution * 0.025))
    offset_y = max(0, min(offset_y, resolution - target_height))

    rgba[
        offset_y : offset_y + target_height,
        offset_x : offset_x + target_width,
    ] = fire_rgba
    return rgba


def png_chunk(chunk_type, payload):
    chunk = chunk_type + payload
    crc = zlib.crc32(chunk) & 0xFFFFFFFF
    return struct.pack(">I", len(payload)) + chunk + struct.pack(">I", crc)


def write_png(path, rgba):
    height, width, channels = rgba.shape
    if channels != 4:
        raise ValueError("write_png expects an RGBA image")

    raw_rows = []
    for row in rgba:
        raw_rows.append(b"\x00" + row.tobytes())
    compressed = zlib.compress(b"".join(raw_rows), level=9)

    with path.open("wb") as handle:
        handle.write(b"\x89PNG\r\n\x1a\n")
        handle.write(
            png_chunk(
                b"IHDR",
                struct.pack(">IIBBBBB", width, height, 8, 6, 0, 0, 0),
            )
        )
        handle.write(png_chunk(b"IDAT", compressed))
        handle.write(png_chunk(b"IEND", b""))


def clear_existing_frames(output_dir):
    output_dir.mkdir(parents=True, exist_ok=True)
    for frame_path in output_dir.glob("frame_*.png"):
        frame_path.unlink()
    metadata_path = output_dir / "metadata.json"
    if metadata_path.exists():
        metadata_path.unlink()


def write_metadata(output_dir, frame_count, frame_step, resolution, crop_box, heat_scale):
    metadata = {
        "frame_count": frame_count,
        "frame_rate": DEFAULT_FRAME_RATE / max(1, frame_step),
        "resolution": resolution,
        "source_frame_step": frame_step,
        "projection_axis": "y",
        "heat_scale": heat_scale,
        "crop_box": list(crop_box),
    }
    (output_dir / "metadata.json").write_text(json.dumps(metadata, indent=2))


def bake_frames(output_dir, frames, global_min, global_shape, crop_box, heat_scale, resolution):
    print(
        f"Baking {len(frames)} fire frames from {frames[0].parent} to {output_dir}"
    )
    for rendered_index, frame_path in enumerate(frames):
        heat = project_heat(volume_to_array(load_flame_grid(frame_path), global_min, global_shape))
        rgba = render_rgba_frame(heat, crop_box, heat_scale, resolution)
        output_path = output_dir / f"frame_{rendered_index:04d}.png"
        write_png(output_path, rgba)
        if rendered_index % 20 == 0:
            print(f"Rendered {rendered_index + 1}/{len(frames)}: {output_path.name}")
    print(f"Rendered {len(frames)} total fire frames")


def main():
    output_dir, vdb_dir, frame_step, resolution = parse_args()
    if frame_step <= 0:
        raise ValueError("frame_step must be >= 1")
    if resolution <= 0:
        raise ValueError("resolution must be >= 1")

    frame_paths = list_vdb_frames(vdb_dir)
    selected_frames = frame_paths[::frame_step]

    global_min, _global_max, global_shape = collect_global_bbox(selected_frames)
    crop_box, heat_scale = build_analysis(selected_frames, global_min, global_shape)

    clear_existing_frames(output_dir)
    bake_frames(
        output_dir,
        selected_frames,
        global_min,
        global_shape,
        crop_box,
        heat_scale,
        resolution,
    )
    write_metadata(
        output_dir,
        len(selected_frames),
        frame_step,
        resolution,
        crop_box,
        heat_scale,
    )
    print(
        f"Completed bake: {len(selected_frames)} frames at {DEFAULT_FRAME_RATE / frame_step:.2f} fps"
    )


if __name__ == "__main__":
    main()
