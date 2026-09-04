#!/usr/bin/env python3
#
# Copyright 2026 O-Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
from typing import Any, Callable


def _load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        document = json.load(stream)
    if not isinstance(document, dict):
        raise RuntimeError(f"{path} must contain a JSON object")
    return document


def _write_json_atomic(path: Path, document: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temp_path = path.with_suffix(path.suffix + ".tmp")
    temp_path.write_text(json.dumps(document, indent=2) + "\n", encoding="utf-8")
    temp_path.replace(path)


def _utc_now() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _sigmoid(value: float) -> float:
    if value >= 0.0:
        z = math.exp(-value)
        return 1.0 / (1.0 + z)
    z = math.exp(value)
    return z / (1.0 + z)


def _control_requested(control_directory: Path, name: str) -> bool:
    return (control_directory / name).exists()


def _progress_percent(
    iteration: int,
    target_iterations: int,
    plateau_windows: int,
    plateau_patience_windows: int,
    plateau_detected: bool,
) -> tuple[float, float]:
    iteration_progress = 100.0 * min(1.0, float(iteration) / float(max(1, target_iterations)))
    plateau_confidence = min(
        1.0,
        float(max(0, plateau_windows)) / float(max(1, plateau_patience_windows)),
    )
    quality_progress = min(100.0, iteration_progress * 0.70 + plateau_confidence * 30.0)
    if plateau_detected:
        quality_progress = max(quality_progress, 100.0)
    return iteration_progress, quality_progress


def _pose_position(capture: dict[str, Any]) -> tuple[float, float, float] | None:
    pose = capture.get("odometry_pose", {})
    if not isinstance(pose, dict):
        return None
    position = pose.get("position", {})
    if not isinstance(position, dict):
        return None
    try:
        x = float(position.get("x", 0.0))
        y = float(position.get("y", 0.0))
        z = float(position.get("z", 0.0))
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in (x, y, z)):
        return None
    return x, y, z


def _tile_id(tile_x: int, tile_y: int) -> str:
    return f"x{tile_x:+05d}_y{tile_y:+05d}".replace("+", "p").replace("-", "m")


def _resolve_capture_file(
    manifest_file: Path,
    capture_manifest: dict[str, Any],
    path: str,
) -> Path:
    candidate = Path(path)
    if candidate.is_absolute():
        return candidate.resolve()
    output_directory = capture_manifest.get("output_directory")
    if isinstance(output_directory, str) and output_directory:
        return (Path(output_directory) / candidate).resolve()
    return (manifest_file.parent / candidate).resolve()


def _translation_from_transform(transform: dict[str, Any]) -> tuple[float, float, float] | None:
    translation = transform.get("translation", {})
    if not isinstance(translation, dict):
        return None
    try:
        x = float(translation.get("x", 0.0))
        y = float(translation.get("y", 0.0))
        z = float(translation.get("z", 0.0))
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in (x, y, z)):
        return None
    return x, y, z


def _matrix_from_transform(transform: dict[str, Any]) -> list[list[float]] | None:
    translation = transform.get("translation", {})
    rotation = transform.get("rotation", {})
    if not isinstance(translation, dict) or not isinstance(rotation, dict):
        return None
    try:
        tx = float(translation.get("x", 0.0))
        ty = float(translation.get("y", 0.0))
        tz = float(translation.get("z", 0.0))
        qx = float(rotation.get("x", 0.0))
        qy = float(rotation.get("y", 0.0))
        qz = float(rotation.get("z", 0.0))
        qw = float(rotation.get("w", 1.0))
    except (TypeError, ValueError):
        return None
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 0.0 or not math.isfinite(norm):
        return None
    qx /= norm
    qy /= norm
    qz /= norm
    qw /= norm
    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz
    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy), tx],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx), ty],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy), tz],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _read_ppm(path: Path) -> Any:
    import numpy as np

    with path.open("rb") as stream:
        magic = stream.readline().strip()
        if magic != b"P6":
            raise RuntimeError(f"{path} is not a binary PPM file")
        tokens = []
        while len(tokens) < 3:
            line = stream.readline()
            if not line:
                raise RuntimeError(f"{path} has an incomplete PPM header")
            line = line.split(b"#", 1)[0]
            tokens.extend(line.split())
        width = int(tokens[0])
        height = int(tokens[1])
        max_value = int(tokens[2])
        if width <= 0 or height <= 0 or max_value <= 0:
            raise RuntimeError(f"{path} has invalid PPM dimensions")
        data = stream.read(width * height * 3)
    if len(data) != width * height * 3:
        raise RuntimeError(f"{path} has incomplete RGB image data")
    image = np.frombuffer(data, dtype=np.uint8).reshape((height, width, 3))
    return image.astype(np.float32) / float(max_value)


def _resize_image_and_intrinsics(
    image: Any,
    k_matrix: list[float],
    max_dimension: int,
) -> tuple[Any, Any]:
    import numpy as np

    height, width = image.shape[:2]
    scale = 1.0
    if max(height, width) > max_dimension:
        scale = float(max_dimension) / float(max(height, width))
        new_width = max(1, int(round(width * scale)))
        new_height = max(1, int(round(height * scale)))
        y_indices = np.linspace(0, height - 1, new_height).round().astype(np.int64)
        x_indices = np.linspace(0, width - 1, new_width).round().astype(np.int64)
        image = image[y_indices][:, x_indices]

    k = np.asarray(k_matrix, dtype=np.float32).reshape((3, 3)).copy()
    k[0, :] *= scale
    k[1, :] *= scale
    return image, k


def _collect_camera_samples(
    manifest_file: Path,
    capture_manifest: dict[str, Any],
    tile_key: tuple[int, int],
    tile_size_meters: float,
) -> list[dict[str, Any]]:
    tile_x, tile_y = tile_key
    min_x = tile_x * tile_size_meters
    min_y = tile_y * tile_size_meters
    max_x = (tile_x + 1) * tile_size_meters
    max_y = (tile_y + 1) * tile_size_meters
    overlap = min(max(tile_size_meters * 0.10, 0.50), 2.00)
    samples: list[dict[str, Any]] = []

    captures = capture_manifest.get("captures", [])
    for capture in captures:
        if not isinstance(capture, dict):
            continue
        position = _pose_position(capture)
        if position is None:
            continue
        x, y, _ = position
        if x < min_x - overlap or x >= max_x + overlap:
            continue
        if y < min_y - overlap or y >= max_y + overlap:
            continue
        cameras = capture.get("cameras", [])
        if not isinstance(cameras, list):
            continue
        for camera in cameras:
            if not isinstance(camera, dict):
                continue
            if camera.get("image_format") != "ppm":
                continue
            image_file = camera.get("image_file")
            camera_info = camera.get("camera_info")
            world_from_camera = camera.get("world_from_camera")
            if not isinstance(image_file, str) or not image_file:
                continue
            if not isinstance(camera_info, dict) or not isinstance(world_from_camera, dict):
                continue
            k = camera_info.get("k")
            camtoworld = _matrix_from_transform(world_from_camera)
            camera_position = _translation_from_transform(world_from_camera)
            if not isinstance(k, list) or len(k) != 9:
                continue
            if camtoworld is None or camera_position is None:
                continue
            image_path = _resolve_capture_file(manifest_file, capture_manifest, image_file)
            if not image_path.exists():
                continue
            samples.append(
                {
                    "image_file": image_path,
                    "k": [float(value) for value in k],
                    "camtoworld": camtoworld,
                    "camera_position": camera_position,
                }
            )
    return samples


def _load_training_backend() -> tuple[Any, Any, Callable[..., Any], str]:
    try:
        import torch
    except Exception as exc:  # noqa: BLE001
        raise RuntimeError("PyTorch is required for CUDA gSplat training") from exc
    try:
        import gsplat
        from gsplat.rendering import rasterization
    except Exception as exc:  # noqa: BLE001
        raise RuntimeError("gsplat is required for CUDA Gaussian splat training") from exc
    if not torch.cuda.is_available():
        raise RuntimeError(
            "CUDA is required for Gaussian splat training, but no CUDA device is available"
        )
    version = str(getattr(gsplat, "__version__", "unknown"))
    return torch, gsplat, rasterization, version


def _initialize_splats(
    torch: Any,
    samples: list[dict[str, Any]],
    tile_bounds: dict[str, float],
) -> dict[str, Any]:
    import numpy as np

    device = torch.device("cuda")
    generator = torch.Generator(device=device)
    generator.manual_seed(42)
    sample_count = len(samples)
    gaussian_count = min(max(sample_count * 256, 2048), 32768)
    camera_positions = np.asarray(
        [sample["camera_position"] for sample in samples],
        dtype=np.float32,
    )
    z_min = float(camera_positions[:, 2].min() - 1.0)
    z_max = float(camera_positions[:, 2].max() + 1.0)

    means = torch.empty((gaussian_count, 3), device=device)
    means[:, 0] = torch.empty(gaussian_count, device=device).uniform_(
        tile_bounds["min_x"], tile_bounds["max_x"], generator=generator
    )
    means[:, 1] = torch.empty(gaussian_count, device=device).uniform_(
        tile_bounds["min_y"], tile_bounds["max_y"], generator=generator
    )
    means[:, 2] = torch.empty(gaussian_count, device=device).uniform_(
        z_min,
        z_max,
        generator=generator,
    )

    quats = torch.zeros((gaussian_count, 4), device=device)
    quats[:, 0] = 1.0
    tile_span = max(tile_bounds["max_x"] - tile_bounds["min_x"], 0.1)
    scale = max(min(tile_span / 150.0, 0.20), 0.01)
    scales = torch.full((gaussian_count, 3), math.log(scale), device=device)
    opacity_value = torch.logit(torch.tensor(0.10)).item()
    opacities = torch.full((gaussian_count,), opacity_value, device=device)
    colors = torch.empty((gaussian_count, 3), device=device).uniform_(
        0.25,
        0.75,
        generator=generator,
    )

    return {
        "means": torch.nn.Parameter(means),
        "quats": torch.nn.Parameter(quats),
        "scales": torch.nn.Parameter(scales),
        "opacities": torch.nn.Parameter(opacities),
        "colors": torch.nn.Parameter(torch.logit(colors.clamp(0.01, 0.99))),
    }


def _write_gaussian_ply(path: Path, torch: Any, splats: dict[str, Any]) -> int:
    means = splats["means"].detach().cpu()
    quats = torch.nn.functional.normalize(splats["quats"].detach().cpu(), dim=-1)
    scales = splats["scales"].detach().cpu()
    opacities = splats["opacities"].detach().cpu()
    colors = torch.sigmoid(splats["colors"].detach().cpu())
    sh_c0 = 0.28209479177387814
    f_dc = (colors - 0.5) / sh_c0
    gaussian_count = int(means.shape[0])

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        stream.write("ply\n")
        stream.write("format ascii 1.0\n")
        stream.write(f"element vertex {gaussian_count}\n")
        for name in ("x", "y", "z", "nx", "ny", "nz"):
            stream.write(f"property float {name}\n")
        for name in ("f_dc_0", "f_dc_1", "f_dc_2"):
            stream.write(f"property float {name}\n")
        stream.write("property float opacity\n")
        for name in ("scale_0", "scale_1", "scale_2"):
            stream.write(f"property float {name}\n")
        for name in ("rot_0", "rot_1", "rot_2", "rot_3"):
            stream.write(f"property float {name}\n")
        stream.write("end_header\n")
        for index in range(gaussian_count):
            x, y, z = means[index].tolist()
            c0, c1, c2 = f_dc[index].tolist()
            s0, s1, s2 = scales[index].tolist()
            r0, r1, r2, r3 = quats[index].tolist()
            opacity = float(opacities[index].item())
            stream.write(
                f"{x:.6f} {y:.6f} {z:.6f} "
                f"0.0 0.0 0.0 {c0:.6f} {c1:.6f} {c2:.6f} "
                f"{opacity:.6f} {s0:.6f} {s1:.6f} {s2:.6f} "
                f"{r0:.6f} {r1:.6f} {r2:.6f} {r3:.6f}\n"
            )
    return gaussian_count


def _write_gaussian_ply_atomic(path: Path, torch: Any, splats: dict[str, Any]) -> int:
    temp_path = path.with_suffix(path.suffix + ".tmp")
    gaussian_count = _write_gaussian_ply(temp_path, torch, splats)
    temp_path.replace(path)
    return gaussian_count


def _save_checkpoint(
    checkpoint_file: Path,
    torch: Any,
    splats: dict[str, Any],
    optimizers: dict[str, Any],
    iteration: int,
    target_iterations: int,
    latest_loss: float,
    loss_ema: float,
    improvement_ema: float,
    plateau_windows: int,
) -> None:
    checkpoint_file.parent.mkdir(parents=True, exist_ok=True)
    torch.save(
        {
            "iteration": int(iteration),
            "target_iterations": int(target_iterations),
            "latest_loss": float(latest_loss),
            "loss_ema": float(loss_ema),
            "loss_improvement_ema": float(improvement_ema),
            "plateau_windows": int(plateau_windows),
            "splats": {name: parameter.detach().cpu() for name, parameter in splats.items()},
            "optimizers": {name: optimizer.state_dict() for name, optimizer in optimizers.items()},
        },
        checkpoint_file,
    )


def _load_checkpoint(
    checkpoint_file: Path,
    torch: Any,
    splats: dict[str, Any],
    optimizers: dict[str, Any],
) -> dict[str, Any] | None:
    if not checkpoint_file.exists():
        return None
    checkpoint = torch.load(checkpoint_file, map_location="cuda", weights_only=False)
    saved_splats = checkpoint.get("splats", {})
    if not isinstance(saved_splats, dict):
        return None
    with torch.no_grad():
        for name, parameter in splats.items():
            saved = saved_splats.get(name)
            if saved is not None and tuple(saved.shape) == tuple(parameter.shape):
                parameter.copy_(saved.to(device=parameter.device))
    saved_optimizers = checkpoint.get("optimizers", {})
    if isinstance(saved_optimizers, dict):
        for name, optimizer in optimizers.items():
            state = saved_optimizers.get(name)
            if state is not None:
                optimizer.load_state_dict(state)
    return checkpoint


def _write_manifest(
    manifest_file: Path,
    capture_manifest: dict[str, Any],
    capture_manifest_file: Path,
    output_directory: Path,
    mission_id: str,
    tile_size_meters: float,
    max_iterations_per_tile: int,
    tile_count: int,
    tiles: list[dict[str, Any]],
    status: str,
    gsplat_version: str,
) -> None:
    completed_tile_count = sum(1 for tile in tiles if tile.get("status") == "completed")
    manifest = {
        "representation": "tiled_gaussian_splat",
        "mission_id": mission_id or str(capture_manifest.get("mission_id", "")),
        "source_capture_manifest_file": str(capture_manifest_file),
        "artifact_directory": str(output_directory),
        "created_at": _utc_now(),
        "updated_at": _utc_now(),
        "tile_size_meters": tile_size_meters,
        "tile_overlap_meters": min(max(tile_size_meters * 0.10, 0.50), 2.00),
        "max_iterations_per_tile": max_iterations_per_tile,
        "tile_count": tile_count,
        "completed_tile_count": completed_tile_count,
        "status": status,
        "training_backend": "gsplat_cuda",
        "gsplat": {
            "available": True,
            "version": gsplat_version,
            "requires_cuda": True,
        },
        "viewer": {
            "type": "tiled_ply_gaussian_splat",
            "manifest_file": "gaussian_splat_manifest.json",
            "tile_root": "tiles",
        },
        "tiles": tiles,
    }
    _write_json_atomic(manifest_file, manifest)


def _train_tile(
    samples: list[dict[str, Any]],
    tile_bounds: dict[str, float],
    output_file: Path,
    max_iterations: int,
    max_image_dimension: int,
    checkpoint_file: Path,
    tile_metadata_file: Path,
    progress_file: Path,
    control_directory: Path,
    checkpoint_callback: Callable[[dict[str, Any]], None],
    tile_metadata: dict[str, Any],
    resume: bool,
    auto_stop_enabled: bool,
    checkpoint_interval: int,
    min_iterations_before_plateau: int,
    plateau_patience_windows: int,
    min_relative_loss_improvement: float,
) -> dict[str, Any]:
    torch, _, rasterization, gsplat_version = _load_training_backend()
    import torch.nn.functional as functional

    if not samples:
        raise RuntimeError("Tile has no trainable PPM camera samples")
    device = torch.device("cuda")
    training_samples = []
    for sample in samples:
        image, k = _resize_image_and_intrinsics(
            _read_ppm(sample["image_file"]),
            sample["k"],
            max_image_dimension,
        )
        image_tensor = torch.from_numpy(image).to(device=device, dtype=torch.float32)
        k_tensor = torch.from_numpy(k).to(device=device, dtype=torch.float32)
        camtoworld = torch.tensor(sample["camtoworld"], device=device, dtype=torch.float32)
        training_samples.append({"image": image_tensor, "k": k_tensor, "camtoworld": camtoworld})

    splats = _initialize_splats(torch, samples, tile_bounds)
    optimizers = {
        "means": torch.optim.Adam([splats["means"]], lr=1.6e-4),
        "quats": torch.optim.Adam([splats["quats"]], lr=1.0e-3),
        "scales": torch.optim.Adam([splats["scales"]], lr=5.0e-3),
        "opacities": torch.optim.Adam([splats["opacities"]], lr=5.0e-2),
        "colors": torch.optim.Adam([splats["colors"]], lr=2.5e-3),
    }

    final_loss = 0.0
    iterations = max(1, int(max_iterations))
    checkpoint_interval = max(1, int(checkpoint_interval))
    start_iteration = 0
    loss_ema = 0.0
    previous_loss_ema = 0.0
    improvement_ema = 1.0
    plateau_windows = 0
    if resume:
        checkpoint = _load_checkpoint(checkpoint_file, torch, splats, optimizers)
        if checkpoint is not None:
            start_iteration = int(checkpoint.get("iteration", 0))
            final_loss = float(checkpoint.get("latest_loss", 0.0))
            loss_ema = float(checkpoint.get("loss_ema", final_loss))
            previous_loss_ema = loss_ema
            improvement_ema = float(checkpoint.get("loss_improvement_ema", 1.0))
            plateau_windows = int(checkpoint.get("plateau_windows", 0))

    status = "running"
    plateau_detected = False
    for step in range(start_iteration, iterations):
        sample = training_samples[step % len(training_samples)]
        image = sample["image"].unsqueeze(0)
        height = int(image.shape[1])
        width = int(image.shape[2])
        render_colors, render_alphas, _ = rasterization(
            means=splats["means"],
            quats=functional.normalize(splats["quats"], dim=-1),
            scales=torch.exp(splats["scales"]),
            opacities=torch.sigmoid(splats["opacities"]),
            colors=torch.sigmoid(splats["colors"]),
            viewmats=torch.linalg.inv_ex(sample["camtoworld"].unsqueeze(0)).inverse,
            Ks=sample["k"].unsqueeze(0),
            width=width,
            height=height,
            packed=False,
            rasterize_mode="classic",
            camera_model="pinhole",
        )
        rgb = render_colors[..., :3]
        background = torch.ones_like(rgb)
        rgb = rgb + background * (1.0 - render_alphas)
        reconstruction_loss = functional.l1_loss(rgb, image)
        scale_loss = torch.exp(splats["scales"]).mean() * 0.001
        opacity_loss = torch.sigmoid(splats["opacities"]).mean() * 0.0001
        loss = reconstruction_loss + scale_loss + opacity_loss
        loss.backward()
        final_loss = float(loss.detach().item())
        for optimizer in optimizers.values():
            optimizer.step()
            optimizer.zero_grad(set_to_none=True)

        completed_iteration = step + 1
        if loss_ema <= 0.0:
            loss_ema = final_loss
            previous_loss_ema = final_loss
        else:
            previous_loss_ema = loss_ema
            loss_ema = 0.90 * loss_ema + 0.10 * final_loss
        relative_improvement = max(0.0, (previous_loss_ema - loss_ema) / max(previous_loss_ema, 1.0e-9))
        improvement_ema = 0.85 * improvement_ema + 0.15 * relative_improvement
        if (
            completed_iteration >= min_iterations_before_plateau
            and improvement_ema < min_relative_loss_improvement
        ):
            plateau_windows += 1
        else:
            plateau_windows = 0
        plateau_detected = plateau_windows >= plateau_patience_windows

        should_checkpoint = (
            completed_iteration == iterations
            or completed_iteration % checkpoint_interval == 0
            or _control_requested(control_directory, "pause.requested")
            or (auto_stop_enabled and plateau_detected)
        )
        if not should_checkpoint:
            continue

        _save_checkpoint(
            checkpoint_file,
            torch,
            splats,
            optimizers,
            completed_iteration,
            iterations,
            final_loss,
            loss_ema,
            improvement_ema,
            plateau_windows,
        )
        gaussian_count = _write_gaussian_ply_atomic(output_file, torch, splats)
        iteration_progress, quality_progress = _progress_percent(
            completed_iteration,
            iterations,
            plateau_windows,
            plateau_patience_windows,
            plateau_detected,
        )
        tile_metadata.update({
            "status": "running",
            "iterations": completed_iteration,
            "target_iterations": iterations,
            "latest_checkpoint_iteration": completed_iteration,
            "gaussian_count": gaussian_count,
            "latest_loss": final_loss,
            "loss_ema": loss_ema,
            "loss_improvement_ema": improvement_ema,
            "plateau_windows": plateau_windows,
            "plateau_detected": plateau_detected,
            "iteration_progress_percent": iteration_progress,
            "quality_progress_percent": quality_progress,
            "updated_at": _utc_now(),
        })
        _write_json_atomic(tile_metadata_file, tile_metadata)
        checkpoint_callback(dict(tile_metadata))
        _write_json_atomic(progress_file, {
            "state": "running",
            "current_tile_id": tile_metadata.get("tile_id", ""),
            "current_iteration": completed_iteration,
            "latest_checkpoint_iteration": completed_iteration,
            "target_iterations_per_tile": iterations,
            "latest_loss": final_loss,
            "loss_ema": loss_ema,
            "loss_improvement_ema": improvement_ema,
            "plateau_detected": plateau_detected,
            "iteration_progress_percent": iteration_progress,
            "quality_progress_percent": quality_progress,
            "updated_at": _utc_now(),
        })
        if _control_requested(control_directory, "pause.requested"):
            status = "paused"
            break
        if auto_stop_enabled and plateau_detected:
            status = "completed"
            break

    final_iteration = min(iterations, max(start_iteration, int(tile_metadata.get("iterations", start_iteration))))
    if status == "running":
        status = "completed"
        final_iteration = iterations

    gaussian_count = int(tile_metadata.get("gaussian_count", 0) or 0)
    if status == "completed":
        gaussian_count = _write_gaussian_ply_atomic(output_file, torch, splats)
    torch.cuda.synchronize()
    iteration_progress, quality_progress = _progress_percent(
        final_iteration,
        iterations,
        plateau_windows,
        plateau_patience_windows,
        plateau_detected,
    )
    return {
        "training_backend": "gsplat_cuda",
        "gsplat_version": gsplat_version,
        "status": status,
        "iterations": final_iteration,
        "target_iterations": iterations,
        "latest_checkpoint_iteration": final_iteration,
        "sample_count": len(training_samples),
        "gaussian_count": gaussian_count,
        "final_loss": final_loss,
        "latest_loss": final_loss,
        "loss_ema": loss_ema,
        "loss_improvement_ema": improvement_ema,
        "plateau_detected": plateau_detected,
        "iteration_progress_percent": iteration_progress,
        "quality_progress_percent": quality_progress,
        "max_image_dimension": max_image_dimension,
    }


def build_splat(
    capture_manifest_file: Path,
    output_directory: Path,
    mission_id: str,
    tile_size_meters: float,
    max_iterations_per_tile: int,
    max_image_dimension: int,
    control_directory: Path,
    resume: bool,
    auto_stop_enabled: bool,
    checkpoint_interval: int,
    min_iterations_before_plateau: int,
    plateau_patience_windows: int,
    min_relative_loss_improvement: float,
) -> dict[str, Any]:
    capture_manifest = _load_json(capture_manifest_file)
    captures = capture_manifest.get("captures", [])
    if not isinstance(captures, list) or not captures:
        raise RuntimeError("Gaussian capture manifest does not contain any captures")

    capture_count_by_tile: dict[tuple[int, int], int] = {}
    for capture in captures:
        if not isinstance(capture, dict):
            continue
        position = _pose_position(capture)
        if position is None:
            continue
        tile_x = math.floor(position[0] / tile_size_meters)
        tile_y = math.floor(position[1] / tile_size_meters)
        key = (tile_x, tile_y)
        capture_count_by_tile[key] = capture_count_by_tile.get(key, 0) + 1

    if not capture_count_by_tile:
        raise RuntimeError("Gaussian capture manifest does not contain usable map-frame poses")

    _, _, _, gsplat_version = _load_training_backend()
    output_directory.mkdir(parents=True, exist_ok=True)
    control_directory.mkdir(parents=True, exist_ok=True)
    progress_file = output_directory / "progress.json"
    manifest_file = output_directory / "gaussian_splat_manifest.json"
    tiles = []
    status = "completed"
    tile_count = len(capture_count_by_tile)
    for key in sorted(capture_count_by_tile):
        tile_x, tile_y = key
        tile_name = _tile_id(tile_x, tile_y)
        tile_directory = output_directory / "tiles" / tile_name
        tile_directory.mkdir(parents=True, exist_ok=True)
        tile_bounds = {
            "min_x": tile_x * tile_size_meters,
            "min_y": tile_y * tile_size_meters,
            "max_x": (tile_x + 1) * tile_size_meters,
            "max_y": (tile_y + 1) * tile_size_meters,
        }
        splat_file = tile_directory / "splat.ply"
        tile_metadata_file = tile_directory / "tile.json"
        if resume and tile_metadata_file.exists() and splat_file.exists():
            existing_tile = _load_json(tile_metadata_file)
            if existing_tile.get("status") == "completed":
                tiles.append(existing_tile)
                continue
        tile_overlap_meters = min(max(tile_size_meters * 0.10, 0.50), 2.00)
        samples = _collect_camera_samples(
            capture_manifest_file,
            capture_manifest,
            key,
            tile_size_meters,
        )
        tile_metadata = {
            "tile_id": tile_name,
            "tile_index": {"x": tile_x, "y": tile_y},
            "bounds": tile_bounds,
            "capture_count": capture_count_by_tile[key],
            "tile_overlap_meters": tile_overlap_meters,
            "status": "running",
            "gaussian_count": 0,
            "sample_count": len(samples),
            "iterations": 0,
            "target_iterations": max_iterations_per_tile,
            "latest_checkpoint_iteration": 0,
            "splat_file": str(splat_file.relative_to(output_directory)),
            "checkpoint_file": str((tile_directory / "checkpoint.pt").relative_to(output_directory)),
            "updated_at": _utc_now(),
        }
        _write_json_atomic(tile_metadata_file, tile_metadata)
        _write_json_atomic(progress_file, {
            "state": "running",
            "current_tile_id": tile_name,
            "current_iteration": 0,
            "latest_checkpoint_iteration": 0,
            "target_iterations_per_tile": max_iterations_per_tile,
            "updated_at": _utc_now(),
        })
        checkpoint_tiles = [*tiles, dict(tile_metadata)]
        _write_manifest(
            manifest_file,
            capture_manifest,
            capture_manifest_file,
            output_directory,
            mission_id,
            tile_size_meters,
            max_iterations_per_tile,
            tile_count,
            checkpoint_tiles,
            "running",
            gsplat_version,
        )

        def handle_tile_checkpoint(checkpoint_tile: dict[str, Any]) -> None:
            checkpoint_tiles = [*tiles, checkpoint_tile]
            _write_manifest(
                manifest_file,
                capture_manifest,
                capture_manifest_file,
                output_directory,
                mission_id,
                tile_size_meters,
                max_iterations_per_tile,
                tile_count,
                checkpoint_tiles,
                str(checkpoint_tile.get("status", "running")),
                gsplat_version,
            )

        stats = _train_tile(
            samples,
            tile_bounds,
            splat_file,
            max_iterations_per_tile,
            max_image_dimension,
            tile_directory / "checkpoint.pt",
            tile_metadata_file,
            progress_file,
            control_directory,
            handle_tile_checkpoint,
            tile_metadata,
            resume,
            auto_stop_enabled,
            checkpoint_interval,
            min_iterations_before_plateau,
            plateau_patience_windows,
            min_relative_loss_improvement,
        )
        tile_metadata.update({
            "status": stats["status"],
            "gaussian_count": stats["gaussian_count"],
            "sample_count": stats["sample_count"],
            "training_backend": stats["training_backend"],
            "gsplat_version": stats["gsplat_version"],
            "iterations": stats["iterations"],
            "target_iterations": stats["target_iterations"],
            "latest_checkpoint_iteration": stats["latest_checkpoint_iteration"],
            "final_loss": stats["final_loss"],
            "latest_loss": stats["latest_loss"],
            "loss_ema": stats["loss_ema"],
            "loss_improvement_ema": stats["loss_improvement_ema"],
            "plateau_detected": stats["plateau_detected"],
            "iteration_progress_percent": stats["iteration_progress_percent"],
            "quality_progress_percent": stats["quality_progress_percent"],
            "splat_file": str(splat_file.relative_to(output_directory)),
            "updated_at": _utc_now(),
        })
        _write_json_atomic(tile_metadata_file, tile_metadata)
        tiles.append(tile_metadata)
        if stats["status"] == "paused":
            status = str(stats["status"])
            break

    completed_tile_count = sum(1 for tile in tiles if tile.get("status") == "completed")
    if status == "completed" and completed_tile_count < len(capture_count_by_tile):
        status = "partial"
    _write_manifest(
        manifest_file,
        capture_manifest,
        capture_manifest_file,
        output_directory,
        mission_id,
        tile_size_meters,
        max_iterations_per_tile,
        tile_count,
        tiles,
        status,
        gsplat_version,
    )
    manifest = _load_json(manifest_file)
    _write_json_atomic(progress_file, {
        "state": status,
        "current_tile_id": tiles[-1].get("tile_id", "") if tiles else "",
        "current_iteration": int(tiles[-1].get("iterations", 0)) if tiles else 0,
        "latest_checkpoint_iteration": int(tiles[-1].get("latest_checkpoint_iteration", 0)) if tiles else 0,
        "target_iterations_per_tile": max_iterations_per_tile,
        "completed_tile_count": completed_tile_count,
        "tile_count": len(capture_count_by_tile),
        "quality_progress_percent": (
            sum(float(tile.get("quality_progress_percent", 0.0)) for tile in tiles) /
            float(max(1, len(capture_count_by_tile)))
        ),
        "updated_at": _utc_now(),
    })
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser(description="Train tiled CUDA gSplat artifacts.")
    parser.add_argument("--capture-manifest", required=True)
    parser.add_argument("--output-directory", required=True)
    parser.add_argument("--mission-id", default="")
    parser.add_argument("--tile-size-meters", type=float, default=10.0)
    parser.add_argument("--max-iterations-per-tile", type=int, default=3000)
    parser.add_argument("--max-image-dimension", type=int, default=480)
    parser.add_argument("--control-directory", required=True)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--auto-stop-enabled", action="store_true")
    parser.add_argument("--checkpoint-interval", type=int, default=100)
    parser.add_argument("--min-iterations-before-plateau", type=int, default=300)
    parser.add_argument("--plateau-patience-windows", type=int, default=3)
    parser.add_argument("--min-relative-loss-improvement", type=float, default=0.0025)
    parser.add_argument("--result-file", required=True)
    args = parser.parse_args()

    try:
        if args.tile_size_meters <= 0.0:
            raise RuntimeError("tile_size_meters must be positive")
        if args.max_iterations_per_tile <= 0:
            raise RuntimeError("max_iterations_per_tile must be positive")
        if args.max_image_dimension <= 0:
            raise RuntimeError("max_image_dimension must be positive")
        if args.checkpoint_interval <= 0:
            raise RuntimeError("checkpoint_interval must be positive")
        output_directory = Path(args.output_directory).expanduser().resolve()
        manifest = build_splat(
            Path(args.capture_manifest).expanduser().resolve(),
            output_directory,
            args.mission_id,
            args.tile_size_meters,
            args.max_iterations_per_tile,
            args.max_image_dimension,
            Path(args.control_directory).expanduser().resolve(),
            bool(args.resume),
            bool(args.auto_stop_enabled),
            args.checkpoint_interval,
            args.min_iterations_before_plateau,
            args.plateau_patience_windows,
            args.min_relative_loss_improvement,
        )
        status = str(manifest.get("status", "completed"))
        result = {
            "success": status in {"completed", "paused", "partial"},
            "message": f"CUDA gSplat build {status}",
            "artifact_manifest_file": str(output_directory / "gaussian_splat_manifest.json"),
            "tile_count": int(manifest["tile_count"]),
            "completed_tile_count": int(manifest["completed_tile_count"]),
            "status": status,
        }
    except Exception as exc:  # noqa: BLE001
        result = {
            "success": False,
            "message": str(exc),
            "artifact_manifest_file": "",
            "tile_count": 0,
            "completed_tile_count": 0,
        }
    Path(args.result_file).write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    return 0 if result["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
