#!/usr/bin/env python3

import argparse
import copy
import re
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, Optional

import yaml


YAML_BOOLISH_STRINGS = {
    "y",
    "n",
    "yes",
    "no",
    "on",
    "off",
    "true",
    "false",
    "null",
    "~",
}


class ExperimentYamlDumper(yaml.SafeDumper):
    pass


def represent_experiment_string(dumper: yaml.SafeDumper, data: str):
    style = '"' if data.strip().lower() in YAML_BOOLISH_STRINGS else None
    return dumper.represent_scalar("tag:yaml.org,2002:str", data, style=style)


ExperimentYamlDumper.add_representer(str, represent_experiment_string)


RECOMMENDED_PRESET_ORDER = [
    "all_off",
    "cspace_only",
    "target_isotropic",
    "collision_on",
    "target_directional",
    "orientation_on",
    "joint_limits_on",
    "damping_on",
]

PRESET_DESCRIPTIONS = {
    "current": "Copy the source params without changing any tuning values.",
    "all_off": (
        "Disable the main RMP leaves by zeroing their metric scalar or metric weight. "
        "Also set cspace_target_inertia and damping_rmp_inertia to zero."
    ),
    "cspace_only": (
        "Start from all_off, then restore the c-space target metric and gains while "
        "keeping inertia at zero."
    ),
    "target_isotropic": (
        "Start from cspace_only, then restore the position target gains and metric "
        "scalars, but keep the directional metric term effectively disabled."
    ),
    "collision_on": (
        "Start from target_isotropic, then restore the collision RMP parameters."
    ),
    "target_directional": (
        "Start from collision_on, then restore the directional target metric terms "
        "and proximity boosting."
    ),
    "orientation_on": (
        "Start from target_directional, then restore the orientation RMP gains and "
        "metric scalar."
    ),
    "joint_limits_on": (
        "Start from orientation_on, then restore the joint-limit RMP metric and gains."
    ),
    "joint_velocity_cap_on": (
        "Optional RB10 operating cap. Restore the joint-velocity-cap RMP after the "
        "core tutorial sequence if you want the solver-level speed limit back."
    ),
    "damping_on": (
        "Start from joint_limits_on, then restore the damping RMP including inertia."
    ),
}

CSPACE_KEYS = [
    "cspace_target_metric_scalar",
    "cspace_target_position_gain",
    "cspace_target_damping_gain",
    "cspace_target_robust_position_term_thresh",
]

TARGET_ISOTROPIC_RESTORE_KEYS = [
    "target_rmp_accel_p_gain",
    "target_rmp_accel_d_gain",
    "target_rmp_accel_norm_eps",
    "target_rmp_max_metric_scalar",
    "target_rmp_min_metric_scalar",
]

TARGET_DIRECTIONAL_KEYS = [
    "target_rmp_metric_alpha_length_scale",
    "target_rmp_min_metric_alpha",
    "target_rmp_proximity_metric_boost_scalar",
    "target_rmp_proximity_metric_boost_length_scale",
]

COLLISION_KEYS = [
    "collision_rmp_margin",
    "collision_rmp_damping_gain",
    "collision_rmp_damping_std_dev",
    "collision_rmp_damping_robustness_eps",
    "collision_rmp_damping_velocity_gate_length_scale",
    "collision_rmp_repulsion_gain",
    "collision_rmp_repulsion_std_dev",
    "collision_rmp_metric_modulation_radius",
    "collision_rmp_metric_scalar",
    "collision_rmp_metric_exploder_std_dev",
    "collision_rmp_metric_exploder_eps",
]

ORIENTATION_KEYS = [
    "orientation_rmp_accel_p_gain",
    "orientation_rmp_accel_d_gain",
    "orientation_rmp_metric_scalar",
]

JOINT_LIMIT_KEYS = [
    "joint_limit_metric_scalar",
    "joint_limit_metric_length_scale",
    "joint_limit_metric_exploder_eps",
    "joint_limit_metric_velocity_gate_length_scale",
    "joint_limit_accel_damper_gain",
    "joint_limit_accel_potential_gain",
    "joint_limit_accel_potential_exploder_eps",
    "joint_limit_accel_potential_exploder_length_scale",
]

JOINT_VELOCITY_CAP_KEYS = [
    "joint_velocity_cap_max_velocity",
    "joint_velocity_cap_velocity_damping_region",
    "joint_velocity_cap_damping_gain",
    "joint_velocity_cap_metric_weight",
    "max_joint_accel",
]

DAMPING_KEYS = [
    "damping_rmp_accel_d_gain",
    "damping_rmp_metric_scalar",
    "damping_rmp_inertia",
]


def project_root() -> Path:
    return Path(__file__).resolve().parents[1]


def default_base_params_path() -> Path:
    return project_root() / "config" / "params.yaml"


def default_scenario_path() -> Path:
    return project_root() / "config" / "tuning_scenarios" / "target_translation_baseline.yaml"


def default_root_dir() -> Path:
    return Path("~/ros2_ws/data/rmp_tuning_runs").expanduser()


def sanitize_name(name: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9._-]+", "_", name.strip()).strip("._-")
    if not cleaned:
        raise RuntimeError("name must contain at least one alphanumeric character")
    return cleaned


def load_yaml_mapping(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise RuntimeError(f"expected a YAML mapping in {path}")
    return data


def dump_yaml_mapping(path: Path, data: Dict[str, Any]) -> None:
    with path.open("w", encoding="utf-8") as handle:
        yaml.dump(
            data,
            handle,
            Dumper=ExperimentYamlDumper,
            sort_keys=False,
            default_flow_style=False,
            allow_unicode=False,
        )


def controller_params(doc: Dict[str, Any]) -> Dict[str, Any]:
    try:
        params = doc["rmpflow_controller"]["ros__parameters"]
    except KeyError as exc:
        raise RuntimeError(
            "params YAML must contain rmpflow_controller.ros__parameters"
        ) from exc
    if not isinstance(params, dict):
        raise RuntimeError("rmpflow_controller.ros__parameters must be a mapping")
    return params


def restore_keys(target: Dict[str, Any], source: Dict[str, Any], keys: Iterable[str]) -> None:
    for key in keys:
        if key not in source:
            raise RuntimeError(f"source params are missing required key: {key}")
        target[key] = copy.deepcopy(source[key])


def zero_all_rmp(params: Dict[str, Any]) -> None:
    params["cspace_target_metric_scalar"] = 0.0
    params["cspace_target_inertia"] = 0.0
    params["joint_limit_metric_scalar"] = 0.0
    params["joint_velocity_cap_metric_weight"] = 0.0
    params["target_rmp_min_metric_alpha"] = 0.0
    params["target_rmp_max_metric_scalar"] = 0.0
    params["target_rmp_min_metric_scalar"] = 0.0
    params["orientation_rmp_metric_scalar"] = 0.0
    params["collision_rmp_metric_scalar"] = 0.0
    params["damping_rmp_metric_scalar"] = 0.0
    params["damping_rmp_inertia"] = 0.0


def apply_preset(doc: Dict[str, Any], base_doc: Dict[str, Any], preset: str) -> None:
    params = controller_params(doc)
    base_params = controller_params(base_doc)

    if preset == "current":
        return

    if preset == "all_off":
        zero_all_rmp(params)
        return

    if preset == "cspace_only":
        apply_preset(doc, base_doc, "all_off")
        restore_keys(params, base_params, CSPACE_KEYS)
        params["cspace_target_inertia"] = 0.0
        return

    if preset == "target_isotropic":
        apply_preset(doc, base_doc, "cspace_only")
        restore_keys(params, base_params, TARGET_ISOTROPIC_RESTORE_KEYS)
        params["target_rmp_min_metric_alpha"] = 0.0
        params["target_rmp_metric_alpha_length_scale"] = 100000.0
        params["target_rmp_proximity_metric_boost_scalar"] = 1.0
        params["target_rmp_proximity_metric_boost_length_scale"] = 1.0
        return

    if preset == "collision_on":
        apply_preset(doc, base_doc, "target_isotropic")
        restore_keys(params, base_params, COLLISION_KEYS)
        return

    if preset == "target_directional":
        apply_preset(doc, base_doc, "collision_on")
        restore_keys(params, base_params, TARGET_DIRECTIONAL_KEYS)
        return

    if preset == "orientation_on":
        apply_preset(doc, base_doc, "target_directional")
        restore_keys(params, base_params, ORIENTATION_KEYS)
        return

    if preset == "joint_limits_on":
        apply_preset(doc, base_doc, "orientation_on")
        restore_keys(params, base_params, JOINT_LIMIT_KEYS)
        return

    if preset == "joint_velocity_cap_on":
        apply_preset(doc, base_doc, "joint_limits_on")
        restore_keys(params, base_params, JOINT_VELOCITY_CAP_KEYS)
        return

    if preset == "damping_on":
        apply_preset(doc, base_doc, "joint_limits_on")
        restore_keys(params, base_params, DAMPING_KEYS)
        return

    raise RuntimeError(f"unsupported preset: {preset}")


def write_text_file(path: Path, text: str, executable: bool = False) -> None:
    path.write_text(text, encoding="utf-8")
    if executable:
        current_mode = path.stat().st_mode
        path.chmod(current_mode | 0o111)


def read_existing_name(path: Path) -> str:
    manifest_path = path / "manifest.yaml"
    if manifest_path.exists():
        manifest = load_yaml_mapping(manifest_path)
        experiment_name = manifest.get("experiment_name")
        if isinstance(experiment_name, str) and experiment_name.strip():
            return experiment_name.strip()
    return path.name


def build_readme(
    experiment_dir: Path,
    experiment_name: str,
    preset: str,
    working_source_params: Path,
    preset_reference_params: Path,
    source_scenario: Path,
    clone_from: Optional[Path],
) -> str:
    compare_block = ""
    if clone_from is not None:
        compare_block = f"""
Compare the latest dataset against the parent run:

```bash
{experiment_dir / "compare_to_parent.sh"}
```
"""

    return f"""# {experiment_name}

Preset: `{preset}`

Description:
{PRESET_DESCRIPTIONS[preset]}

Files:
- `params.yaml`: experiment-specific controller parameters
- `scenario.yaml`: repeatable goal sequence
- `notes.md`: hypothesis, changed values, observations
- `results/datasets/`: recorder CSV output
- `results/joint_velocity_logs/`: joint-velocity text logs

Source files:
- working params source: `{working_source_params}`
- preset reference params: `{preset_reference_params}`
- scenario: `{source_scenario}`

Recommended preset order:
- `all_off`
- `cspace_only`
- `target_isotropic`
- `collision_on`
- `target_directional`
- `orientation_on`
- `joint_limits_on`
- `damping_on`

Optional RB10 operating preset:
- `joint_velocity_cap_on`

Move the real robot to the saved start pose before every run:

```bash
{experiment_dir / "move_to_start_pose.sh"}
```

If you manually place the robot at a new desired goal, capture that current
`/rmp_ee_pose` into the first absolute step with:

```bash
{experiment_dir / "capture_current_goal.sh"}
```

Launch the experiment:

```bash
{experiment_dir / "launch_experiment.sh"}
```

Run the fixed scenario once:

```bash
{experiment_dir / "run_scenario.sh"}
```

Analyze the newest recorder CSV:

```bash
{experiment_dir / "analyze_latest.sh"}
```
{compare_block}
Create the next variant from this run:

```bash
python3 {project_root() / "scripts" / "create_tuning_experiment.py"} \\
  --name next_variant \\
  --clone-from {experiment_dir} \\
  --preset current
```
"""


def build_notes_template(experiment_name: str, preset: str) -> str:
    return f"""# {experiment_name}

Preset: `{preset}`

## Hypothesis

## Changed Values

## Observations

## Decision
"""


def build_manifest(
    experiment_dir: Path,
    experiment_name: str,
    slug: str,
    preset: str,
    working_source_params: Path,
    preset_reference_params: Path,
    source_scenario: Path,
    clone_from: Optional[Path],
    notes: str,
) -> Dict[str, Any]:
    return {
        "experiment_name": experiment_name,
        "slug": slug,
        "preset": preset,
        "preset_description": PRESET_DESCRIPTIONS[preset],
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "working_source_params": str(working_source_params),
        "preset_reference_params": str(preset_reference_params),
        "source_scenario": str(source_scenario),
        "clone_from": str(clone_from) if clone_from is not None else None,
        "recommended_preset_order": RECOMMENDED_PRESET_ORDER,
        "notes": notes,
        "files": {
            "params": str(experiment_dir / "params.yaml"),
            "scenario": str(experiment_dir / "scenario.yaml"),
            "manifest": str(experiment_dir / "manifest.yaml"),
            "notes": str(experiment_dir / "notes.md"),
            "readme": str(experiment_dir / "README.md"),
            "datasets_dir": str(experiment_dir / "results" / "datasets"),
            "joint_velocity_dir": str(experiment_dir / "results" / "joint_velocity_logs"),
            "move_to_start_script": str(experiment_dir / "move_to_start_pose.sh"),
            "capture_goal_script": str(experiment_dir / "capture_current_goal.sh"),
            "launch_script": str(experiment_dir / "launch_experiment.sh"),
            "scenario_script": str(experiment_dir / "run_scenario.sh"),
            "analyze_script": str(experiment_dir / "analyze_latest.sh"),
        },
    }


def resolve_latest_compare_script(
    experiment_dir: Path,
    clone_from: Path,
    experiment_name: str,
    parent_name: str,
) -> str:
    analyzer = project_root() / "scripts" / "analyze_rmp_dataset.py"
    parent_dir = clone_from / "results" / "datasets"
    child_dir = experiment_dir / "results" / "datasets"
    return f"""#!/usr/bin/env bash
set -euo pipefail

parent_csv=$(ls -1t "{parent_dir}"/*.csv 2>/dev/null | head -n 1 || true)
child_csv=$(ls -1t "{child_dir}"/*.csv 2>/dev/null | head -n 1 || true)

if [[ -z "${{parent_csv}}" ]]; then
  echo "No CSV found in parent datasets directory: {parent_dir}" >&2
  exit 1
fi

if [[ -z "${{child_csv}}" ]]; then
  echo "No CSV found in child datasets directory: {child_dir}" >&2
  exit 1
fi

python3 "{analyzer}" "${{parent_csv}}" \\
  --compare-other "${{child_csv}}" \\
  --baseline-label "{parent_name}" \\
  --variant-label "{experiment_name}" "$@"
"""


def build_launch_script(experiment_dir: Path, slug: str) -> str:
    params_file = experiment_dir / "params.yaml"
    dataset_dir = experiment_dir / "results" / "datasets"
    joint_velocity_dir = experiment_dir / "results" / "joint_velocity_logs"
    return f"""#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
source /home/song/ros2_ws/install/setup.bash
set -u

ros2 launch rb10_rmpflow_rviz rb10_rmpflow_test.launch.py \\
  params_file:="{params_file}" \\
  record_data:=true \\
  recording_output_directory:="{dataset_dir}" \\
  recording_output_prefix:="{slug}" \\
  record_joint_velocity:=true \\
  joint_velocity_log_directory:="{joint_velocity_dir}" \\
  joint_velocity_log_prefix:="{slug}" \\
  "$@"
"""


def build_move_to_start_script(experiment_dir: Path) -> str:
    mover = project_root() / "scripts" / "move_to_saved_start_pose.py"
    params_file = experiment_dir / "params.yaml"
    return f"""#!/usr/bin/env bash
set -euo pipefail

python3 "{mover}" --params "{params_file}" "$@"
"""


def build_capture_goal_script(experiment_dir: Path) -> str:
    capturer = project_root() / "scripts" / "capture_goal_from_ee_pose.py"
    scenario_file = experiment_dir / "scenario.yaml"
    return f"""#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
source /home/song/ros2_ws/install/setup.bash
set -u

python3 "{capturer}" --scenario "{scenario_file}" "$@"
"""


def build_scenario_script(experiment_dir: Path) -> str:
    scenario_file = experiment_dir / "scenario.yaml"
    return f"""#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
source /home/song/ros2_ws/install/setup.bash
set -u

ros2 run rb10_rmpflow_rviz run_tuning_scenario.py \\
  --scenario "{scenario_file}" "$@"
"""


def build_analyze_script(experiment_dir: Path) -> str:
    analyzer = project_root() / "scripts" / "analyze_rmp_dataset.py"
    dataset_dir = experiment_dir / "results" / "datasets"
    return f"""#!/usr/bin/env bash
set -euo pipefail

python3 "{analyzer}" --latest-dir "{dataset_dir}" "$@"
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Create a self-contained RMP tuning experiment directory with a copied "
            "params.yaml, copied scenario.yaml, result folders, and helper scripts."
        )
    )
    parser.add_argument(
        "--name",
        required=True,
        help="Short experiment name used in the directory name.",
    )
    parser.add_argument(
        "--preset",
        default="current",
        choices=sorted(PRESET_DESCRIPTIONS.keys()),
        help="Preset that will be applied to the copied params.yaml.",
    )
    parser.add_argument(
        "--root-dir",
        default=str(default_root_dir()),
        help="Directory where experiment folders will be created.",
    )
    parser.add_argument(
        "--base-params",
        help="Source params.yaml. Defaults to clone-from/params.yaml or project config/params.yaml.",
    )
    parser.add_argument(
        "--scenario",
        help="Source scenario YAML. Defaults to clone-from/scenario.yaml or the baseline tuning scenario.",
    )
    parser.add_argument(
        "--clone-from",
        help="Existing experiment directory to clone before applying the preset.",
    )
    parser.add_argument(
        "--notes",
        default="",
        help="Optional note stored in manifest.yaml.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    clone_from = Path(args.clone_from).expanduser().resolve() if args.clone_from else None
    if clone_from is not None and not clone_from.is_dir():
        raise RuntimeError(f"--clone-from is not a directory: {clone_from}")

    working_source_params = (
        Path(args.base_params).expanduser().resolve()
        if args.base_params
        else ((clone_from / "params.yaml") if clone_from is not None else default_base_params_path())
    )
    preset_reference_params = (
        Path(args.base_params).expanduser().resolve()
        if args.base_params
        else default_base_params_path()
    )
    source_scenario = (
        Path(args.scenario).expanduser().resolve()
        if args.scenario
        else ((clone_from / "scenario.yaml") if clone_from is not None else default_scenario_path())
    )

    if not working_source_params.exists():
        raise RuntimeError(f"params source file does not exist: {working_source_params}")
    if not preset_reference_params.exists():
        raise RuntimeError(
            f"preset reference params file does not exist: {preset_reference_params}"
        )
    if not source_scenario.exists():
        raise RuntimeError(f"scenario source file does not exist: {source_scenario}")

    slug = sanitize_name(args.name)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    experiment_dir = Path(args.root_dir).expanduser().resolve() / f"{timestamp}_{slug}"
    experiment_dir.mkdir(parents=True, exist_ok=False)
    (experiment_dir / "results" / "datasets").mkdir(parents=True, exist_ok=True)
    (experiment_dir / "results" / "joint_velocity_logs").mkdir(parents=True, exist_ok=True)

    working_source_doc = load_yaml_mapping(working_source_params)
    preset_reference_doc = load_yaml_mapping(preset_reference_params)
    experiment_doc = copy.deepcopy(working_source_doc)
    apply_preset(experiment_doc, preset_reference_doc, args.preset)
    scenario_doc = load_yaml_mapping(source_scenario)

    dump_yaml_mapping(experiment_dir / "params.yaml", experiment_doc)
    dump_yaml_mapping(experiment_dir / "scenario.yaml", scenario_doc)

    write_text_file(
        experiment_dir / "notes.md",
        build_notes_template(args.name, args.preset),
    )

    manifest = build_manifest(
        experiment_dir=experiment_dir,
        experiment_name=args.name,
        slug=slug,
        preset=args.preset,
        working_source_params=working_source_params,
        preset_reference_params=preset_reference_params,
        source_scenario=source_scenario,
        clone_from=clone_from,
        notes=args.notes,
    )
    dump_yaml_mapping(experiment_dir / "manifest.yaml", manifest)

    write_text_file(
        experiment_dir / "README.md",
        build_readme(
            experiment_dir=experiment_dir,
            experiment_name=args.name,
            preset=args.preset,
            working_source_params=working_source_params,
            preset_reference_params=preset_reference_params,
            source_scenario=source_scenario,
            clone_from=clone_from,
        ),
    )
    write_text_file(
        experiment_dir / "move_to_start_pose.sh",
        build_move_to_start_script(experiment_dir),
        executable=True,
    )
    write_text_file(
        experiment_dir / "capture_current_goal.sh",
        build_capture_goal_script(experiment_dir),
        executable=True,
    )
    write_text_file(
        experiment_dir / "launch_experiment.sh",
        build_launch_script(experiment_dir, slug),
        executable=True,
    )
    write_text_file(
        experiment_dir / "run_scenario.sh",
        build_scenario_script(experiment_dir),
        executable=True,
    )
    write_text_file(
        experiment_dir / "analyze_latest.sh",
        build_analyze_script(experiment_dir),
        executable=True,
    )

    if clone_from is not None:
        write_text_file(
            experiment_dir / "compare_to_parent.sh",
            resolve_latest_compare_script(
                experiment_dir=experiment_dir,
                clone_from=clone_from,
                experiment_name=args.name,
                parent_name=read_existing_name(clone_from),
            ),
            executable=True,
        )

    print(f"Created experiment directory: {experiment_dir}")
    print(f"Preset: {args.preset}")
    print(f"Params copy: {experiment_dir / 'params.yaml'}")
    print(f"Scenario copy: {experiment_dir / 'scenario.yaml'}")
    print(f"Launch helper: {experiment_dir / 'launch_experiment.sh'}")
    print(f"Scenario helper: {experiment_dir / 'run_scenario.sh'}")
    print(f"Analyzer helper: {experiment_dir / 'analyze_latest.sh'}")
    if clone_from is not None:
        print(f"Parent compare helper: {experiment_dir / 'compare_to_parent.sh'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
