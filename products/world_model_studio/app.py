"""
World Model Studio - Streamlit UI.
Enhanced: Scene selector, Import from Foundry, Reality Bridge validation, training, batch, substrate.
"""

import json
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parent
PRODUCTS = ROOT.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(PRODUCTS) not in sys.path:
    sys.path.insert(0, str(PRODUCTS))
from shared.id_generator import artifact_id, run_id

import streamlit as st

from core.asset_library import list_assets, load_asset, import_from_foundry, list_foundry_designs
from core.scene_composer import SceneComposer
from core.scene_loader import list_scenes, load_scene, get_scene_objects
from core.task_builder import (
    TaskDefinition,
    TaskBuilder,
    PICK,
    PLACE,
    HANDOVER,
    PICK_EGG,
    PICK_BOX,
    HOLD,
    PICK_AND_PLACE,
    SORT,
    STACK,
    INSERT,
    POUR,
    compute_reward,
)
from core.simulator import Simulator
from core.trainer import (
    random_policy,
    run_episode,
    evaluate_policy,
    random_search,
    train_loop,
)
from core.policies import LinearPolicy, ConstantPolicy
from core.batch import create_batch_job, run_batch_job, get_batch_results, find_best_parameters, list_jobs

try:
    from substrate_integration import (
        record_training_to_substrate,
        find_similar_training,
        get_training_lineage,
        record_scene_to_substrate,
    )
except Exception:
    record_training_to_substrate = None
    find_similar_training = None
    get_training_lineage = None
    record_scene_to_substrate = None

st.set_page_config(page_title="World Model Studio", layout="wide")

# Session state defaults
if "validation_passed" not in st.session_state:
    st.session_state.validation_passed = None
if "validation_result" not in st.session_state:
    st.session_state.validation_result = None
if "episode_success_count" not in st.session_state:
    st.session_state.episode_success_count = 0
if "episode_total_count" not in st.session_state:
    st.session_state.episode_total_count = 0


def _validate_reality_bridge(mjcf: str) -> dict:
    """POST MJCF to Reality Bridge /validate. Returns response dict or error dict."""
    try:
        import requests
        r = requests.post(
            "http://localhost:8000/validate",
            data={"xml_string": mjcf},
            timeout=10,
        )
        if r.status_code == 200:
            return r.json()
        return {"success": False, "passed": False, "message": f"HTTP {r.status_code}"}
    except Exception as e:
        return {"success": False, "passed": False, "message": str(e)}


def _task_from_scene(scene_name: str, scene_path: str):
    """Build TaskDefinition from scene template (index.json task_type)."""
    scenes = list_scenes()
    task_type = "pick_and_place"
    for s in scenes:
        if s.get("name") == scene_name or s.get("id") == scene_name:
            task_type = (s.get("task_type") or "pick_and_place").lower()
            break
    if "pick_and_place" in task_type or "pick_and_place" in scene_name:
        return TaskBuilder.pick_and_place_task(scene_path)
    if "bin" in task_type or "bin_picking" in scene_name:
        return TaskBuilder.pick_task("bin_pick", scene_path, "object_box", 0.1)
    if "conveyor" in task_type or "conveyor" in scene_name:
        return TaskBuilder.pick_task("conveyor_pick", scene_path, "object_box", 0.1)
    if "assembly" in task_type or "assembly" in scene_name:
        return TaskBuilder.insert_task(scene_path)
    if "inspection" in task_type or "inspection" in scene_name:
        return TaskBuilder.pick_task("inspect", scene_path, "object_box", 0.08)
    return TaskBuilder.pick_and_place_task(scene_path)


def main():
    st.sidebar.title("World Model Studio")
    page = st.sidebar.radio("Page", ["Main (Studio)", "Scene Builder", "Task Designer", "Simulator", "Training (legacy)"], key="main_page")
    if page != "Main (Studio)":
        if page == "Scene Builder":
            render_scene_builder()
        elif page == "Task Designer":
            render_task_designer()
        elif page == "Simulator":
            render_simulator()
        else:
            render_training()
        return

    st.title("WORLD MODEL STUDIO")
    st.markdown("---")

    # ┌─────────────────────────────────────────────────────────────┐
    # │ Scene: [Pick and Place ▼]    Gripper: [Import from Foundry] │
    # └─────────────────────────────────────────────────────────────┘
    row1 = st.columns([1, 1])
    with row1[0]:
        scenes_list = list_scenes()
        scene_options = [s.get("name") or s.get("id", "?") for s in scenes_list]
        if not scene_options:
            scene_options = ["pick_and_place"]
        scene_name = st.selectbox("Scene", scene_options, key="main_scene")
        scene_path = None
        scene_data = None
        if scene_name:
            scene_data = load_scene(scene_name)
            if scene_data and scene_data.get("path"):
                scene_path = scene_data["path"]

        # Also allow custom scene file
        scenes_dir = ROOT / "scenes"
        scene_files = list(scenes_dir.glob("*.xml")) + list(scenes_dir.glob("*.mjcf")) if scenes_dir.exists() else []
        custom_paths = [str(p) for p in scene_files if p.stem != scene_name]
        if custom_paths:
            custom = st.selectbox("Or custom scene file", [""] + custom_paths, key="custom_scene")
            if custom:
                scene_path = custom

        if not scene_path and scene_data and scene_data.get("path"):
            scene_path = scene_data["path"]
        if not scene_path and scene_name:
            scene_path = str(ROOT / "scenes" / f"{scene_name}.xml")
            if not Path(scene_path).exists():
                scene_path = str(ROOT / "scenes" / f"{scene_name}.mjcf")

    with row1[1]:
        st.subheader("Gripper")
        foundry_designs = list_foundry_designs()
        local_grippers = list_assets("grippers") or []
        gripper_source = st.radio("Gripper source", ["Local assets", "Import from Foundry"], key="gripper_src", horizontal=True)
        selected_gripper_name = None
        if gripper_source == "Import from Foundry":
            if foundry_designs:
                design_labels = [f"{d['name']} ({d['id']})" for d in foundry_designs]
                idx = st.selectbox("Select Foundry design", range(len(design_labels)), format_func=lambda i: design_labels[i], key="foundry_sel")
                if st.button("Import selected"):
                    try:
                        path = import_from_foundry(foundry_designs[idx]["path"])
                        st.success(f"Imported to {path}")
                        selected_gripper_name = Path(path).stem
                    except Exception as e:
                        st.error(str(e))
                # Show imported name for reference
                if idx is not None:
                    st.caption(f"Path: {foundry_designs[idx]['path']}")
            else:
                st.info("No Foundry designs found in omega_foundry/outputs. Create a design in Omega Foundry first.")
        else:
            selected_gripper_name = st.selectbox("Local gripper", ["(none)"] + local_grippers, key="local_gripper")

    st.markdown("---")

    # Load scene path for rest of UI
    path = scene_path
    if not path and scene_name:
        p = ROOT / "scenes" / f"{scene_name}.xml"
        path = str(p) if p.exists() else str(ROOT / "scenes" / f"{scene_name}.mjcf") if (ROOT / "scenes" / f"{scene_name}.mjcf").exists() else None

    if not path:
        st.info("Select a scene (or add scene XML in products/world_model_studio/scenes/).")
        return

    # Build task from scene
    task = _task_from_scene(scene_name, path)
    if task:
        task.scene_path = path

    # ┌────────────────────────┐  ┌────────────────────────────┐
    # │   [3D Scene Preview]    │  │ Task: Pick and Place       │
    # │                         │  │ Difficulty: Medium        │
    # │                         │  │ Objects: cube, sphere      │
    # └────────────────────────┘  │ Success: 0/10               │
    #                             │ [▶ Run Episode]             │
    # [Record Video]              │ [🔄 Train (10 episodes)]     │
    #                             │ [📊 Batch Optimize]          │
    #                             └────────────────────────────┘
    col_left, col_right = st.columns([1, 1])
    with col_left:
        st.subheader("Scene Preview")
        if scene_data and scene_data.get("mjcf"):
            with st.expander("View MJCF", expanded=False):
                st.text_area("MJCF", scene_data["mjcf"][:8000], height=200, key="mjcf_preview")
        else:
            try:
                with open(path, "r", encoding="utf-8") as f:
                    mjcf_preview = f.read()
                with st.expander("View MJCF", expanded=False):
                    st.text_area("MJCF", mjcf_preview[:8000], height=200, key="mjcf_preview2")
            except Exception:
                st.caption("Scene loaded. Use Run Episode or Record Video to see simulation.")

        st.subheader("Validate with Reality Bridge")
        validate_mjcf = None
        if scene_data and scene_data.get("mjcf"):
            validate_mjcf = scene_data["mjcf"]
        else:
            try:
                validate_mjcf = Path(path).read_text(encoding="utf-8")
            except Exception:
                pass
        if validate_mjcf:
            if st.button("Validate with Reality Bridge"):
                res = _validate_reality_bridge(validate_mjcf)
                st.session_state.validation_result = res
                st.session_state.validation_passed = res.get("passed", res.get("success", False))
                st.rerun()
            if st.session_state.validation_result is not None:
                res = st.session_state.validation_result
                passed = res.get("passed", res.get("success", False))
                if passed:
                    st.success(f"Validation passed. Score: {res.get('score', 0):.2f}")
                else:
                    st.error("Validation failed. Fix design before training.")
                    if res.get("failures"):
                        for f in res["failures"]:
                            st.warning(f.get("message", str(f)))
        else:
            st.caption("Load a scene to validate.")

        st.subheader("Record Video")
        record_steps = st.number_input("Steps", 50, 2000, 500, key="record_steps")
        if st.button("Record Episode"):
            try:
                sim_rec = Simulator()
                sim_rec.load_scene(path=path)
                sim_rec.set_task(task or SimpleNamespace(max_steps=record_steps, goal="pick", success_criteria={}))
                videos_dir = ROOT / "videos"
                videos_dir.mkdir(parents=True, exist_ok=True)
                out_path = videos_dir / f"episode_{datetime.now().strftime('%Y%m%d_%H%M%S')}.mp4"
                result = sim_rec.record_episode(random_policy, task or SimpleNamespace(max_steps=record_steps, goal="pick", success_criteria={}), str(out_path), max_steps=record_steps)
                st.metric("Total reward", f"{result['total_reward']:.4f}")
                st.metric("Success", result["success"])
                if result.get("video_path"):
                    st.video(result["video_path"])
            except Exception as e:
                st.error(str(e))

    with col_right:
        st.subheader("Task")
        if task:
            st.write(f"**Task:** {task.name}")
            difficulty = "medium"
            if scene_data and scene_data.get("config"):
                difficulty = scene_data["config"].get("difficulty", "medium")
            st.write(f"**Difficulty:** {difficulty}")
            objs = get_scene_objects(scene_name) if scene_name else []
            st.write(f"**Objects:** {', '.join(objs) if objs else '—'}")
            st.write(f"**Success:** {st.session_state.episode_success_count}/{st.session_state.episode_total_count}")

        if st.button("▶ Run Episode"):
            try:
                sim = Simulator()
                sim.load_scene(path=path)
                sim.set_task(task or SimpleNamespace(max_steps=500, goal="pick", success_criteria={"min_height": 0.1}))
                out = run_episode(sim, random_policy, task or SimpleNamespace(max_steps=500, goal="pick", success_criteria={"min_height": 0.1}), use_task_reward=True)
                st.session_state.episode_total_count = st.session_state.get("episode_total_count", 0) + 1
                if out["success"]:
                    st.session_state.episode_success_count = st.session_state.get("episode_success_count", 0) + 1
                st.metric("Episode reward", f"{out['total_reward']:.4f}")
                st.metric("Success", out["success"])
                st.rerun()
            except Exception as e:
                st.error(str(e))

        if find_similar_training and task:
            similar = find_similar_training(task.name, selected_gripper_name or "default", n=5)
            if similar:
                with st.expander("Similar past runs (substrate)"):
                    for s in similar:
                        st.caption(str(s.get("metadata") or s.get("id", "")))

        st.subheader("Train")
        # Block training if validation failed (when user has run validation)
        allow_training = True
        if st.session_state.validation_passed is False:
            allow_training = False
            st.warning("Reality Bridge validation failed. Validate and pass before training.")
        n_episodes_train = st.number_input("Episodes", 5, 100, 10, key="n_ep_train")
        algo = st.selectbox("Algorithm", ["random_search", "grid_search", "simple_rl", "evaluate_only"], key="algo")
        if st.button("🔄 Train", disabled=not allow_training) and path and task:
            try:
                sim = Simulator()
                sim.load_scene(path=path)
                config = {
                    "n_iterations": 50,
                    "n_episodes_per_eval": min(5, n_episodes_train),
                    "n_params": 10,
                    "early_stopping_patience": 15,
                    "checkpoint_path": str(ROOT / "outputs" / "best_policy.json"),
                }
                if algo == "grid_search":
                    config["param_grid"] = {"scale": [0.05, 0.1], "n_params": [5, 10]}
                if algo == "simple_rl":
                    config["n_steps"] = 100
                    config["lr"] = 0.01
                if algo == "evaluate_only":
                    config["n_episodes"] = n_episodes_train
                progress = st.progress(0.0)
                status = st.empty()
                status.info("Training...")
                result = train_loop(sim, task, method=algo, config=config)
                progress.progress(1.0)
                status.success("Training complete.")
                st.session_state.training_result = result
                st.session_state.training_task = task
                st.session_state.training_path = path
                st.session_state.training_best_weights = result.get("best_weights")
                # Substrate
                if record_training_to_substrate and result.get("run_id"):
                    gripper_id = selected_gripper_name or "default"
                    policy_id = f"policy_{result.get('run_id', run_id())}"
                    record_training_to_substrate(
                        result["run_id"],
                        gripper_id,
                        scene_name or "unknown",
                        {"success_rate": result.get("evaluation", {}).get("success_rate"), "avg_reward": result.get("evaluation", {}).get("avg_reward"), "best_reward": result.get("best_reward")},
                        policy_id=policy_id,
                    )
                    st.session_state.last_policy_id = policy_id
                st.rerun()
            except Exception as e:
                st.error(str(e))

        st.subheader("Batch Optimize")
        n_trials_batch = st.number_input("Trials", 5, 100, 10, key="batch_trials")
        if st.button("📊 Create batch job"):
            try:
                job_id = create_batch_job("pick_and_place", {"scale": [0.05, 0.1], "n_params": [5, 10]}, n_trials=n_trials_batch)
                st.session_state.last_batch_job_id = job_id
                st.success(f"Created job {job_id}")
            except Exception as e:
                st.error(str(e))
        batch_jobs = list_jobs(10)
        if batch_jobs:
            job_sel = st.selectbox("Batch job", [j["id"] for j in batch_jobs], key="batch_job_sel")
            if job_sel:
                res = get_batch_results(job_sel)
                if res.get("found"):
                    st.metric("Status", res["job"].get("status"))
                    st.metric("Trials", len(res.get("trials", [])))
                    best = find_best_parameters(job_sel)
                    if best.get("best_score") is not None:
                        st.metric("Best score", f"{best['best_score']:.4f}")
                    if st.button("Run this batch job") and path and task:
                        def run_fn(sim, t, params):
                            return run_episode(sim, random_policy, t, use_task_reward=True)
                        run_batch_job(job_sel, run_fn, path, task)
                        st.success("Batch run finished.")
                        st.rerun()

    st.markdown("---")
    st.subheader("Training Progress & Results")
    if st.session_state.get("training_result"):
        res = st.session_state.training_result
        hist = res.get("history") or []
        if hist:
            st.line_chart({"reward": hist})
        st.metric("Best reward", f"{res.get('best_reward', 0):.4f}" if res.get("best_reward") is not None else "—")
        if res.get("evaluation"):
            ev = res["evaluation"]
            st.metric("Success rate", f"{ev.get('success_rate', 0):.2%}")
        st.metric("Episode", f"{len(hist)}/{(res.get('config') or {}).get('n_iterations', '?')}")

        col1, col2, col3 = st.columns(3)
        with col1:
            if st.button("View Results (expand)"):
                st.json({k: v for k, v in res.items() if k != "best_weights"})
        with col2:
            if st.button("Export Policy") and st.session_state.get("training_best_weights") is not None:
                try:
                    out_dir = ROOT / "outputs"
                    out_dir.mkdir(parents=True, exist_ok=True)
                    policy_path = out_dir / f"policy_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
                    with open(policy_path, "w") as f:
                        json.dump({"weights": st.session_state.training_best_weights, "obs_dim": res.get("obs_dim"), "act_dim": res.get("act_dim")}, f)
                    st.success(f"Exported to {policy_path}")
                    st.download_button("Download policy JSON", policy_path.read_text(), file_name=policy_path.name, mime="application/json")
                except Exception as e:
                    st.error(str(e))
        with col3:
            if st.button("Save to Substrate") and record_training_to_substrate and st.session_state.get("training_result"):
                st.info("Training run already recorded to substrate when training completed.")
            if get_training_lineage and st.session_state.get("last_policy_id"):
                lineage = get_training_lineage(st.session_state.last_policy_id)
                if lineage:
                    with st.expander("Policy lineage"):
                        st.json(lineage)


def render_scene_builder():
    st.header("Scene Builder")
    surfaces = list_assets("surfaces")
    objects = list_assets("objects")
    grippers = list_assets("grippers")
    col1, col2 = st.columns([1, 1])
    with col1:
        surface_name = st.selectbox("Surface", surfaces or ["table"], index=0)
        surface_pos = st.slider("Surface Z", 0.0, 0.2, 0.0, 0.01, key="surf_z")
        gripper_name = st.selectbox("Gripper", ["(none)"] + (grippers or []), key="gripper")
        gripper_pos_z = st.slider("Gripper Z", 0.1, 0.5, 0.3, 0.01, key="grip_z")
        obj_name = st.selectbox("Object", objects or ["box"], key="obj")
        obj_pos_z = st.slider("Object Z", 0.02, 0.15, 0.05, 0.01, key="obj_z")
        obj_mass = st.slider("Object mass (kg)", 0.01, 1.0, 0.1, 0.01, key="mass")
    composer = SceneComposer()
    composer.add_surface(surface_name or "table", [0, 0, surface_pos])
    if gripper_name and gripper_name != "(none)":
        try:
            composer.add_gripper(gripper_name, [0, 0, gripper_pos_z])
        except FileNotFoundError:
            st.warning(f"Gripper '{gripper_name}' not found. Add MJCF to assets/grippers/ or import from Foundry.")
    composer.add_object(obj_name or "box", [0, 0, obj_pos_z], obj_mass)
    composer.set_camera([0.8, 0.6, 0.5], [0, 0, 0.1])
    with col2:
        if st.button("Preview MJCF"):
            mjcf = composer.compose()
            st.text_area("Composed MJCF", mjcf, height=300)
        if st.button("Validate with Reality Bridge"):
            try:
                import requests
                mjcf = composer.compose()
                r = requests.post("http://localhost:8000/validate", data={"xml_string": mjcf}, timeout=5)
                if r.status_code == 200:
                    res = r.json()
                    st.json(res)
                    st.success(f"Passed: {res.get('passed', res.get('success', False))}, Score: {res.get('score', 0):.2f}")
                else:
                    st.error(f"Validation failed: {r.status_code}")
            except Exception as e:
                st.warning(f"Reality Bridge not reachable: {e}. Start reality_bridge app first.")
    save_path = st.text_input("Save scene to", str(ROOT / "scenes" / f"{artifact_id()}.xml"))
    if st.button("Save Scene") and save_path:
        composer.save(save_path)
        st.success(f"Saved to {save_path}")
    st.sidebar.subheader("Import from Foundry")
    foundry_designs = list_foundry_designs()
    if foundry_designs:
        design_labels = [f"{d['name']} - {d['path']}" for d in foundry_designs]
        idx = st.sidebar.selectbox("Foundry design", range(len(design_labels)), format_func=lambda i: design_labels[i])
        if st.sidebar.button("Import"):
            try:
                out = import_from_foundry(foundry_designs[idx]["path"])
                st.sidebar.success(f"Imported to {out}")
            except Exception as e:
                st.sidebar.error(str(e))
    else:
        st.sidebar.caption("No Foundry designs. Enter path below.")
        foundry_path = st.sidebar.text_input("Foundry MJCF path", str(PRODUCTS / "omega_foundry" / "outputs" / "OF_20260129_0001" / "design.mjcf"))
        if st.sidebar.button("Import from path"):
            try:
                out = import_from_foundry(foundry_path)
                st.sidebar.success(f"Imported to {out}")
            except Exception as e:
                st.sidebar.error(str(e))


def render_task_designer():
    st.header("Task Designer")
    scenes_list = list_scenes()
    scene_options = [s.get("name") or s.get("id", "?") for s in scenes_list]
    scene_files = list((ROOT / "scenes").glob("*.xml")) + list((ROOT / "scenes").glob("*.mjcf")) if (ROOT / "scenes").exists() else []
    scene_path = st.selectbox("Scene", [str(p) for p in scene_files] + scene_options + [""], key="task_scene")
    goal = st.selectbox("Goal", [PICK, PLACE, HANDOVER, PICK_AND_PLACE, SORT, STACK, INSERT, POUR], key="goal")
    target_object = st.text_input("Target object body/geom name", "object_box", key="target_obj")
    max_steps = st.number_input("Max steps", 100, 2000, 500, key="max_steps")
    reward_type = st.selectbox("Reward type", ["sparse", "dense"], key="reward_type")
    if goal == PICK:
        height_threshold = st.slider("Pick height threshold (m)", 0.05, 0.3, 0.1, 0.01)
        success_criteria = {"object_z_min": height_threshold}
    elif goal in (PLACE, PICK_AND_PLACE, INSERT):
        tx = st.number_input("Target X", -0.5, 0.5, 0.0, 0.05)
        ty = st.number_input("Target Y", -0.5, 0.5, 0.0, 0.05)
        tz = st.number_input("Target Z", 0.05, 0.5, 0.15, 0.01)
        success_criteria = {"target_pos": [tx, ty, tz], "tolerance": 0.05}
    else:
        success_criteria = {"handover_zone": [0.0, 0.0, 0.2]} if goal == HANDOVER else {"min_height": 0.1}
    task_name = st.text_input("Task name", "my_task", key="task_name")
    if st.button("Save Task"):
        if not scene_path:
            st.error("Select a scene")
        else:
            task = TaskDefinition(name=task_name, scene_path=scene_path, goal=goal, target_object=target_object, success_criteria=success_criteria, max_steps=max_steps, reward_type=reward_type)
            path = TaskBuilder.save_task(task)
            st.success(f"Saved to {path}")


def render_simulator():
    st.header("Simulator")
    scenes_dir = ROOT / "scenes"
    scene_files = list(scenes_dir.glob("*.xml")) + list(scenes_dir.glob("*.mjcf")) if scenes_dir.exists() else []
    scene_path = st.selectbox("Scene file", [""] + [str(p) for p in scene_files], key="sim_scene")
    load_path = st.text_input("Or path", "", key="sim_path")
    path = load_path or (scene_path if scene_path else None)
    if path:
        try:
            sim = Simulator()
            sim.load_scene(path=path)
            st.success("Scene loaded")
            if st.button("Reset"):
                obs = sim.reset()
                st.session_state["sim_obs"] = obs
                st.json({k: (v.tolist() if hasattr(v, "tolist") else v) for k, v in obs.items()})
            nu = sim.model.nu if sim.model else 0
            if nu > 0:
                st.subheader("Manual control")
                ctrls = [st.slider(f"Actuator {i}", -1.0, 1.0, 0.0, 0.05, key=f"act_{i}") for i in range(min(nu, 8))]
                if st.button("Step"):
                    action = ctrls + [0.0] * (nu - len(ctrls)) if len(ctrls) < nu else ctrls[:nu]
                    obs, reward, done, info = sim.step(action)
                    st.session_state["sim_obs"] = obs
                    st.metric("Reward", f"{reward:.4f}")
                    st.json({k: (v.tolist() if hasattr(v, "tolist") else v) for k, v in obs.items()})
            if st.button("Run random policy (10 steps)"):
                sim2 = Simulator()
                sim2.load_scene(path=path)
                obs = sim2.reset()
                total = 0.0
                for _ in range(10):
                    action = random_policy(obs)
                    obs, reward, done, info = sim2.step(action)
                    total += reward
                st.metric("Total reward (10 steps)", f"{total:.4f}")
            st.subheader("Viewer & recording")
            if st.button("Launch Viewer"):
                try:
                    subprocess.Popen([sys.executable, "-c", f"import sys; sys.path.insert(0, {repr(str(ROOT))}); from core.viewer import launch_viewer; launch_viewer(scene_path={repr(path)})"], cwd=str(ROOT))
                    st.success("Viewer launched in a separate window.")
                except Exception as e:
                    st.error(str(e))
            record_steps = st.number_input("Record episode steps", 50, 2000, 500, key="record_steps")
            if st.button("Record Episode"):
                try:
                    sim_rec = Simulator()
                    sim_rec.load_scene(path=path)
                    dummy_task = SimpleNamespace(max_steps=record_steps, goal="pick", success_criteria={})
                    videos_dir = ROOT / "videos"
                    videos_dir.mkdir(parents=True, exist_ok=True)
                    out_path = videos_dir / f"episode_{datetime.now().strftime('%Y%m%d_%H%M%S')}.mp4"
                    result = sim_rec.record_episode(random_policy, dummy_task, str(out_path), max_steps=record_steps)
                    st.metric("Total reward", f"{result['total_reward']:.4f}")
                    st.metric("Success", result["success"])
                    if result.get("video_path"):
                        st.video(result["video_path"])
                except Exception as e:
                    st.error(str(e))
        except Exception as e:
            st.error(str(e))
    else:
        st.info("Select or enter a scene path to load.")


def render_training():
    st.header("Training (legacy)")
    scenes_dir = ROOT / "scenes"
    scene_files = list(scenes_dir.glob("*.xml")) + list(scenes_dir.glob("*.mjcf")) if scenes_dir.exists() else []
    scene_path = st.selectbox("Scene", [""] + [str(p) for p in scene_files], key="train_scene")
    scene_path_override = st.text_input("Or scene path", "", key="train_scene_override")
    path = (scene_path_override or scene_path) if (scene_path_override or scene_path) else None
    task_source = st.radio("Task source", ["Saved task", "Built-in task"], key="task_src")
    tasks_dir = ROOT / "tasks"
    task_files = list(tasks_dir.glob("*.json")) if tasks_dir.exists() else []
    task = None
    if task_source == "Saved task":
        task_path = st.selectbox("Task file", [""] + [str(p) for p in task_files], key="train_task")
        if task_path:
            try:
                task = TaskBuilder.load_task(task_path)
                if path:
                    task.scene_path = path
                else:
                    path = task.scene_path
            except Exception as e:
                st.error(str(e))
    else:
        builtin = st.selectbox("Built-in task", ["PICK_EGG", "PICK_BOX", "HOLD", "PICK_AND_PLACE", "SORT", "STACK", "INSERT", "POUR"], key="builtin_task")
        if path:
            if builtin == "PICK_EGG":
                task = TaskBuilder.pick_egg_task(path)
            elif builtin == "PICK_BOX":
                task = TaskBuilder.pick_box_task(path)
            elif builtin == "HOLD":
                task = TaskBuilder.hold_task(path, hold_steps=100)
            elif builtin == "PICK_AND_PLACE":
                task = TaskBuilder.pick_and_place_task(path)
            elif builtin == "SORT":
                task = TaskBuilder.sort_task(path)
            elif builtin == "STACK":
                task = TaskBuilder.stack_task(path)
            elif builtin == "INSERT":
                task = TaskBuilder.insert_task(path)
            elif builtin == "POUR":
                task = TaskBuilder.pour_task(path)
            else:
                task = TaskBuilder.pick_box_task(path)
    method = st.selectbox("Training method", ["random_search", "grid_search", "simple_rl", "evaluate_only"], key="train_method")
    n_iterations = st.number_input("Iterations (random_search)", 10, 500, 100, key="n_iter")
    n_episodes_per_eval = st.number_input("Episodes per evaluation", 1, 20, 5, key="n_ep_eval")
    n_episodes_final = st.number_input("Episodes for final eval", 1, 50, 10, key="n_ep_final")
    if st.button("Start Training") and path and task:
        try:
            sim = Simulator()
            sim.load_scene(path=path)
            config = {"n_iterations": n_iterations, "n_episodes_per_eval": n_episodes_per_eval, "n_params": 10, "early_stopping_patience": 15, "checkpoint_path": str(ROOT / "outputs" / "best_policy.json")}
            if method == "grid_search":
                config["param_grid"] = {"scale": [0.05, 0.1], "n_params": [5, 10]}
            if method == "simple_rl":
                config["n_steps"] = 200
                config["lr"] = 0.01
            if method == "evaluate_only":
                config["n_episodes"] = n_episodes_final
            progress = st.progress(0.0)
            status = st.empty()
            status.info("Training...")
            result = train_loop(sim, task, method=method, config=config)
            progress.progress(1.0)
            status.success("Training complete.")
            st.session_state.training_result = result
            st.session_state.training_task = task
            st.session_state.training_path = path
            st.session_state.training_best_weights = result.get("best_weights")
            st.subheader("Results")
            if result.get("history"):
                st.line_chart({"reward": result["history"]})
            st.metric("Best reward", f"{result.get('best_reward', 0):.4f}" if result.get("best_reward") is not None else "—")
            if result.get("evaluation"):
                ev = result["evaluation"]
                st.metric("Success rate", f"{ev.get('success_rate', 0):.2%}")
            if result.get("log_path"):
                st.caption(f"Log: {result['log_path']}")
        except Exception as e:
            st.error(str(e))
    if st.button("Record Best Policy") and path and st.session_state.get("training_best_weights") is not None:
        try:
            res = st.session_state.get("training_result", {})
            task = st.session_state.get("training_task")
            if task is None:
                task = TaskDefinition(name="eval", scene_path=path, goal=PICK, target_object="object_box", success_criteria={"min_height": 0.1}, max_steps=500)
            sim = Simulator()
            sim.load_scene(path=path)
            obs0 = sim.reset()
            obs_dim = res.get("obs_dim") or (len(obs0.get("qpos", [])) + len(obs0.get("qvel", [])))
            act_dim = res.get("act_dim") or (int(obs0.get("nu", 1)) or 1)
            weights = st.session_state["training_best_weights"]
            policy = LinearPolicy(obs_dim, act_dim, weights=weights)
            videos_dir = ROOT / "videos"
            videos_dir.mkdir(parents=True, exist_ok=True)
            out_path = videos_dir / f"best_policy_{datetime.now().strftime('%Y%m%d_%H%M%S')}.mp4"
            rec = sim.record_episode(policy, task, str(out_path), max_steps=task.max_steps)
            st.metric("Recorded reward", f"{rec['total_reward']:.4f}")
            if rec.get("video_path"):
                st.video(rec["video_path"])
        except Exception as e:
            st.error(str(e))


if __name__ == "__main__":
    main()
