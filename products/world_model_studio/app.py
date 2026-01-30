"""
World Model Studio - Streamlit UI.
Pages: Scene Builder, Task Designer, Simulator, Training.
"""

import subprocess
import sys
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import streamlit as st

from core.asset_library import list_assets, load_asset, import_from_foundry
from core.scene_composer import SceneComposer
from core.task_builder import TaskDefinition, TaskBuilder, PICK, PLACE, HANDOVER, PICK_EGG, PICK_BOX, HOLD, compute_reward
from core.simulator import Simulator
from core.trainer import random_policy, run_episode, train_random_search, evaluate_policy, random_search, train_loop
from core.policies import LinearPolicy, ConstantPolicy, GraspSequencePolicy

st.set_page_config(page_title="World Model Studio", layout="wide")

PAGES = {
    "Scene Builder": "scene_builder",
    "Task Designer": "task_designer",
    "Simulator": "simulator",
    "Training": "training",
}

def main():
    st.sidebar.title("World Model Studio")
    page = st.sidebar.radio("Page", list(PAGES.keys()))
    if page == "Scene Builder":
        render_scene_builder()
    elif page == "Task Designer":
        render_task_designer()
    elif page == "Simulator":
        render_simulator()
    else:
        render_training()


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
                    st.success(f"Passed: {res.get('passed', False)}, Score: {res.get('score', 0):.2f}")
                else:
                    st.error(f"Validation failed: {r.status_code}")
            except Exception as e:
                st.warning(f"Reality Bridge not reachable: {e}. Start reality_bridge app first.")

    save_path = st.text_input("Save scene to", str(ROOT / "scenes" / "my_scene.xml"))
    if st.button("Save Scene") and save_path:
        composer.save(save_path)
        st.success(f"Saved to {save_path}")

    st.sidebar.subheader("Import from Foundry")
    foundry_path = st.sidebar.text_input("Foundry MJCF path", str(ROOT.parent / "omega_foundry" / "outputs" / "OF_20260129_0001" / "design.mjcf"))
    if st.sidebar.button("Import"):
        try:
            out = import_from_foundry(foundry_path)
            st.sidebar.success(f"Imported to {out}")
        except Exception as e:
            st.sidebar.error(str(e))


def render_task_designer():
    st.header("Task Designer")
    scenes_dir = ROOT / "scenes"
    scene_files = list(scenes_dir.glob("*.xml")) + list(scenes_dir.glob("*.mjcf")) if scenes_dir.exists() else []
    scene_path = st.selectbox("Scene", [str(p) for p in scene_files] + [""], key="task_scene")
    goal = st.selectbox("Goal", [PICK, PLACE, HANDOVER], key="goal")
    target_object = st.text_input("Target object body/geom name", "object_box", key="target_obj")
    max_steps = st.number_input("Max steps", 100, 2000, 500, key="max_steps")
    reward_type = st.selectbox("Reward type", ["sparse", "dense"], key="reward_type")

    if goal == PICK:
        height_threshold = st.slider("Pick height threshold (m)", 0.05, 0.3, 0.1, 0.01)
        success_criteria = {"object_z_min": height_threshold}
    elif goal == PLACE:
        tx = st.number_input("Target X", -0.5, 0.5, 0.0, 0.05)
        ty = st.number_input("Target Y", -0.5, 0.5, 0.0, 0.05)
        tz = st.number_input("Target Z", 0.05, 0.5, 0.15, 0.01)
        success_criteria = {"target_pos": [tx, ty, tz], "tolerance": 0.05}
    else:
        success_criteria = {"handover_zone": [0.0, 0.0, 0.2]}

    task_name = st.text_input("Task name", "my_task", key="task_name")
    if st.button("Save Task"):
        if not scene_path:
            st.error("Select a scene")
        else:
            task = TaskDefinition(
                name=task_name,
                scene_path=scene_path,
                goal=goal,
                target_object=target_object,
                success_criteria=success_criteria,
                max_steps=max_steps,
                reward_type=reward_type,
            )
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
                    subprocess.Popen(
                        [sys.executable, "-c",
                         f"import sys; sys.path.insert(0, {repr(str(ROOT))}); from core.viewer import launch_viewer; launch_viewer(scene_path={repr(path)})"],
                        cwd=str(ROOT),
                    )
                    st.success("Viewer launched in a separate window. Close the window when done.")
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
                        st.caption("Recorded video")
                        preview = result.get("preview_frames", [])
                        if preview:
                            st.write("Frame preview")
                            cols = st.columns(min(len(preview), 3))
                            for i, img in enumerate(preview):
                                if i < len(cols):
                                    cols[i].image(img, caption=f"Frame {i + 1}", use_container_width=True)
                    else:
                        st.warning("Video could not be saved (imageio/ffmpeg may be missing).")
                except Exception as e:
                    st.error(str(e))
        except Exception as e:
            st.error(str(e))
    else:
        st.info("Select or enter a scene path to load.")


def render_training():
    st.header("Training")
    scenes_dir = ROOT / "scenes"
    scene_files = list(scenes_dir.glob("*.xml")) + list(scenes_dir.glob("*.mjcf")) if scenes_dir.exists() else []
    scene_path = st.selectbox("Scene", [""] + [str(p) for p in scene_files], key="train_scene")
    scene_path_override = st.text_input("Or scene path", "", key="train_scene_override")
    path = (scene_path_override or scene_path) if (scene_path_override or scene_path) else None

    task_source = st.radio("Task source", ["Saved task", "Built-in task"], key="task_src")
    tasks_dir = ROOT / "tasks"
    task_files = list(tasks_dir.glob("*.json")) if tasks_dir.exists() else []
    task_path = None
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
        builtin = st.selectbox("Built-in task", ["PICK_EGG", "PICK_BOX", "HOLD"], key="builtin_task")
        if path:
            if builtin == "PICK_EGG":
                task = TaskBuilder.pick_egg_task(path)
            elif builtin == "PICK_BOX":
                task = TaskBuilder.pick_box_task(path)
            else:
                task = TaskBuilder.hold_task(path, hold_steps=100)

    method = st.selectbox("Training method", ["random_search", "evaluate_only"], key="train_method")
    n_iterations = st.number_input("Iterations (random_search)", 10, 500, 100, key="n_iter")
    n_episodes_per_eval = st.number_input("Episodes per evaluation", 1, 20, 5, key="n_ep_eval")
    n_episodes_final = st.number_input("Episodes for final eval", 1, 50, 10, key="n_ep_final")

    if st.button("Start Training") and path and task:
        try:
            sim = Simulator()
            sim.load_scene(path=path)
            config = {
                "n_iterations": n_iterations,
                "n_episodes_per_eval": n_episodes_per_eval,
                "n_params": 10,
            }
            progress = st.progress(0.0)
            status = st.empty()
            if method == "random_search":
                status.info("Running random search...")
                result = train_loop(sim, task, method="random_search", config=config)
                progress.progress(1.0)
                status.success("Training complete.")
            else:
                status.info("Evaluating policy...")
                result = train_loop(sim, task, method="evaluate_only", config={"n_episodes": n_episodes_final})
                progress.progress(1.0)
                status.success("Evaluation complete.")

            st.subheader("Results")
            if result.get("history"):
                st.line_chart({"reward": result["history"]})
            if result.get("best_reward") is not None:
                st.metric("Best reward", f"{result['best_reward']:.4f}")
            if result.get("evaluation"):
                ev = result["evaluation"]
                st.metric("Success rate", f"{ev.get('success_rate', 0):.2%}")
                st.metric("Avg reward", f"{ev.get('avg_reward', 0):.4f}")
                st.metric("Avg steps", f"{ev.get('avg_steps', 0):.1f}")
            if result.get("log_path"):
                st.caption(f"Log: {result['log_path']}")

            best_weights = result.get("best_weights")
            if best_weights is not None and method == "random_search":
                st.session_state["training_best_weights"] = best_weights
                st.session_state["training_result"] = result
                st.session_state["training_task"] = task
                st.session_state["training_path"] = path
            else:
                st.session_state["training_best_weights"] = None
                st.session_state["training_result"] = result
                st.session_state["training_task"] = task
                st.session_state["training_path"] = path
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
    elif not path:
        st.info("Select a scene and task to train.")


if __name__ == "__main__":
    main()
