import json
import queue
import csv
import asyncio
import math
import os
import shutil
import subprocess
import sys
import time
import traceback
from pathlib import Path

import carb
import carb.input
import carb.settings
import omni.appwindow
import omni.ext
import omni.kit.app
import omni.timeline
import omni.ui as ui
import omni.usd
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
from isaacsim.storage.native import get_assets_root_path
from . import runtime_physics as _dynamic_control
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, UsdSkel

from .ollama_agent import DEFAULT_HOST as AGENT_DEFAULT_HOST
from .ollama_agent import DEFAULT_MODEL as AGENT_DEFAULT_MODEL
from .ollama_agent import OllamaAgent
from .web_control import ExternalControlServer
from .object_editor import ObjectEditor
from .scene_preferences import read_startup_scene, write_startup_scene
from .room_scene import is_ai_graduate_room, room_focus_prim, building_camera_eye, room_library, validate_room, include_csv_object, build_environment, saved_rooms, save_snapshot, snapshot_config, restore_snapshot, LABELS as ROOM_LABELS
from .modbus_temperature import ModbusTemperatureReader, RiseWindow
from .robot_messages import RobotMessages
from .ollama_narrator import OllamaNarrator
from .navigation_modes import NavigationModes
from .navigation_observation import collision_rectangles
from .room_spawn import find_room_spawn, camera_local_height, ROBOT_BASE_HEIGHT
from .school_branding import create_dongshin_branding
from .room_scene import wall_logo_asset


LAB_ROOT = "/World/InteractivePhysicsLab"
OBJECT_ROOT = f"{LAB_ROOT}/Objects"
WORKER_PATH = f"{OBJECT_ROOT}/worker_01"
PUSHER_PATH = f"{LAB_ROOT}/Pusher"
GROUND_PATH = f"{LAB_ROOT}/Ground"
# Keep the PhysicsScene outside LAB_ROOT. Lab scene rebuilds delete LAB_ROOT;
# deleting a PhysicsScene cached by SimulationContext leaves an invalid schema.
PHYSICS_PATH = "/World/PhysicsScene"
ROBOT_PATH = f"{LAB_ROOT}/MobileRobot"
TARGET_PATH = f"{OBJECT_ROOT}/box_02"
TEACHING_CAMERA_PATH = f"{LAB_ROOT}/TeachingCamera"
GIST_SIGN_PATH = f"{LAB_ROOT}/Branding/GISTWallSign"


class InteractivePhysicsLabExtension:
    """Standalone PhysX AGV camera-view education lab v3.5.1 for Isaac Sim 4.5.

    Select a lab object in the viewport or Stage window, then use the buttons or
    keyboard shortcuts to change its physics properties.
    """

    MASS_LEVELS = (1.0, 5.0, 20.0, 50.0, 200.0)
    COLOR_LEVELS = (
        ("Blue", Gf.Vec3f(0.10, 0.32, 0.85)),
        ("Red", Gf.Vec3f(0.85, 0.15, 0.15)),
        ("Green", Gf.Vec3f(0.12, 0.65, 0.25)),
        ("Yellow", Gf.Vec3f(0.95, 0.78, 0.08)),
        ("Orange", Gf.Vec3f(1.00, 0.42, 0.05)),
    )
    MODEL_LEVELS = (
        ("Box", "box"),
        ("Worker", "worker"),
        ("Warehouse Robot", "robot"),
        ("Forklift", "forklift"),
    )
    SCALE_FACTORS = (0.8, 1.25)
    ROBOT_LINEAR_SPEED = 1.5
    ROBOT_ANGULAR_SPEED_DEG = 75.0
    AUTO_LINEAR_SPEED = 1.80
    AUTO_APPROACH_GAIN = 2.0
    AUTO_PUSH_SPEED = 1.35
    AUTO_TURN_SPEED_DEG = 65.0
    AGV_SIZE_SCALE = 0.70
    APPROACH_CLEARANCE = 1.204
    APPROACH_CONTACT_GUARD = 0.794
    BACK_OFF_DISTANCE = 1.40
    FINAL_BACK_OFF_DISTANCE = 1.20
    PUSH_CONTACT_MOVE_EPS = 0.015
    PUSH_CONTACT_SPEED_EPS = 0.06
    PUSH_PULSE_SEC = 1.05
    PUSH_MAX_DURATION_SEC = 3.0
    PUSH_MAX_TRAVEL = 1.50
    PUSH_MIN_BOX_MOVE = 0.010
    INTER_PUSH_WAIT_SEC = 2.00
    NAVIGATION_TIMEOUT_SEC = 45.0
    BACK_OFF_TIMEOUT_SEC = 6.0
    BOX_SAFETY_MAX_X = 9.50
    SETTLE_LINEAR_LIMIT = 0.08
    SETTLE_ANGULAR_LIMIT = 4.0
    SETTLE_REQUIRED_SEC = 0.75
    SETTLE_TIMEOUT_SEC = 5.0
    MISSION_TOTAL_TIMEOUT_SEC = 180.0
    ALARM_COLOR = Gf.Vec3f(1.0, 0.08, 0.03)

    # Lab 3: conversational control (local LLM via Ollama). Movement tools
    # deliberately reuse AUTO_LINEAR_SPEED/AUTO_TURN_SPEED_DEG (already tuned
    # for the Ridgeback's mass/damping by the Lab 2 mission) instead of
    # inventing new untested speeds. Only the two genuinely new concerns for
    # free-form agent moves get their own constants: how close counts as
    # "arrived", and a safety timeout no other control path needs.
    AGENT_MOVE_TOLERANCE = 0.10
    AGENT_ACTION_TIMEOUT_SEC = 20.0
    WORKER_AUTO_SPEED = 0.65
    # Keep the Worker in the documented safety aisle so the person's collider
    # cannot deadlock the temperature mission's post-push back-off path.
    WORKER_START_X = 1.20
    WORKER_START_Y = 4.00
    WORKER_MIN_Y = 3.20
    WORKER_MAX_Y = 4.80
    PERSON_SLOW_DISTANCE = 3.00
    PERSON_STOP_DISTANCE = 1.70
    PERSON_EMERGENCY_DISTANCE = 1.40
    PERSON_PATH_HALF_WIDTH = 1.10
    PERSON_AVOID_TURN_SPEED_DEG = 68.0
    PERSON_AVOIDANCE_HOLD_SEC = 0.65
    PERSON_MISSION_BLOCK_TIMEOUT_SEC = 4.0
    PERSON_EMERGENCY_ESCAPE_SPEED = 0.24
    PERSON_EMERGENCY_TURN_SPEED_DEG = 28.0
    OBSTACLE_STALL_SEC = 1.5
    OBSTACLE_PROGRESS_EPS = 0.06
    OBSTACLE_BACKOFF_DISTANCE = 0.80
    OBSTACLE_BYPASS_LATERAL = 1.25
    OBSTACLE_MAX_RETRIES = 3

    # Opaque ABGR colors used by omni.ui. Buttons that operate on the same
    # subsystem deliberately share one color so the control panel can be read
    # by function at a glance.
    BUTTON_STYLES = {
        "appearance": {"background_color": 0xFF75552C, "color": 0xFFFFFFFF},
        "agentic": {"background_color": 0xFF743E78, "color": 0xFFFFFFFF},
        "scene": {"background_color": 0xFF9A642E, "color": 0xFFFFFFFF},
        "worker": {"background_color": 0xFF39763E, "color": 0xFFFFFFFF},
        "face": {"background_color": 0xFF326EB2, "color": 0xFFFFFFFF},
        "temperature": {"background_color": 0xFF4146B5, "color": 0xFFFFFFFF},
        "robot": {"background_color": 0xFF7B7160, "color": 0xFFFFFFFF},
        "physics": {"background_color": 0xFF5B5B5B, "color": 0xFFFFFFFF},
    }

    def on_startup(self, ext_id):
        self._ext_id = ext_id
        self._robot_messages = RobotMessages()
        self._navigation = NavigationModes()
        self._navigation_geometry = []
        self._navigation_geometry_time = 0.0
        self._narrator = OllamaNarrator(AGENT_DEFAULT_HOST, AGENT_DEFAULT_MODEL)
        self._narrator_status = "Ollama 설명 준비 중"
        self._narrator_submitted_id = None
        self._narrated_motion = None
        self._startup_view_task = None
        self._startup_view_cancelled = False
        self._humidity_trigger_description = ""
        self._window = None
        self._mission_window = None
        self._guide_window = None
        self._status_label = None
        self._last_status_text = "Extension starting"
        self._keyboard = None
        self._keyboard_sub_id = None
        self._rebuild_task = None
        self._next_experience_after_rebuild = False
        self._experience_task = None
        self._experience_progress = ""
        self._pressed_keys = set()
        self._external_drive_key = None
        self._external_drive_started_at = 0.0
        self._external_drive_lease_until = 0.0
        self._robot_yaw_deg = 0.0
        self._pusher_has_launched = False
        self._worker_auto_active = False
        self._worker_direction = 1.0
        self._worker_visual_yaw_deg = 180.0
        self._person_safety_mode = "CLEAR"
        self._person_avoidance_until = 0.0
        self._mission_person_blocked_elapsed = 0.0
        self._face_file_picker = None
        self._face_path_model = None
        self._selected_face_path = ""
        self._face3d_task = None
        self._face3d_progress = ""
        self._face_restore_task = None
        self._busy_status_model = None
        self._sensor_unarmed_warning_latched = False
        self._temperature = 25.0
        self._temperature_threshold = 40.0
        self._sensor_mode = "Keyboard Simulation"
        self._hardware_temperature = None
        self._hardware_humidity = None
        self._hardware_humidity_delta = None
        self._hardware_sample_timestamp = None
        self._hardware_sample_interval = None
        self._hardware_sample_count = 0
        self._humidity = 50.0
        self._ui_humidity_delta = None
        self._ui_humidity_rise = RiseWindow(max_gap=None)
        self._hardware_sensor_error = "waiting for sensor" if os.environ.get("ISAAC_TEMP_PORT", "").strip() else ""
        self._hardware_rise = RiseWindow()
        self._hardware_reader = ModbusTemperatureReader()
        self._mission_state = "IDLE"
        self._mission_active = False
        self._mission_latched = False
        self._lab2_prepared = False
        self._push_count = 0
        self._box_initial_xy = None
        self._box_current_xy = None
        self._box_original_color = None
        self._approach_xy = None
        self._state_elapsed = 0.0
        self._mission_total_elapsed = 0.0
        self._settle_elapsed = 0.0
        self._push_start_xy = None
        self._push_box_start_xy = None
        self._push_contact_detected = False
        self._push_contact_elapsed = 0.0
        self._backoff_start_xy = None
        self._mission_labels = {}
        self._mission_models = {}
        self._temperature_control_model = None
        self._humidity_readout_label = None
        self._temperature_refresh_task = None
        self._mission_debug_elapsed = 0.0
        self._mission_person_blocked_elapsed = 0.0
        self._agv_install_task = None
        self._robot_motion_prim_path = None
        self._dynamic_control = _dynamic_control.acquire_dynamic_control_interface()
        self._dynamic_robot_handle = _dynamic_control.INVALID_HANDLE
        self._dynamic_control_error_signature = None
        self._last_drive_backend = "WAITING"
        self._last_requested_linear_speed = 0.0
        self._last_requested_angular_speed_deg = 0.0
        self._mobile_base_drive_paths = None
        self._mobile_base_drive_error_signature = None
        self._gripper_open = True
        self._last_arm_pose = "unknown"
        self._robot_camera_path = None
        self._main_camera_path = None
        self._robot_viewport_window = None
        self._camera_stream_task = None
        self._camera_frame_path = None
        self._popup_id = 0
        self._popup_phase = ""
        self._popup_title = ""
        self._popup_message = ""
        self._experience_active = False
        self._experience_name = ""
        self._experience_completion_pending = False
        self._avoidance_state = "IDLE"
        self._avoidance_retry_count = 0
        self._avoidance_target = None
        self._avoidance_last_xy = None
        self._avoidance_last_progress_at = 0.0
        self._avoidance_phase_start_xy = None
        self._avoidance_waypoint = None
        self._avoidance_failure = ""
        self._view_nav_window = None
        self._previous_view_camera_path = None
        self._teaching_camera_target = None
        self._external_control_server = None
        self._object_editor = ObjectEditor(self)
        self._scene_dirty = False
        self._startup_scene_file = None
        self._room_kind = "warehouse"
        self._room_branding = "gist"
        self._room_source = ""
        self._saved_rooms = saved_rooms(self)
        self._room_library = room_library(self)
        self._object_library = room_library(self, "object")
        self._update_sub = None
        self._control_loop_task = None
        self._external_drive_task = None
        self._temperature_mission_task = None
        self._input = carb.input.acquire_input_interface()
        self._timeline = omni.timeline.get_timeline_interface()
        self._background_setting_previous = {}
        self._background_simulation_enabled = self._configure_background_simulation()

        # Lab 3: conversational control state. _agent_action_active mirrors
        # _mission_active as a mutual-exclusion flag so a drive_robot/turn_robot
        # tool call and manual arrow keys (or the temperature mission) never
        # command velocity in the same frame.
        self._agent = OllamaAgent(host=AGENT_DEFAULT_HOST, model=AGENT_DEFAULT_MODEL)
        self._agent_busy = False
        self._agent_action_active = False
        self._agent_turn_task = None
        self._agent_field = None
        self._agent_input_model = None
        self._agent_input_active = False
        self._agent_reply_label = None
        self._agent_reply_model = None
        self._agent_motion_epoch = 0
        self._is_shutting_down = False

        # Browser-only control mode: do not create native omni.ui control or
        # mission-status windows inside the Isaac Sim application frame.
        self._window = None
        self._mission_window = None
        self._subscribe_keyboard()
        # Use the same per-frame async mechanism as the proven Ollama motion
        # tools.  The former update-event subscription stopped driving manual
        # and temperature-mission motion when another OS window owned focus,
        # even though next_update_async-based Agentic motion kept working.
        self._control_loop_task = asyncio.ensure_future(self._run_control_loop())
        self._open_external_control_panel()
        self._learned_service_manager = None
        self._start_learned_service_manager()
        self._nav2_service_manager = None
        self._start_nav2_service_manager()
        self._hardware_reader.start()
        self._startup_view_task = asyncio.ensure_future(self._startup_scene_only())
        # Run before window.file's order=0 hook, so its save prompt is visible.
        self._shutdown_ui_sub = omni.kit.app.get_app().get_shutdown_event_stream().create_subscription_to_pop(
            self._on_exit_requested, name="Interactive Physics Lab exit UI", order=-100
        )
        carb.log_info("[Interactive Physics Lab] Extension started")
        # Isaac Sim 4.5 can stall when PhysX prims are authored while the full
        # application and renderer are still starting.  Scene creation is
        # therefore deliberately user-triggered after the UI is ready.
        self._set_status(
            "Extension 준비 완료.\n"
            "Lab 1-A에서 물리를 확인하고 Lab 1-B에서 창고를 만든 뒤\n"
            "Lab 1-C로 PhysX AGV를 가상공간에 투입하세요."
        )

    def on_shutdown(self):
        if getattr(self, "_dynamic_control", None):
            self._dynamic_control.close()
        self._shutdown_ui_sub = None
        self._is_shutting_down = True
        self._navigation.cancel()
        self._command_robot_velocity(0.0, 0.0)
        manager = getattr(self, "_learned_service_manager", None)
        self._learned_service_manager = None
        if manager is not None and manager.poll() is None:
            manager.terminate()
            try:manager.wait(timeout=5)
            except subprocess.TimeoutExpired:
                carb.log_warn("학습 모델 서비스 종료가 지연되고 있습니다.")
        manager = getattr(self, "_nav2_service_manager", None)
        self._nav2_service_manager = None
        if manager is not None and manager.poll() is None:
            manager.terminate()
        web_app = getattr(self, "_remote_web_app", None)
        if web_app is not None and web_app.poll() is None:
            web_app.terminate()
        self._remote_web_app = None
        homepage_agent = getattr(self, "_homepage_agent", None)
        if homepage_agent is not None and homepage_agent.poll() is None:
            homepage_agent.terminate()
            try:
                homepage_agent.wait(timeout=12)
            except subprocess.TimeoutExpired:
                homepage_agent.kill()
                homepage_agent.wait(timeout=2)
        self._homepage_agent = None
        self._narrator.stop()
        self._navigation.close()
        if self._startup_view_task is not None:
            self._startup_view_task.cancel()
        self._hardware_reader.stop()
        self._worker_auto_active = False
        self._person_safety_mode = "CLEAR"
        self._person_avoidance_until = 0.0
        self._close_face_file_picker()
        self._agent_motion_epoch += 1
        self._restore_previous_view(silent=True)
        if self._view_nav_window is not None:
            self._view_nav_window.visible = False
        if self._external_control_server is not None:
            self._external_control_server.stop()
            self._external_control_server = None
        self._show_main_view(silent=True)
        self._command_robot_velocity(0.0, 0.0)
        self._restore_background_simulation_settings()
        if self._rebuild_task is not None and not self._rebuild_task.done():
            self._rebuild_task.cancel()
        if self._agv_install_task is not None and not self._agv_install_task.done():
            self._agv_install_task.cancel()
        if self._agent_turn_task is not None and not self._agent_turn_task.done():
            self._agent_turn_task.cancel()
        if self._temperature_refresh_task is not None and not self._temperature_refresh_task.done():
            self._temperature_refresh_task.cancel()
        if self._face3d_task is not None and not self._face3d_task.done():
            self._face3d_task.cancel()
        if self._face_restore_task is not None and not self._face_restore_task.done():
            self._face_restore_task.cancel()
        if self._control_loop_task is not None and not self._control_loop_task.done():
            self._control_loop_task.cancel()
        if self._camera_stream_task is not None and not self._camera_stream_task.done():
            self._camera_stream_task.cancel()
        if self._external_drive_task is not None and not self._external_drive_task.done():
            self._external_drive_task.cancel()
        if self._temperature_mission_task is not None and not self._temperature_mission_task.done():
            self._temperature_mission_task.cancel()
        if self._experience_task is not None and not self._experience_task.done():
            self._experience_task.cancel()
        if self._keyboard_sub_id is not None and self._keyboard is not None:
            self._input.unsubscribe_to_keyboard_events(
                self._keyboard, self._keyboard_sub_id
            )
        self._keyboard_sub_id = None
        self._keyboard = None
        self._update_sub = None
        self._control_loop_task = None
        self._external_drive_task = None
        self._temperature_mission_task = None
        self._experience_task = None
        self._experience_progress = ""
        self._camera_stream_task = None
        self._pressed_keys.clear()
        self._external_drive_key = None
        self._agent_action_active = False
        self._agent_busy = False
        self._agent = None
        self._agent_field = None
        self._agent_input_model = None
        self._agent_input_active = False
        self._agent_reply_label = None
        self._agent_reply_model = None
        self._temperature_control_model = None
        self._humidity_readout_label = None
        self._temperature_refresh_task = None
        self._face_path_model = None
        self._selected_face_path = ""
        self._face3d_task = None
        self._face3d_progress = ""
        self._face_restore_task = None
        self._busy_status_model = None
        self._guide_window = None
        self._mission_window = None
        self._view_nav_window = None
        self._previous_view_camera_path = None
        self._teaching_camera_target = None
        self._window = None

    def _configure_background_simulation(self):
        """Keep Kit and PhysX ticking while the browser owns OS focus."""
        settings = carb.settings.get_settings()
        updates = (
            ("/app/window/sleepMsOnFocusLoss", 0),
            ("/app/runLoopsGlobal/syncToPresent", False),
            ("/app/runLoops/main/rateLimitEnabled", True),
            ("/app/runLoops/main/rateLimitFrequency", 60),
            ("/persistent/simulation/minFrameRate", 60),
        )
        try:
            for path, value in updates:
                self._background_setting_previous[path] = settings.get(path)
                settings.set(path, value)
            carb.log_info(
                "[External Control] background simulation enabled: "
                "focusLossSleep=0ms, mainLoop=60Hz, syncToPresent=false"
            )
            return True
        except Exception as error:
            carb.log_warn(f"[External Control] background simulation setting failed: {error}")
            return False

    def _restore_background_simulation_settings(self):
        settings = carb.settings.get_settings()
        for path, value in self._background_setting_previous.items():
            if value is not None:
                try:
                    settings.set(path, value)
                except Exception:
                    pass
        self._background_setting_previous.clear()

    def _button(self, text, group, **kwargs):
        """Create a control button with the color assigned to its function."""
        base = dict(self.BUTTON_STYLES[group])
        style = {
            "Button": base,
            "Button:hovered": {
                **base,
                "border_color": 0xFFFFFFFF,
                "border_width": 2.0,
            },
            "Button:pressed": {
                **base,
                "border_color": 0xFF66CCFF,
                "border_width": 2.0,
            },
            "Button:disabled": {
                "background_color": 0xFF3D3D3D,
                "color": 0xFF888888,
            },
        }
        return ui.Button(text, style=style, **kwargs)

    @staticmethod
    def _face_trace(code, message):
        """Emit numbered Face3D diagnostics directly to the launch terminal."""
        line = f"[FACE-{code}] {message}"
        print(line, flush=True)
        carb.log_info(line)

    def _build_window(self):
        self._window = ui.Window(
            "Interactive Physics Lab",
            width=390,
            height=1025,
        )
        with self._window.frame:
            # Isaac Sim 4.5 can leave stale glyphs while ScrollingFrame repaints.
            # The compact control panel therefore has no scroll container;
            # changing mission values live in a separate opaque status window.
            with ui.VStack(height=0):
                with ui.VStack(spacing=2, height=0):
                    with ui.HStack(height=26):
                        ui.Label("Interactive Physics Lab", height=26)
                        ui.Label(
                            "Version 3.5.1",
                            height=26,
                        )
                    ui.Label(
                        "Select an object in the Stage, then use buttons or keys.",
                        word_wrap=True,
                        height=26,
                    )
                    self._busy_status_model = ui.SimpleStringModel("● 준비 완료")
                    ui.StringField(
                        model=self._busy_status_model,
                        read_only=True,
                        height=28,
                    )
                    ui.Label("Selected Object Appearance", height=22)
                    with ui.HStack(height=40, spacing=5):
                        self._button(
                            "C  Change Color",
                            "appearance",
                            clicked_fn=self._cycle_color,
                        )
                        self._button(
                            "H  Change Model",
                            "appearance",
                            clicked_fn=self._cycle_model,
                        )
                    ui.Separator(height=6)

                    # Keep conversational control in the immediately visible
                    # part of the panel.  It was previously below the long
                    # mission status list and appeared to be missing.
                    ui.Label("Lab 3: Ollama Agentic Control", height=22)
                    ui.Label(
                        f"Ollama: {self._agent.host} · model: {self._agent.model}",
                        height=18,
                    )
                    ui.Label("Route: OLLAMA LLM ONLY (no local command bypass)", height=18)
                    self._agent_input_model = ui.SimpleStringModel("")
                    self._agent_field = ui.StringField(
                        model=self._agent_input_model,
                        read_only=False,
                        password_mode=False,
                        height=32,
                        activated_fn=lambda *args: self._on_agent_send(),
                        mouse_pressed_fn=lambda *args: self._focus_agent_input(),
                    )
                    with ui.HStack(height=34, spacing=5):
                        self._button(
                            "Focus",
                            "agentic",
                            height=32,
                            clicked_fn=self._focus_agent_input,
                        )
                        self._button(
                            "Run Prompt",
                            "agentic",
                            height=32,
                            mouse_pressed_fn=lambda *args: self._on_agent_send(),
                        )
                        self._button(
                            "Manual Keys",
                            "agentic",
                            height=32,
                            clicked_fn=self._leave_agent_input,
                        )
                    self._agent_reply_model = ui.SimpleStringModel(
                        "Example: turn left 45 degrees / move forward 1 meter"
                    )
                    self._agent_reply_label = ui.StringField(
                        model=self._agent_reply_model,
                        read_only=True,
                        height=30,
                    )
                    ui.Separator(height=6)

                    ui.Label("Lab selection", height=20)
                    with ui.HStack(height=34, spacing=5):
                        self._button("Lab 1-A: Minimal Scene", "scene", clicked_fn=self._create_minimal_scene)
                        self._button("Lab 1-B: Warehouse Scene", "scene", clicked_fn=self._create_demo_scene)
                    self._button("Lab 1-C: Add PhysX AGV", "scene", height=32, clicked_fn=self._add_agv_to_twin)
                    with ui.HStack(height=34, spacing=5):
                        self._button("Start Worker", "worker", clicked_fn=self._start_worker_motion)
                        self._button("Stop Worker", "worker", clicked_fn=self._stop_worker_motion)
                        self._button("Reset Worker", "worker", clicked_fn=self._reset_worker_motion)
                    with ui.HStack(height=34, spacing=5):
                        self._button("Rotate Worker +45°", "worker", clicked_fn=self._rotate_worker_45)
                        self._button("Worker Smaller", "worker", clicked_fn=lambda: self._scale_worker(0.8))
                        self._button("Worker Larger", "worker", clicked_fn=lambda: self._scale_worker(1.25))
                    ui.Label("Worker Face Mapping", height=22)
                    selected_face_name = (
                        Path(self._selected_face_path).name
                        if self._selected_face_path
                        and Path(self._selected_face_path).is_file()
                        else "No face image selected"
                    )
                    self._face_path_model = ui.SimpleStringModel(selected_face_name)
                    ui.StringField(
                        model=self._face_path_model,
                        read_only=True,
                        height=28,
                    )
                    with ui.HStack(height=34, spacing=5):
                        self._button("Browse Face Photo", "face", clicked_fn=self._open_face_file_picker)
                        self._button("Map 2D Face", "face", clicked_fn=self._start_2d_face_mapping)
                    self._button("Lab 2: Temperature Mission", "temperature", height=32, clicked_fn=self._prepare_temperature_mission)
                    ui.Separator(height=6)
    
                    ui.Label("Temperature Mission", height=22)
                    with ui.HStack(height=38, spacing=5):
                        self._button(
                            "Temperature +5 (T)",
                            "temperature",
                            mouse_pressed_fn=self._on_temperature_increase,
                        )
                        self._button(
                            "Temperature -5",
                            "temperature",
                            mouse_pressed_fn=self._on_temperature_decrease,
                        )
                    self._temperature_control_model = ui.SimpleStringModel(
                        "Temperature: 25.0 °C | Threshold: 40.0 °C"
                    )
                    ui.StringField(
                        model=self._temperature_control_model,
                        read_only=True,
                        height=30,
                    )
                    ui.Label(
                        "Live values: Temperature Mission Status window",
                        height=22,
                    )
                    self._humidity_readout_label = ui.Label("Humidity: waiting for data", height=58, word_wrap=True)
                    self._button("Reset Temperature Mission", "temperature", height=32, clicked_fn=self._reset_temperature_mission)
                    ui.Separator(height=6)
    
                    ui.Label("Robot: ↑/↓ drive, ←/→ turn, Space stop", height=24)
                    with ui.HStack(height=34, spacing=5):
                        self._button("Stop Robot", "robot", clicked_fn=self._stop_robot)
                        self._button("Reset Robot", "robot", clicked_fn=self._reset_robot)
                    with ui.HStack(height=34, spacing=5):
                        self._button("AGV Cam", "robot", clicked_fn=self._show_robot_camera_view)
                        self._button("Close Cam", "robot", clicked_fn=self._show_main_view)
                        self._button("View Nav", "robot", clicked_fn=self._open_view_navigation)
                        self._button("External Browser", "robot", clicked_fn=self._open_external_control_panel)
                    ui.Label("AGV Safety: person avoidance active in ALL modes", height=24)
                    ui.Separator(height=6)
    
                    ui.Label("Selected Object Physics", height=22)
                    with ui.HStack(height=34, spacing=5):
                        self._button("1  Fixed", "physics", clicked_fn=lambda: self._set_mobility("fixed"))
                        self._button("2  Movable", "physics", clicked_fn=lambda: self._set_mobility("movable"))
                        self._button("3  Kinematic", "physics", clicked_fn=lambda: self._set_mobility("kinematic"))

                    with ui.HStack(height=34, spacing=5):
                        self._button("M  Mass", "physics", clicked_fn=self._cycle_mass)
                        self._button("[  Smaller", "physics", clicked_fn=lambda: self._scale_selected(0.8))
                        self._button("]  Larger", "physics", clicked_fn=lambda: self._scale_selected(1.25))
    
                    ui.Separator(height=6)
    
                    ui.Label("Drive ↑/←/↓/→ | C Color | H Model | M Mass | T +/-", height=24)
                    self._status_label = ui.Label(
                        "Run Step 1: Minimal Test first.",
                        word_wrap=True,
                        height=44,
                    )
        self._build_mission_status_window()
        self._refresh_mission_ui()

    def _build_mission_status_window(self):
        """Build a non-scrolling live status window with opaque value fields."""
        self._mission_window = ui.Window(
            "Temperature Mission Status",
            width=390,
            height=350,
        )
        self._mission_models.clear()
        self._mission_labels.clear()
        with self._mission_window.frame:
            with ui.VStack(spacing=3, height=0):
                for key in (
                    "temperature", "threshold", "sensor", "target", "state",
                    "push", "initial", "current", "moved", "mission",
                ):
                    model = ui.SimpleStringModel("")
                    self._mission_models[key] = model
                    self._mission_labels[key] = ui.StringField(
                        model=model,
                        read_only=True,
                        height=28,
                    )

    def _open_control_guide_txt(self):
        """Open the UTF-8 Korean guide in Ubuntu's default text viewer.

        Isaac Sim 4.5's bundled UI font can display Korean as question marks.
        Opening the file externally avoids that font limitation.  Isaac's
        Python and shared-library environment is removed for the child app so
        Ubuntu GUI applications do not load Kit's bundled libraries.
        """
        guide_path = self._extension_root() / "docs" / "controls_guide.txt"
        if not guide_path.exists():
            carb.log_error(f"[Interactive Physics Lab] Guide not found: {guide_path}")
            self._set_status(f"Guide file not found:\n{guide_path}")
            return

        try:
            if sys.platform.startswith("win"):
                os.startfile(str(guide_path))
            else:
                opener_name = "open" if sys.platform == "darwin" else "xdg-open"
                opener = shutil.which(opener_name)
                if opener is None:
                    raise FileNotFoundError(
                        f"No document opener found in PATH: {opener_name}"
                    )
                child_env = os.environ.copy()
                for name in ("PYTHONHOME", "PYTHONPATH", "LD_PRELOAD", "LD_LIBRARY_PATH"):
                    child_env.pop(name, None)
                subprocess.Popen(
                    [opener, str(guide_path)],
                    env=child_env,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    start_new_session=True,
                )
            carb.log_info(f"[Interactive Physics Lab] Opened external guide: {guide_path}")
        except Exception as error:
            carb.log_error(f"[Interactive Physics Lab] Cannot open guide: {error}")
            self._set_status(f"Open this file manually:\n{guide_path}")

    def _subscribe_keyboard(self):
        app_window = omni.appwindow.get_default_app_window()
        if app_window is None:
            self._set_status("키보드 입력 창을 찾지 못했습니다. UI 버튼을 사용하세요.")
            return
        self._keyboard = app_window.get_keyboard()
        self._keyboard_sub_id = self._input.subscribe_to_keyboard_events(
            self._keyboard, self._on_keyboard_event
        )

    def _on_keyboard_event(self, event):
        key_name = getattr(event.input, "name", str(event.input)).upper()
        key_name = key_name.split(".")[-1]
        if key_name.startswith("KEY_"):
            key_name = key_name[4:]
        # Carb/Kit releases use both short and ARROW_* style names.
        key_name = {
            "ARROW_UP": "UP",
            "UP_ARROW": "UP",
            "ARROW_DOWN": "DOWN",
            "DOWN_ARROW": "DOWN",
            "ARROW_LEFT": "LEFT",
            "LEFT_ARROW": "LEFT",
            "ARROW_RIGHT": "RIGHT",
            "RIGHT_ARROW": "RIGHT",
        }.get(key_name, key_name)

        # Use the widget's real focus state, not a persistent software latch.
        # Clicking elsewhere immediately restores arrow-key manual control.
        prompt_focused = (
            self._agent_field is not None
            and bool(getattr(self._agent_field, "focused", False))
        )
        if prompt_focused and key_name in {"ENTER", "NUMPAD_ENTER", "RETURN"}:
            if event.type == carb.input.KeyboardEventType.KEY_PRESS:
                self._on_agent_send()
            return False
        if prompt_focused:
            return True

        # F8 is an Isaac/Kit-version-independent fallback for Run Prompt.
        if key_name == "F8" and event.type == carb.input.KeyboardEventType.KEY_PRESS:
            self._on_agent_send()
            return False

        if key_name in {"LEFT_SHIFT", "RIGHT_SHIFT", "SHIFT"}:
            if event.type == carb.input.KeyboardEventType.KEY_PRESS:
                self._pressed_keys.add(key_name)
            elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
                self._pressed_keys.discard(key_name)
            return True

        movement_keys = {"UP", "DOWN", "LEFT", "RIGHT"}
        if key_name in movement_keys:
            if self._mission_active:
                if event.type == carb.input.KeyboardEventType.KEY_PRESS:
                    self._set_status("Automatic mission active: manual arrow-key input is locked.")
                return False
            if self._agent_action_active:
                if event.type == carb.input.KeyboardEventType.KEY_PRESS:
                    self._set_status("Conversational control action in progress: manual arrow-key input is locked.")
                return False
            if event.type == carb.input.KeyboardEventType.KEY_PRESS:
                self._pressed_keys.add(key_name)
                self._play_simulation()
                carb.log_info(f"[Interactive Physics Lab] Key pressed: {key_name}")
            elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
                self._pressed_keys.discard(key_name)
            return True

        if key_name == "SPACE" and event.type == carb.input.KeyboardEventType.KEY_PRESS:
            self._pressed_keys.clear()
            self._stop_robot()
            return False

        if event.type != carb.input.KeyboardEventType.KEY_PRESS:
            return True

        if key_name == "T":
            shift_pressed = bool(self._pressed_keys.intersection({"LEFT_SHIFT", "RIGHT_SHIFT", "SHIFT"}))
            delta = -5.0 if shift_pressed else 5.0
            carb.log_info(f"[Lab 2 Key] T received, delta={delta:+.1f}")
            self._adjust_temperature(delta)
            return False

        handlers = {
            "1": lambda: self._set_mobility("fixed"),
            "2": lambda: self._set_mobility("movable"),
            "3": lambda: self._set_mobility("kinematic"),
            "M": self._cycle_mass,
            "C": self._cycle_color,
            "H": self._cycle_model,
            "LEFT_BRACKET": lambda: self._scale_selected(0.8),
            "RIGHT_BRACKET": lambda: self._scale_selected(1.25),
            "P": self._launch_pusher,
            "R": self._reset_objects,
            "J": self._save_json,
        }
        handler = handlers.get(key_name)
        if handler is not None:
            handler()
            return False
        return True

    async def _run_control_loop(self):
        """Drive browser, manual and mission control on Kit's async frame loop."""
        app = omni.kit.app.get_app()
        previous = time.monotonic()
        try:
            while not self._is_shutting_down:
                await app.next_update_async()
                now = time.monotonic()
                dt = min(max(now - previous, 1.0 / 240.0), 0.1)
                previous = now
                self._control_tick(dt)
        except asyncio.CancelledError:
            return
        except Exception as error:
            carb.log_error(f"[Interactive Physics Lab] control loop failed: {error}")
            self._set_status(f"CONTROL LOOP ERROR: {type(error).__name__}: {error}")

    def _on_update(self, event):
        """Compatibility entry point for tests and older Kit integrations."""
        payload = getattr(event, "payload", None)
        dt = float(payload.get("dt", 1.0 / 60.0)) if payload else 1.0 / 60.0
        self._control_tick(min(max(dt, 1.0 / 240.0), 0.1))

    def _control_tick(self, dt):
        # Browser requests arrive on a localhost HTTP thread, but every USD and
        # PhysX operation is executed here on Kit's async main frame.
        self._drain_external_control_commands()
        self._drain_hardware_temperature()
        self._update_busy_indicator()
        self._navigation.poll()
        self._update_narrator()
        self._publish_external_control_status()
        if not self._timeline.is_playing():
            return
        self._update_worker_motion(dt)
        robot = self._stage.GetPrimAtPath(ROBOT_PATH) if self._stage else None
        if robot is None or not robot.IsValid():
            return
        if self._mission_active:
            if self._temperature_mission_task is None or self._temperature_mission_task.done():
                self._update_temperature_mission(dt)
            return
        if self._agent_action_active:
            # A drive_robot/turn_robot tool call owns the velocity commands
            # for its own duration (see _agent_drive_distance/_agent_turn_degrees);
            # skip the manual arrow-key branch below so the two never fight.
            return
        if self._external_drive_task is not None and not self._external_drive_task.done():
            return

        now = time.monotonic()
        if self._external_drive_key and now >= self._external_drive_lease_until:
            self._external_drive_key = None
        active_keys = set(self._pressed_keys)
        if self._external_drive_key:
            active_keys.add(self._external_drive_key)
        drive = float("UP" in active_keys) - float("DOWN" in active_keys)
        turn = float("LEFT" in active_keys) - float("RIGHT" in active_keys)
        if drive or turn:
            self._command_robot_velocity(
                drive * self.ROBOT_LINEAR_SPEED,
                turn * self.ROBOT_ANGULAR_SPEED_DEG,
            )
        else:
            self._command_robot_velocity(0.0, 0.0)

    def _show_experience_popup(self, phase, title, message):
        self._popup_id += 1
        self._popup_phase = str(phase)
        self._popup_title = str(title)
        self._popup_message = str(message)
        category = "error" if "실패" in phase else "safe" if "막힘" in phase else "done" if "완료" in phase else "plan" if any(w in phase for w in ("계산", "우회", "의미")) else "action"
        self._narrate(category, str(message), alert="operation" if category == "error" else None)

    def _begin_experience(self, name, meaning):
        self._robot_messages.clear()
        self._narrated_motion = None
        self._experience_active = True
        self._experience_name = str(name)
        self._experience_completion_pending = True
        self._show_experience_popup("체험 의미", name, meaning)

    def _complete_experience(self, message):
        name = self._experience_name or "Interactive Physics Lab"
        self._experience_active = False
        self._experience_completion_pending = False
        self._show_experience_popup("체험 완료", name, message)

    def _current_busy_operation(self):
        """Return the highest-priority long-running operation for both UIs."""
        if self._face3d_task is not None and not self._face3d_task.done():
            return self._face3d_progress or "2D 얼굴 매핑 준비 중"
        if self._rebuild_task is not None and not self._rebuild_task.done():
            return "창고 장면 생성 중"
        if self._agv_install_task is not None and not self._agv_install_task.done():
            return "PhysX AGV 설치 중"
        if self._experience_task is not None and not self._experience_task.done():
            return self._experience_progress or "체험 자동실행 중"
        if self._agent_busy:
            return "Ollama 명령 수행 중"
        if self._mission_active:
            return f"습도 미션 수행 중 · {self._mission_state}"
        if self._agent_action_active:
            return "AGV 목표 동작 수행 중"
        return ""

    def _update_busy_indicator(self):
        operation = self._current_busy_operation()
        model = self._busy_status_model
        if model is None:
            return
        if operation:
            frames = ("◐", "◓", "◑", "◒")
            frame = frames[int(time.monotonic() * 8.0) % len(frames)]
            model.set_value(f"{frame} {operation}")
        else:
            model.set_value("● 준비 완료")

    async def _run_external_drive(self):
        """Maintain browser-button/key velocity on an independent frame task."""
        app = omni.kit.app.get_app()
        command_by_key = {
            "UP": (self.ROBOT_LINEAR_SPEED, 0.0),
            "DOWN": (-self.ROBOT_LINEAR_SPEED, 0.0),
            "LEFT": (0.0, self.ROBOT_ANGULAR_SPEED_DEG),
            "RIGHT": (0.0, -self.ROBOT_ANGULAR_SPEED_DEG),
        }
        carb.log_info("[External Drive Task] started")
        try:
            while not self._is_shutting_down:
                key = self._external_drive_key
                if (
                    key not in command_by_key
                    or time.monotonic() >= self._external_drive_lease_until
                    or self._mission_active
                    or self._agent_action_active
                ):
                    break
                self._command_robot_velocity(*command_by_key[key])
                await app.next_update_async()
        except asyncio.CancelledError:
            pass
        except Exception as error:
            carb.log_error(f"[External Drive Task] failed: {error}")
            self._set_status(
                f"BROWSER DRIVE ERROR: {type(error).__name__}: {error}"
            )
        finally:
            # Do not let a cancelled manual task overwrite the first command
            # of a newly started automatic/Agentic motion task.
            if not self._mission_active and not self._agent_action_active:
                self._command_robot_velocity(0.0, 0.0)
            carb.log_info("[External Drive Task] stopped")

    async def _run_temperature_mission(self):
        """Advance Lab 2 with the same frame task used by Agentic motion."""
        app = omni.kit.app.get_app()
        previous = time.monotonic()
        carb.log_info("[Temperature Mission Task] started")
        try:
            while self._mission_active and not self._is_shutting_down:
                now = time.monotonic()
                dt = min(max(now - previous, 1.0 / 240.0), 0.1)
                previous = now
                self._update_temperature_mission(dt)
                if self._mission_active:
                    await app.next_update_async()
        except asyncio.CancelledError:
            pass
        except Exception as error:
            self._abort_mission(f"async mission loop error: {error}")
        finally:
            self._command_robot_velocity(0.0, 0.0)
            carb.log_info("[Temperature Mission Task] stopped")

    @staticmethod
    def _yaw_from_quaternion(quaternion):
        if quaternion is None:
            return 0.0
        w = float(quaternion.GetReal())
        imaginary = quaternion.GetImaginary()
        x, y, z = float(imaginary[0]), float(imaginary[1]), float(imaginary[2])
        return math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )

    @property
    def _stage(self):
        return omni.usd.get_context().get_stage()

    def _create_demo_scene(self):
        self._begin_experience("Lab 1-B · Warehouse", "CSV 설비와 Worker로 창고 디지털 트윈을 구성합니다.")
        self._request_scene_build(max_csv_objects=None)

    def _create_dongshin_scene(self):
        self._begin_experience("Lab 1-B · 동신여고", "공식 학교 로고를 벽 3면과 바닥에 적용한 창고를 구성합니다.")
        self._request_scene_build(max_csv_objects=None, room_config={
            "kind": "warehouse", "include_lab": True, "branding": "dongshin",
        })

    def _current_warehouse_config(self):
        if self._room_branding == "dongshin":
            return {"kind": "warehouse", "include_lab": True, "branding": "dongshin"}
        return None

    def _create_minimal_scene(self):
        self._begin_experience("Lab 1-A · Minimal", "중력·충돌·강체가 동작하는 최소 PhysX 장면을 구성합니다.")
        self._request_scene_build(max_csv_objects=3)

    def _room_ready(self):
        stage = self._stage
        if stage is None:
            return False
        lab = stage.GetPrimAtPath(LAB_ROOT)
        return bool(lab.IsValid() and lab.GetAttribute('lab:roomReady').Get())

    def _save_current_room(self, label):
        try:
            if self._current_busy_operation() or self._agent_action_active:
                raise ValueError('작업을 마치거나 STOP 후 저장하세요.')
            if self._external_drive_key or self._pressed_keys:
                raise ValueError('수동 주행을 멈춘 뒤 저장하세요.')
            if self._stage is None or not self._stage.GetPrimAtPath(LAB_ROOT).IsValid():
                raise ValueError('저장할 장면이 없습니다.')
            self._timeline.pause()
            self._worker_auto_active = False
            self._set_status('현재 공간 저장 중: 추가 객체와 배치를 저장하고 있습니다…')
            path = save_snapshot(self, label)
            write_startup_scene(self._extension_root().parents[1], path.name)
            self._startup_scene_file = path.name
            self._scene_dirty = False
            self._saved_rooms = saved_rooms(self)
            self._set_status(f'현재 장면 저장 완료: {path.name} · 다음 실행 때 이 장면을 복원합니다. 추가/삭제/배치가 함께 저장되었습니다.')
        except Exception as error:
            self._set_status(f'장면 저장 실패: {error}')

    def _mark_scene_changed(self):
        self._scene_dirty = True

    def _decline_scene_save(self):
        self._scene_dirty = False
        self._set_status('저장하지 않음: 현재 화면은 유지합니다. 다음 실행에는 이전 저장본이 있으면 그 장면을 사용합니다.')

    def _restore_startup_scene(self):
        try:
            filename = read_startup_scene(self._extension_root().parents[1])
            self._startup_scene_file = filename
            # A hot reload must never overwrite an already open lab.
            if not filename or self._stage is None or self._stage.GetPrimAtPath(LAB_ROOT).IsValid():
                return
            if self._current_busy_operation():return
            self._load_saved_room(filename)
        except Exception as error:
            self._set_status(f'시작 장면 복원 실패: {error} · 저장 장면 목록에서 다시 선택하세요.')

    def _load_saved_room(self, filename):
        try:
            if self._current_busy_operation() or self._agent_action_active:
                raise ValueError('작업을 마치거나 STOP 후 불러오세요.')
            config = snapshot_config(self, filename)
            self._begin_experience('저장 장면 복원', str(filename))
            self._request_scene_build(max_csv_objects=None, room_config=config)
        except Exception as error:
            self._set_status(f'장면 불러오기 실패: {error}')

    async def _restore_room_snapshot_async(self, config):
        app = omni.kit.app.get_app()
        self._worker_auto_active = False
        self._dynamic_robot_handle = _dynamic_control.INVALID_HANDLE
        self._robot_motion_prim_path = None
        self._mobile_base_drive_paths = None
        await app.next_update_async()
        lab = restore_snapshot(self._stage, config['snapshot_path'])
        await app.next_update_async()
        await app.next_update_async()
        self._room_kind = lab.GetAttribute('lab:roomKind').Get() or 'warehouse'
        self._room_branding = lab.GetAttribute('lab:branding').Get() or 'gist'
        self._room_source = self._stage.GetRootLayer().customLayerData.get('roomSource', '') or config['snapshot_path']
        self._ensure_physics_scene()
        # Re-arm only on explicit Lab 2 preparation; retain the saved object's mobility.
        target = self._stage.GetPrimAtPath(TARGET_PATH)
        mobility = target.GetAttribute('lab:mobility').Get() if target.IsValid() else None
        self._initialize_mission_from_stage(armed=False)
        if mobility:
            self._apply_mobility(target, mobility)
        for prim in self._stage.Traverse():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                UsdPhysics.RigidBodyAPI(prim).CreateVelocityAttr().Set(Gf.Vec3f(0))
                UsdPhysics.RigidBodyAPI(prim).CreateAngularVelocityAttr().Set(Gf.Vec3f(0))
        robot = self._stage.GetPrimAtPath(ROBOT_PATH)
        if robot.IsValid():
            self._apply_default_agv_size(robot)
            self._install_robot_camera(robot)
        saved_target = self._stage.GetRootLayer().customLayerData.get('teachingTarget')
        eye = self._teaching_camera_eye()
        if saved_target is not None and eye is not None:
            self._set_teaching_camera(eye, Gf.Vec3d(saved_target))
        else:
            self._focus_warehouse_view()
        self._saved_rooms = saved_rooms(self)
        self._set_status('저장 장면 복원 완료: 추가·삭제·모델·배치·AGV 복원. 습도 미션은 Lab 2로 재준비하세요.')

    def _change_room(self, data):
        try:
            if self._current_busy_operation() or self._agent_action_active:
                raise ValueError('현재 작업을 마치거나 STOP 후 방을 교체하세요.')
            config = validate_room(data)
            self._begin_experience('공간 교체', ROOM_LABELS[config['kind']])
            self._request_scene_build(max_csv_objects=None, room_config=config)
        except Exception as error:
            self._set_status(f'공간 교체 실패: {error}')

    def _remember_room_start(self, max_csv_objects, room_config):
        import copy
        self._room_start_recipe = (max_csv_objects, copy.deepcopy(room_config))
        self._scene_dirty = False

    def _reset_current_room(self, value):
        try:
            if not isinstance(value, dict) or value.get('confirmed') is not True:
                raise ValueError('추가·삭제·배치 편집이 초기화됩니다. 공간 초기화 버튼에서 확인해 주세요.')
            if self._current_busy_operation() or self._external_drive_key or self._pressed_keys:
                raise ValueError('진행 중인 작업과 주행을 멈춘 뒤 다시 시도하세요.')
            recipe = getattr(self, '_room_start_recipe', None)
            if recipe is None:
                raise ValueError('초기 공간 정보가 없습니다. 먼저 공간을 생성하거나 불러오세요.')
            import copy
            count, config = copy.deepcopy(recipe)
            # Check the source before removing the current scene.
            if config and config['kind'] == 'saved':
                config = snapshot_config(self, Path(config['snapshot_path']).name)
            elif config:
                config = validate_room(config)
            self._begin_experience('현재 공간 초기화', '현재 공간을 다시 불러옵니다. 편집 내용과 AGV 배치는 초기화됩니다.')
            self._request_scene_build(max_csv_objects=count, room_config=config)
        except Exception as error:
            self._set_status(f'공간 초기화 실패: {error}')

    def _prepare_temperature_mission(self):
        if is_ai_graduate_room({'kind':self._room_kind,'usd_path':self._room_source}):
            self._set_status('AI대학원동은 AGV 주행 전용입니다. 습도 미션·자동체험·Worker는 다른 실습 공간에서 사용하세요.')
            return
        if self._rebuild_task is not None and not self._rebuild_task.done():
            self._set_status("장면 복원·생성이 끝난 뒤 습도 미션을 준비하세요.")
            return
        stage = self._stage
        if not self._room_ready():
            self._set_status("공간이 준비되지 않았습니다. Warehouse 또는 방 교체를 먼저 실행하세요.")
            return
        if not stage.GetPrimAtPath(ROBOT_PATH).IsValid():
            self._set_status("ROBOT NOT INSTALLED: complete Lab 1-C: Add PhysX AGV first.")
            return
        target = stage.GetPrimAtPath(TARGET_PATH)
        if not target.IsValid():
            self._set_status("습도 미션 대상 box_02가 없습니다. 실습 객체 포함으로 공간을 교체하거나 box_02를 추가하세요.")
            return
        self._begin_experience("Lab 2 · Humidity", "습도 20초 내 초기값보다 20%p 상승을 감지한 AGV가 box_02를 정확히 두 번 밀고 안전 후퇴합니다.")
        self._select_path(TARGET_PATH)
        self._initialize_mission_from_stage(armed=True)
        self._lab2_prepared = True
        self._narrate("sensor", f"UI 시뮬레이션 준비 완료: {self._humidity:.1f}%RH. 첫 버튼 입력부터 20초 안에 +5%p를 네 번 누르면 미션이 시작됩니다.")
        armed_attr = target.GetAttribute("lab:temperatureSensorTarget")
        if not armed_attr.IsValid() or not bool(armed_attr.Get()):
            raise RuntimeError("Lab 2 sensor target could not be armed.")
        carb.log_info("[Lab 2 Prepare] target=box_02 armed=True temperature=25.0 C")
        self._set_status(
            "Lab 2 READY: humidity sensor armed. "
            "Raise relative humidity by 20 percentage points within 20 seconds."
        )
        self._show_experience_popup("진행", "Lab 2 · Humidity", "습도 센서 준비 완료 · 20초 내 초기값보다 20%p 상승하면 AGV가 출발합니다.")

    def _add_agv_to_twin(self):
        stage = self._stage
        if self._rebuild_task is not None and not self._rebuild_task.done():
            self._next_experience_after_rebuild = True
            self._set_status("창고 생성이 끝나면 AGV를 자동으로 추가합니다. 잠시 기다리세요.")
            return
        if not self._room_ready():
            self._set_status("공간을 먼저 생성한 뒤 Add AGV를 실행하세요.")
            return
        if self._agv_install_task is not None and not self._agv_install_task.done():
            self._set_status("PhysX AGV installation is already running.")
            return
        if self._agv_is_installed():
            self._reveal_agv()
            self._ensure_robot_camera_stream()
            self._select_path(ROBOT_PATH)
            self._frame_prim_with_teaching_camera(stage.GetPrimAtPath(ROBOT_PATH))
            self._set_status("AGV 위치로 화면을 이동했습니다. 화살표 키로 주행할 수 있습니다.")
            return
        self._begin_experience("Lab 1-C · PhysX AGV", "동적 강체 AGV와 전방 카메라를 창고에 설치합니다.")
        self._agv_install_task = asyncio.ensure_future(self._install_agv_async())
        self._agv_install_task.add_done_callback(self._on_agv_install_done)

    def _on_agv_install_done(self, task):
        if task.cancelled():
            return
        error = task.exception()
        if error is not None:
            self._experience_active = False
            self._experience_completion_pending = False
            detail = f"{type(error).__name__}: {error}"
            carb.log_error("[Interactive Physics Lab] PhysX AGV install failed:\n" +
                           "".join(traceback.format_exception(type(error), error, error.__traceback__)))
            self._set_status(f"Robot installation failed: {detail}")
            self._show_experience_popup("설치 실패", "Lab 1-C · PhysX AGV",
                                        f"{detail}\nC Add AGV를 다시 누르면 불완전한 차량을 복구합니다.")
        else:
            self._complete_experience("AGV 설치가 완료되었습니다. 화살표 키와 방향 버튼으로 주행할 수 있습니다.")

    def _request_scene_build(self, max_csv_objects, prepare_lab2=False, room_config=None):
        if self._rebuild_task is not None and not self._rebuild_task.done():
            self._set_status("초기화가 이미 진행 중입니다. 잠시 기다리세요.")
            return
        if self._agv_install_task is not None and not self._agv_install_task.done():
            self._agv_install_task.cancel()
        self._show_main_view(silent=True)
        self._restore_previous_view(silent=True)
        self._pressed_keys.clear()
        self._external_drive_key = None
        self._external_drive_lease_until = 0.0
        self._robot_camera_path = None
        if self._camera_stream_task is not None and not self._camera_stream_task.done():
            self._camera_stream_task.cancel()
        self._camera_stream_task = None
        if self._external_control_server is not None:
            self._external_control_server.set_camera_frame_path(None)
        self._teaching_camera_target = None
        self._agent_motion_epoch += 1
        self._agent_action_active = False
        self._person_safety_mode = "CLEAR"
        self._person_avoidance_until = 0.0
        if self._external_drive_task is not None and not self._external_drive_task.done():
            self._external_drive_task.cancel()
        self._external_drive_task = None
        if self._temperature_mission_task is not None and not self._temperature_mission_task.done():
            self._temperature_mission_task.cancel()
        self._temperature_mission_task = None
        if self._face_restore_task is not None and not self._face_restore_task.done():
            self._face_restore_task.cancel()
        self._face_restore_task = None
        self._command_robot_velocity(0.0, 0.0)
        self._rebuild_task = asyncio.ensure_future(
            self._create_demo_scene_async(
                max_csv_objects=max_csv_objects,
                prepare_lab2=prepare_lab2,
                room_config=room_config,
            )
        )
        self._rebuild_task.add_done_callback(self._on_rebuild_done)

    def _on_rebuild_done(self, task):
        if task.cancelled():
            self._next_experience_after_rebuild = False
            return
        error = task.exception()
        if error is not None:
            self._next_experience_after_rebuild = False
            carb.log_error(f"[Interactive Physics Lab] Rebuild failed: {error}")
            self._set_status(
                "초기화 또는 CSV 장면 생성에 실패했습니다.\n"
                f"{type(error).__name__}: {error}\n\n"
                "Window > Console에서 상세 로그를 확인하세요."
            )
            self._experience_active = False
        else:
            if self._next_experience_after_rebuild:
                self._next_experience_after_rebuild = False
                self._add_agv_to_twin()
                return
            self._complete_experience("장면 생성과 물리 초기화가 완료되었습니다.")

    def _prepare_next_experience(self):
        if is_ai_graduate_room({'kind':self._room_kind,'usd_path':self._room_source}):
            self._set_status('AI대학원동은 AGV 주행 전용입니다. 습도 미션·자동체험·Worker는 다른 실습 공간에서 사용하세요.')
            return
        """Restore a clean warehouse and AGV baseline for the next lesson."""
        if self._rebuild_task is not None and not self._rebuild_task.done():
            self._set_status("다음 체험 준비가 이미 진행 중입니다. 잠시 기다리세요.")
            return
        if self._face3d_task is not None and not self._face3d_task.done():
            self._set_status("얼굴 매핑이 진행 중입니다. 완료 후 다시 시도하세요.")
            return
        if self._experience_task is not None and not self._experience_task.done():
            self._experience_task.cancel()
        self._experience_task = None
        self._experience_progress = ""
        self._agent_motion_epoch += 1
        for task in (self._agent_turn_task, self._temperature_mission_task, self._external_drive_task):
            if task is not None and not task.done():
                task.cancel()
        self._agent_turn_task = None
        self._temperature_mission_task = None
        self._external_drive_task = None
        self._agent_busy = False
        self._agent_action_active = False
        self._mission_active = False
        self._mission_latched = False
        self._worker_auto_active = False
        self._pressed_keys.clear()
        self._external_drive_key = None
        self._external_drive_lease_until = 0.0
        self._person_safety_mode = "CLEAR"
        self._person_avoidance_until = 0.0
        self._reset_obstacle_avoidance()
        self._command_robot_velocity(0.0, 0.0)
        self._zero_robot_velocities()
        self._next_experience_after_rebuild = True
        self._begin_experience("다음 체험 준비", "이전 체험의 주행·키·미션·물리 상태를 제거하고 기준 창고와 AGV를 복원합니다.")
        self._show_experience_popup("진행", "다음 체험 준비", "창고 장면을 기준 상태로 다시 생성합니다.")
        self._request_scene_build(max_csv_objects=None, room_config=self._current_warehouse_config())

    def _start_experience(self, experience_id):
        if is_ai_graduate_room({'kind':self._room_kind,'usd_path':self._room_source}):
            self._set_status('AI대학원동은 AGV 주행 전용입니다. 습도 미션·자동체험·Worker는 다른 실습 공간에서 사용하세요.')
            return
        lessons = self._experience_definitions()
        experience_id = str(experience_id or "").strip().lower()
        if experience_id not in lessons:
            self._set_status("알 수 없는 체험입니다: " + experience_id)
            return
        if self._experience_task is not None and not self._experience_task.done():
            self._set_status("다른 체험이 자동실행 중입니다. 완료 후 다시 누르세요.")
            return
        if any(task is not None and not task.done() for task in (self._rebuild_task, self._agv_install_task, self._face3d_task)):
            self._set_status("장면 생성·AGV 설치·얼굴 매핑 완료 후 자동체험을 시작하세요.")
            return
        self._agent_motion_epoch += 1
        for task in (self._agent_turn_task, self._temperature_mission_task, self._external_drive_task):
            if task is not None and not task.done():
                task.cancel()
        self._agent_turn_task = None
        self._temperature_mission_task = None
        self._external_drive_task = None
        self._agent_busy = False
        self._agent_action_active = False
        self._mission_active = False
        self._pressed_keys.clear()
        self._external_drive_key = None
        self._external_drive_lease_until = 0.0
        self._reset_obstacle_avoidance()
        self._command_robot_velocity(0.0, 0.0)
        self._zero_robot_velocities()
        lesson = lessons[experience_id]
        self._begin_experience(lesson[0], lesson[1])
        self._experience_task = asyncio.ensure_future(self._run_experience(experience_id))
        self._experience_task.add_done_callback(self._on_experience_done)

    @staticmethod
    def _experience_definitions():
        return {
            "drive": ("체험 1 · AGV 이동·회전", "거리와 회전각 명령이 AGV의 실제 움직임으로 변환되는 과정을 관찰합니다."),
            "approach": ("체험 2 · 라벨 객체 접근", "객체 라벨로 목표를 식별하고 안전거리까지 자율 접근하는 과정을 관찰합니다."),
            "bump": ("체험 3 · 반복 충돌", "충돌로 객체 위치가 바뀐 뒤 목표를 다시 계산하여 반복 작업하는 과정을 관찰합니다."),
            "mass": ("체험 4 · 질량 비교", "같은 충돌에서도 객체 질량에 따라 이동량이 달라지는 PhysX 특성을 비교합니다."),
            "mobility": ("체험 5 · 물리 속성", "Movable과 Fixed 설정에 따른 객체의 충돌 반응 차이를 비교합니다."),
            "appearance": ("체험 6 · 외형·크기", "객체 속성 변경이 디지털 트윈 외형에 즉시 반영되는지 확인합니다."),
            "temperature": ("체험 7 · 습도 자동 미션", "습도 상승이 AGV 미션을 시작하고 box_02를 두 번 미는 과정을 관찰합니다."),
            "worker_safety": ("체험 8 · Worker 안전", "주행 중 사람 감지 시 AGV 감속·정지·회피가 적용되는지 확인합니다."),
        }

    def _experience_step(self, message):
        self._experience_progress = str(message)
        self._show_experience_popup("진행", self._experience_name, message)
        self._set_status(self._experience_progress)

    def _on_experience_done(self, task):
        if task.cancelled():
            self._experience_active = False
            return
        error = task.exception()
        if error is not None:
            carb.log_error(f"[Experience] automatic run failed: {error}")
            self._experience_active = False
            self._show_experience_popup("체험 실패", self._experience_name, f"{type(error).__name__}: {error}")
            self._set_status(f"체험 자동실행 오류: {type(error).__name__}: {error}")

    async def _run_experience(self, experience_id):
        """Build a clean baseline and visibly execute one selected lesson."""
        app = omni.kit.app.get_app()
        # Keep the educational meaning popup visible for its full three seconds.
        until = time.monotonic() + 3.0
        while time.monotonic() < until:
            await app.next_update_async()
        self._experience_step("이전 상태 제거 및 기준 창고 생성")
        # Automatic lessons rebuild directly instead of going through
        # _request_scene_build(). Dispose the previous camera observer before
        # its USD prim is removed, just as the public rebuild path does.
        self._show_main_view(silent=True)
        self._restore_previous_view(silent=True)
        if self._camera_stream_task is not None and not self._camera_stream_task.done():
            self._camera_stream_task.cancel()
        self._camera_stream_task = None
        self._robot_camera_path = None
        if self._external_control_server is not None:
            self._external_control_server.set_camera_frame_path(None)
        await app.next_update_async()
        await self._create_demo_scene_async(max_csv_objects=None, prepare_lab2=False,
                                           room_config=self._current_warehouse_config())
        self._experience_step("PhysX AGV 설치")
        await self._install_agv_async()
        result = None
        if experience_id == "drive":
            self._experience_step("AGV 전진 1.5m")
            result = await self._agent_drive_distance("forward", 1.5)
            if result.get("ok"):
                self._experience_step("AGV 오른쪽 45도 회전")
                result = await self._agent_turn_degrees("right", 45.0)
        elif experience_id == "approach":
            self._experience_step("box_01 라벨 인식 및 안전거리 접근")
            result = await self._experience_approach_object("box_01", 1.25)
        elif experience_id == "bump":
            self._experience_step("box_02 충돌·후퇴 3회 반복")
            result = await self._experience_bump_object("box_02", 3)
        elif experience_id == "mass":
            self._experience_step("pallet_01 10kg 충돌")
            self._agent_tool_set_mass({"object_id": "pallet_01", "mass_kg": 10.0})
            first = await self._experience_bump_object("pallet_01", 1)
            pallet, _ = self._agent_find_object("pallet_01")
            if pallet is not None:
                self._reset_single_prim(pallet)
            self._reset_robot()
            await self._wait_frames(app, 30)
            self._experience_step("pallet_01 500kg 충돌 비교")
            self._agent_tool_set_mass({"object_id": "pallet_01", "mass_kg": 500.0})
            result = await self._experience_bump_object("pallet_01", 1) if first.get("ok") else first
        elif experience_id == "mobility":
            self._experience_step("box_01 Movable 충돌")
            self._agent_tool_set_mobility({"object_id": "box_01", "mobility": "movable"})
            first = await self._experience_bump_object("box_01", 1)
            box, _ = self._agent_find_object("box_01")
            if box is not None:
                self._reset_single_prim(box)
            self._reset_robot()
            await self._wait_frames(app, 30)
            self._experience_step("box_01 Fixed 충돌 비교")
            self._agent_tool_set_mobility({"object_id": "box_01", "mobility": "fixed"})
            result = await self._experience_bump_object("box_01", 1) if first.get("ok") else first
        elif experience_id == "appearance":
            self._experience_step("box_04 색상을 빨강으로 변경")
            result = self._agent_tool_set_color({"object_id": "box_04", "color": "red"})
            await self._wait_frames(app, 90)
            if result.get("ok"):
                self._experience_step("box_04 크기를 1.25배로 변경")
                result = self._agent_tool_scale({"object_id": "box_04", "factor": 1.25})
                await self._wait_frames(app, 90)
        elif experience_id == "temperature":
            self._experience_step("습도 시뮬레이션 준비 · 50%")
            self._prepare_temperature_mission()
            # Restore the selected lesson title overwritten by Lab 2 setup.
            title, meaning = self._experience_definitions()[experience_id]
            self._experience_name = title
            for humidity in (55, 60, 65, 70):
                self._experience_step(f"습도 시뮬레이션 · {humidity}%")
                self._adjust_humidity(5.0)
                await self._wait_frames(app, 45)
            if self._temperature_mission_task is not None:
                self._experience_step("box_02 두 번 밀기 자동 미션")
                await self._temperature_mission_task
            result = {"ok": self._mission_state == "COMPLETED", "message": f"습도 미션 {self._mission_state}, Push {self._push_count}/2"}
        else:
            self._experience_step("Worker를 AGV 주행 경로에 배치")
            worker = self._stage.GetPrimAtPath(WORKER_PATH)
            translate = self._find_xform_op(UsdGeom.Xformable(worker), UsdGeom.XformOp.TypeTranslate) if worker.IsValid() else None
            if translate is not None:
                translate.Set(Gf.Vec3d(-0.5, -3.0, 0.0))
            self._experience_step("AGV의 Worker 감속·정지·회피 관찰")
            self._timeline.play()
            started = time.monotonic()
            while time.monotonic() - started < 6.0:
                self._command_robot_velocity(self.AUTO_LINEAR_SPEED, 0.0)
                await app.next_update_async()
            self._command_robot_velocity(0.0, 0.0)
            result = {"ok": True, "message": f"Worker 안전상태={self._person_safety_mode}"}
        self._command_robot_velocity(0.0, 0.0)
        self._experience_progress = ""
        if isinstance(result, dict) and result.get("ok"):
            self._complete_experience(result.get("message", "자동체험을 완료했습니다."))
            self._set_status("체험 자동실행 완료: " + str(result.get("message", "PASS")))
        else:
            message = result.get("error", "자동체험 실패") if isinstance(result, dict) else str(result)
            self._experience_active = False
            self._show_experience_popup("체험 실패", self._experience_name, message)
            self._set_status("체험 자동실행 실패: " + message)

    async def _create_demo_scene_async(self, max_csv_objects=None, prepare_lab2=False, room_config=None):
        """Hard-reset only this extension's namespace, then rebuild from CSV.

        Timeline stop invalidates the active physics state. Waiting for Kit update
        frames after stop and Prim deletion prevents stale PhysX objects from the
        previous run from colliding with newly authored Prims.
        """
        self._navigation.cancel()
        self._set_status("공간 교체 1/8 · 시뮬레이션 정지·기존 물리 상태 정리 중…")
        self._timeline.stop()
        selection = omni.usd.get_context().get_selection()
        clear_selection = getattr(selection, "clear_selected_prim_paths", None)
        if callable(clear_selection):
            clear_selection()
        else:
            selection.set_selected_prim_paths([], True)

        app = omni.kit.app.get_app()
        await app.next_update_async()

        # The extension may start before Isaac Sim creates its initial USD Stage.
        stage = None
        for _ in range(300):
            stage = self._stage
            if stage is not None:
                break
            await app.next_update_async()
        if stage is None:
            self._set_status(
                "USD Stage가 준비되지 않았습니다. File > New 실행 후 "
                "Lab 1-A 또는 Lab 1-B 버튼을 누르세요."
            )
            return

        if room_config and room_config['kind'] == 'saved':
            await self._restore_room_snapshot_async(room_config)
            self._remember_room_start(max_csv_objects, room_config)
            return

        world = stage.GetPrimAtPath("/World")
        if not world.IsValid():
            world = UsdGeom.Xform.Define(stage, "/World").GetPrim()
            stage.SetDefaultPrim(world)

        self._set_status('공간 교체 2/8 · 기존 공간 제거 중…')
        await app.next_update_async()
        # AGV repairs live in the session layer. Clear this app's repair and
        # mask a robot inherited from a saved/reference scene before rebuilding.
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            stage.RemovePrim(ROBOT_PATH)
            stage.OverridePrim(ROBOT_PATH).SetActive(False)
        old = stage.GetPrimAtPath(LAB_ROOT)
        if old.IsValid():
            stage.RemovePrim(LAB_ROOT)
        self._robot_motion_prim_path = None
        self._dynamic_robot_handle = _dynamic_control.INVALID_HANDLE
        self._mobile_base_drive_paths = None
        self._gripper_open = True

        # Let USD notices and PhysX stage-update processing finish the deletion.
        await app.next_update_async()
        await app.next_update_async()

        lab = UsdGeom.Xform.Define(stage, LAB_ROOT).GetPrim()
        self._room_kind = room_config['kind'] if room_config else ('warehouse' if max_csv_objects is None else 'minimal')
        self._room_branding = room_config.get('branding', 'gist') if room_config else 'gist'
        self._room_source = room_config.get('usd_path', '') if room_config else ''
        lab.CreateAttribute('lab:roomReady', Sdf.ValueTypeNames.Bool).Set(False)
        lab.CreateAttribute('lab:roomKind', Sdf.ValueTypeNames.String).Set(self._room_kind)
        lab.CreateAttribute('lab:branding', Sdf.ValueTypeNames.String).Set(self._room_branding)
        UsdGeom.Xform.Define(stage, OBJECT_ROOT)
        await app.next_update_async()

        self._set_status('공간 교체 3/8 · 바닥·기본 실습 객체 준비 중…')
        await app.next_update_async()
        if not self._ensure_physics_scene():
            raise RuntimeError("Could not create a valid PhysX PhysicsScene.")
        # Avoid PhysicsSchemaTools.addGroundPlane on Isaac Sim 4.5 because its
        # USD 22.x implementation can re-add existing named xformOps during a
        # same-session rebuild. A thin fixed cube is equivalent for this lab.
        if not is_ai_graduate_room(room_config):
            self._create_cube(
                GROUND_PATH,
                position=(0.0, 0.0, -0.05),
                scale=(10.0, 10.0, 0.05),
                color=(0.25, 0.25, 0.25),
                mobility="fixed",
                mass=1000.0,
            )
        await app.next_update_async()

        object_count = await self._load_objects_from_csv_async(
            app, max_objects=max_csv_objects, room_config=room_config
        )
        await build_environment(self, room_config, app)
        self._set_status("공간 교체 8/8 · Worker·실습 객체·카메라 준비 중…")
        await app.next_update_async()
        include_lab = not is_ai_graduate_room(room_config) and (room_config is None or room_config["include_lab"])
        worker_created = False
        branding_created = False
        if max_csv_objects is None:
            if self._room_kind != "usd":
                if self._room_branding == "dongshin":
                    branding = create_dongshin_branding(stage, LAB_ROOT, OBJECT_ROOT)
                    branding_created = branding["created_count"] == 4
                else:
                    branding_created = self._create_gist_wall_sign()
            if include_lab:
                worker_created = self._create_worker_custom()
            await app.next_update_async()
        if include_lab:
            self._create_cube(
                PUSHER_PATH,
                position=(-2.0, 0.0, 0.5),
                scale=(0.5, 0.5, 0.5),
                color=(0.85, 0.2, 0.2),
                mobility="movable",
                mass=20.0,
            )
            pusher = stage.GetPrimAtPath(PUSHER_PATH)
            pusher.CreateAttribute("lab:isPusher", Sdf.ValueTypeNames.Bool).Set(True)
            self._pusher_has_launched = False

        first_object = stage.GetPrimAtPath(f"{OBJECT_ROOT}/box_01")
        if first_object.IsValid():
            self._select_path(str(first_object.GetPath()))
        else:
            self._editor_clear_selection()
        lab.CreateAttribute('lab:roomReady', Sdf.ValueTypeNames.Bool).Set(max_csv_objects is None)
        await app.next_update_async()
        self._initialize_mission_from_stage(armed=False)
        if max_csv_objects is None:
            # Use the same oblique teaching-camera direction as Zoom Out,
            # framing the warehouse immediately instead of retaining the old view.
            self._focus_warehouse_view()
            if worker_created:
                self._orient_worker_toward_active_camera()
        self._remember_room_start(max_csv_objects, room_config)
        mode_text = (ROOM_LABELS[self._room_kind] if room_config else ("최소 시험" if max_csv_objects is not None else "전체 CSV"))
        branding_label = "동신여고" if self._room_branding == "dongshin" else "GIST"
        self._set_status(
            f"{mode_text} 장면 생성 완료: {object_count} CSV objects\n"
            f"Worker_Custom: {'worker_01 생성 완료' if worker_created else '미생성'}\n"
            f"{branding_label} 벽 3면·바닥 로고: {'자동 매핑 완료' if branding_created else '이미지 없음'}\n"
            f"실습 객체: {'포함' if include_lab else '없음 · 객체 편집으로 추가 가능'}\n\n"
            "PhysX AGV는 아직 설치되지 않았습니다.\n"
            "다음 단계: Lab 1-C: Add PhysX AGV"
        )

    def _create_worker_custom(self):
        """Add Worker_Custom without changing the existing v3.5.1 lab flow."""
        worker_usd = (
            self._extension_root()
            / "assets"
            / "characters"
            / "Worker_Custom_Face_Prim"
            / "Worker_Custom"
            / "Worker_Custom.usda"
        )
        if not worker_usd.is_file():
            carb.log_error(f"[Worker] Asset not found: {worker_usd}")
            return False

        old = self._stage.GetPrimAtPath(WORKER_PATH)
        if old.IsValid():
            self._stage.RemovePrim(WORKER_PATH)

        worker = UsdGeom.Xform.Define(self._stage, WORKER_PATH).GetPrim()
        self._author_clean_transform(
            worker,
            position=(self.WORKER_START_X, self.WORKER_START_Y, 0.0),
            scale=(1.0, 1.0, 1.0),
        )
        worker.CreateAttribute("lab:initialPosition", Sdf.ValueTypeNames.Double3).Set(
            Gf.Vec3d(self.WORKER_START_X, self.WORKER_START_Y, 0.0)
        )
        worker.CreateAttribute("lab:initialScale", Sdf.ValueTypeNames.Float3).Set(
            Gf.Vec3f(1.0, 1.0, 1.0)
        )
        worker.CreateAttribute("lab:initialOrientation", Sdf.ValueTypeNames.Quatf).Set(
            Gf.Quatf(1.0, Gf.Vec3f(0.0))
        )
        worker.CreateAttribute("lab:mobility", Sdf.ValueTypeNames.String).Set("fixed")
        worker.CreateAttribute("lab:massKg", Sdf.ValueTypeNames.Float).Set(70.0)
        worker.CreateAttribute("lab:modelIndex", Sdf.ValueTypeNames.Int).Set(1)
        worker.CreateAttribute("lab:objectType", Sdf.ValueTypeNames.String).Set("character")
        worker.CreateAttribute("lab:semanticLabel", Sdf.ValueTypeNames.String).Set("person")
        worker.CreateAttribute("lab:source", Sdf.ValueTypeNames.String).Set("Worker_Custom.usda")
        worker.CreateAttribute("lab:workerScale", Sdf.ValueTypeNames.Float).Set(1.0)

        collision = UsdGeom.Capsule.Define(
            self._stage, f"{WORKER_PATH}/CollisionShape"
        )
        collision.CreateAxisAttr(UsdGeom.Tokens.z)
        collision.CreateRadiusAttr(0.28)
        collision.CreateHeightAttr(1.20)
        collision.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
        self._author_clean_transform(
            collision.GetPrim(),
            position=(0.0, 0.0, 0.88),
            scale=(1.0, 1.0, 1.0),
        )
        UsdPhysics.CollisionAPI.Apply(collision.GetPrim())

        UsdGeom.Xform.Define(self._stage, f"{WORKER_PATH}/ModelVisuals")
        asset = UsdGeom.Xform.Define(
            self._stage, f"{WORKER_PATH}/ModelVisuals/WorkerAsset"
        ).GetPrim()
        asset.GetReferences().AddReference(worker_usd.as_posix())
        UsdGeom.Imageable(asset).CreateVisibilityAttr().Set(UsdGeom.Tokens.inherited)
        self._author_clean_transform(
            asset,
            position=(0.0, 0.0, 0.0),
            scale=(0.01, 0.01, 0.01),
        )
        # Debra's authored forward direction faces away from the warehouse's
        # default teaching view. Rotate only the visual reference so the face
        # is seen straight-on while the parent Prim and capsule stay unchanged.
        asset_orient = self._find_xform_op(
            UsdGeom.Xformable(asset), UsdGeom.XformOp.TypeOrient
        )
        if asset_orient is not None:
            asset_orient.Set(Gf.Quatf(0.0, Gf.Vec3f(0.0, 0.0, 1.0)))
        self._worker_visual_yaw_deg = 180.0
        # 2D-only mode: never restore runtime-generated face/head geometry.
        # The original Debra skinned mesh and animation remain untouched.

        self._apply_mobility(worker, "kinematic")
        self._set_mass_attr(worker, 70.0)

        carb.log_info(
            f"[Worker] Created {WORKER_PATH} from {worker_usd} "
            "(visual scale=0.01, yaw=180 deg; front view)"
        )
        return True

    def _open_face_file_picker(self):
        """Open Kit's file browser for a JPG/PNG portrait."""
        try:
            from omni.kit.window.filepicker import FilePickerDialog

            self._close_face_file_picker()
            self._face_file_picker = FilePickerDialog(
                "Select Worker Face Photo",
                allow_multi_selection=False,
                apply_button_label="Select",
                click_apply_handler=self._on_face_file_selected,
                click_cancel_handler=lambda *args: self._close_face_file_picker(),
            )
            show = getattr(self._face_file_picker, "show", None)
            if callable(show):
                show()
        except Exception as error:
            carb.log_error(f"[Worker Face] File picker failed: {error}")
            self._set_status(
                "얼굴 사진 탐색창을 열지 못했습니다.\n"
                f"{type(error).__name__}: {error}"
            )

    def _close_face_file_picker(self):
        picker = getattr(self, "_face_file_picker", None)
        if picker is not None:
            hide = getattr(picker, "hide", None)
            if callable(hide):
                try:
                    hide()
                except Exception:
                    pass
        self._face_file_picker = None

    def _on_face_file_selected(self, filename, directory, *args):
        self._face_trace(
            "00",
            f"file-picker callback filename={filename!r} directory={directory!r} extras={args!r}",
        )
        raw_values = [filename, directory, *args]
        candidates = []
        for value in raw_values:
            if value is None or str(value) == "":
                continue
            path = Path(str(value)).expanduser()
            candidates.append(path)
            if not path.is_absolute():
                for base in raw_values:
                    if base is None or str(base) in {"", str(value)}:
                        continue
                    base_path = Path(str(base)).expanduser()
                    if base_path.is_absolute() or base_path.is_dir():
                        candidates.append(base_path / path)
        candidate = next(
            (
                path.resolve()
                for path in candidates
                if path.suffix.lower() in {".jpg", ".jpeg", ".png"} and path.is_file()
            ),
            None,
        )
        self._close_face_file_picker()
        if candidate is None:
            self._set_status(
                "선택한 얼굴 사진 경로를 확인하지 못했습니다.\n"
                "JPG, JPEG 또는 PNG 파일을 다시 선택하세요."
            )
            self._face_trace("00-FAIL", f"no valid candidate among {candidates!r}")
            return
        self._selected_face_path = str(candidate)
        self._face_trace("01", f"photo selected: {candidate}")
        if self._face_path_model is not None:
            self._face_path_model.set_value(candidate.name)
        self._set_status(
            f"얼굴 사진 선택: {candidate.name}\n"
            "Map 2D Face를 선택하세요."
        )

    def _select_uploaded_face(self, value):
        """Accept a browser-uploaded portrait after the HTTP bridge validates it."""
        candidate = Path(str(value or "")).expanduser().resolve()
        self._face_trace("02", f"browser photo candidate: {candidate}")
        if candidate.suffix.lower() not in {".jpg", ".jpeg", ".png"} or not candidate.is_file():
            self._set_status(f"브라우저 얼굴사진을 찾을 수 없습니다:\n{candidate}")
            self._face_trace("02-FAIL", "browser photo validation failed")
            return
        self._selected_face_path = str(candidate)
        if self._face_path_model is not None:
            self._face_path_model.set_value(candidate.name)
        self._set_status(
            f"브라우저 얼굴사진 선택 완료: {candidate.name}\n"
            "Map 2D Face를 누르세요."
        )
        self._face_trace("03", f"browser photo accepted: {candidate.name}")

    def _resolve_selected_face(self):
        """Return the selected portrait, recovering the latest web upload.

        The browser upload response can arrive one Kit frame before its queued
        selection command is drained.  Falling back to the newest validated
        upload removes that timing window and avoids a false "no face image"
        message when the user immediately presses a mapping button.
        """
        if self._selected_face_path:
            candidate = Path(self._selected_face_path).expanduser()
            if candidate.is_file():
                return candidate.resolve()
        upload_dir = self._extension_root() / "assets" / "web_control" / "uploads"
        uploaded = []
        if upload_dir.is_dir():
            uploaded = [
                path
                for path in upload_dir.iterdir()
                if path.is_file() and path.suffix.lower() in {".jpg", ".jpeg", ".png"}
            ]
        if not uploaded:
            return None
        candidate = max(uploaded, key=lambda path: path.stat().st_mtime_ns).resolve()
        self._selected_face_path = str(candidate)
        if self._face_path_model is not None:
            self._face_path_model.set_value(candidate.name)
        self._face_trace("04", f"recovered latest browser upload: {candidate}")
        return candidate

    def _start_2d_face_mapping(self):
        portrait = self._resolve_selected_face()
        self._face_trace("09", f"Map 2D requested; portrait={portrait}")
        if portrait is None or not portrait.is_file():
            self._set_status("Browse Face Photo로 얼굴 사진을 먼저 선택하세요.")
            self._face_trace("09-FAIL", "selected portrait is missing")
            return
        if self._face3d_task is not None and not self._face3d_task.done():
            self._set_status("얼굴 매핑 작업이 이미 진행 중입니다.")
            self._face_trace("09-FAIL", "another face task is active")
            return
        self._face3d_progress = "2D 얼굴 텍스처 매핑 중"
        self._face3d_task = asyncio.ensure_future(
            self._run_2d_face_mapping_async()
        )

    async def _run_2d_face_mapping_async(self):
        try:
            self._set_status("선택한 사진을 Worker 얼굴 텍스처에 매핑 중입니다...")
            await omni.kit.app.get_app().next_update_async()
            await self._apply_selected_face_async()
        except asyncio.CancelledError:
            raise
        except Exception as error:
            self._face_trace("19-FAIL", f"uncaught 2D task error: {error}\n{traceback.format_exc()}")
            self._set_status(f"2D 얼굴 매핑 실패: {type(error).__name__}: {error}")
        finally:
            self._face3d_progress = ""

    async def _apply_selected_face_async(self):
        """Bake the selected portrait and update the live Shader immediately."""
        portrait = self._resolve_selected_face()
        self._face_trace("10", f"2D mapping entered; portrait={portrait}")
        if portrait is None or not portrait.is_file():
            self._set_status("Browse Face Photo로 얼굴 사진을 먼저 선택하세요.")
            self._face_trace("10-FAIL", "portrait path is empty or not a file")
            return False

        asset_root = (
            self._extension_root()
            / "assets"
            / "characters"
            / "Worker_Custom_Face_Prim"
        )
        head_dir = asset_root / "Debra" / "Materials" / "Textures" / "Std_Skin_Head"
        original = head_dir / "Std_Skin_Head_BaseColor_Original.jpg"
        installed = head_dir / "Std_Skin_Head_BaseColor.jpg"
        canonical = (
            asset_root
            / "Worker_Custom"
            / "Textures"
            / "Worker_Custom_Head_BaseColor.jpg"
        )
        runtime_texture = (
            asset_root
            / "Worker_Custom"
            / "Textures"
            / "Runtime"
            / f"Worker_Custom_Head_{time.time_ns()}.jpg"
        )
        self._face_trace(
            "11",
            f"paths: original={original.is_file()} installed={installed.is_file()} "
            f"canonical_parent={canonical.parent.is_dir()}",
        )

        try:
            source = original if original.is_file() else installed
            if not source.is_file():
                raise FileNotFoundError(f"Debra head texture not found: {source}")
            self._face_trace("12", f"mapping source={source.name} photo={portrait.name}")
            project_root = self._extension_root().parents[1]
            face_python = project_root / ".face2d_env" / "bin" / "python"
            legacy_face_python = project_root / ".face3d_env" / "bin" / "python"
            if not face_python.is_file() and legacy_face_python.is_file():
                # Existing users do not need to reinstall: the former Face3D
                # environment already contains Pillow required by 2D mapping.
                face_python = legacy_face_python
            mapper = self._extension_root() / "edu_interactive_physics" / "face_mapper_runtime.py"
            if not face_python.is_file():
                raise FileNotFoundError(
                    f"Face2D Python not found: {face_python}. Run ./install_face2d.sh first."
                )
            if not mapper.is_file():
                raise FileNotFoundError(f"Face mapper script not found: {mapper}")
            map_command = [
                str(face_python), str(mapper),
                "--source", str(source),
                "--photo", str(portrait),
                "--output", str(runtime_texture),
            ]
            landmark_model = self._extension_root() / "assets" / "face3d" / "face_landmarker.task"
            if landmark_model.is_file():
                map_command.extend(["--model", str(landmark_model)])
            self._face_trace("12A", "mapper subprocess: " + " ".join(map_command))
            mapped = await asyncio.to_thread(
                subprocess.run,
                map_command,
                capture_output=True,
                text=True,
                timeout=60,
                check=False,
            )
            self._face_trace(
                "12B",
                f"mapper returncode={mapped.returncode} "
                f"stdout={(mapped.stdout or '').strip()[-1000:]!r} "
                f"stderr={(mapped.stderr or '').strip()[-1000:]!r}",
            )
            if mapped.returncode != 0:
                details = (mapped.stderr or mapped.stdout or "unknown mapper error").strip()
                raise RuntimeError(details[-1200:])
            self._face_trace(
                "13",
                f"texture baked: exists={runtime_texture.is_file()} "
                f"bytes={runtime_texture.stat().st_size if runtime_texture.is_file() else 0}",
            )
            # Canonical copies make the chosen face persist after rebuilding
            # the Lab or restarting Isaac Sim.  The unique runtime path below
            # forces Hydra to bypass a previously cached texture.
            shutil.copy2(runtime_texture, canonical)
            shutil.copy2(runtime_texture, installed)
            self._face_trace("14", "canonical and installed textures replaced")

            stage = self._stage
            worker = stage.GetPrimAtPath(WORKER_PATH) if stage else None
            self._face_trace(
                "15",
                f"stage={stage is not None} worker_valid={bool(worker and worker.IsValid())}",
            )
            if worker is None or not worker.IsValid():
                self._set_status(
                    f"얼굴 매핑 파일 저장 완료: {portrait.name}\n"
                    "Lab 1-B를 실행하면 새 얼굴이 적용됩니다."
                )
                self._face_trace("18", "texture saved; live Worker absent, deferred to next Lab 1-B")
                return True

            shader_path = (
                f"{WORKER_PATH}/ModelVisuals/WorkerAsset/ManRoot/Debra/"
                "CC_Base_Body/CC_Base_Body/CC_Base_Body/Std_Skin_Head/"
                "Material/Shader"
            )
            shader_candidates = []
            expected_shader = stage.GetPrimAtPath(shader_path)
            if expected_shader.IsValid():
                shader_candidates.append(expected_shader)
            for prim in Usd.PrimRange(worker):
                texture_attr = prim.GetAttribute("inputs:diffuse_texture")
                if not texture_attr.IsValid():
                    continue
                current_value = texture_attr.Get()
                identity = f"{prim.GetPath()} {current_value}".lower()
                if "std_skin_head" in identity and prim not in shader_candidates:
                    shader_candidates.append(prim)
            self._face_trace(
                "16",
                "head shader candidates="
                + (", ".join(str(item.GetPath()) for item in shader_candidates) or "NONE"),
            )
            verified_paths = []
            for shader in shader_candidates:
                texture_input = shader.GetAttribute("inputs:diffuse_texture")
                if not texture_input.IsValid():
                    texture_input = shader.CreateAttribute(
                        "inputs:diffuse_texture", Sdf.ValueTypeNames.Asset, custom=False
                    )
                if not texture_input.Set(Sdf.AssetPath(runtime_texture.as_posix())):
                    raise RuntimeError(f"Failed to set diffuse texture on {shader.GetPath()}")
                applied = texture_input.Get()
                verified_paths.append(f"{shader.GetPath()} -> {applied}")
            if verified_paths:
                self._face_trace("16A", "legacy MDL texture updated: " + " | ".join(verified_paths))
            else:
                self._face_trace("16A-WARN", "legacy MDL shader absent; using PreviewSurface override")

            # Do not depend on Debra's bundled MDL compiling correctly.  Bind a
            # standard USD material directly to the head GeomSubset, using the
            # body's real UV primvar (Reallusion Debra uses ``st0``).  This is
            # also a unique material/texture path, avoiding Hydra texture cache.
            body_path = (
                f"{WORKER_PATH}/ModelVisuals/WorkerAsset/ManRoot/Debra/"
                "CC_Base_Body/CC_Base_Body/CC_Base_Body"
            )
            body_prim = stage.GetPrimAtPath(body_path)
            head_subset = stage.GetPrimAtPath(f"{body_path}/Std_Skin_Head")
            if not body_prim.IsValid() or not head_subset.IsValid():
                raise RuntimeError("Live Debra body or Std_Skin_Head subset was not found")

            uv_name = None
            for candidate in ("st0", "st", "UV0", "uv", "map1"):
                if body_prim.GetAttribute(f"primvars:{candidate}").IsValid():
                    uv_name = candidate
                    break
            if uv_name is None:
                available = [
                    attr.GetName() for attr in body_prim.GetAttributes()
                    if attr.GetName().startswith("primvars:")
                ]
                raise RuntimeError(f"Debra UV primvar not found; available={available}")

            material_path = f"{WORKER_PATH}/RuntimeFace2DMaterial"
            material = UsdShade.Material.Define(stage, material_path)
            surface = UsdShade.Shader.Define(stage, f"{material_path}/Surface")
            surface.CreateIdAttr("UsdPreviewSurface")
            surface.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.48)
            surface.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)

            reader = UsdShade.Shader.Define(stage, f"{material_path}/PrimvarReader")
            reader.CreateIdAttr("UsdPrimvarReader_float2")
            reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set(uv_name)
            reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)

            texture = UsdShade.Shader.Define(stage, f"{material_path}/Texture")
            texture.CreateIdAttr("UsdUVTexture")
            texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(
                Sdf.AssetPath(runtime_texture.as_posix())
            )
            texture.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
            texture.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
            texture.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
            texture.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
                reader.ConnectableAPI(), "result"
            )
            texture.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
            surface.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
                texture.ConnectableAPI(), "rgb"
            )
            surface.CreateOutput("surface", Sdf.ValueTypeNames.Token)
            material.CreateSurfaceOutput().ConnectToSource(
                surface.ConnectableAPI(), "surface"
            )
            UsdShade.MaterialBindingAPI.Apply(head_subset).Bind(material)

            bound_material, _ = UsdShade.MaterialBindingAPI(head_subset).ComputeBoundMaterial()
            bound_path = str(bound_material.GetPath()) if bound_material else ""
            texture_value = texture.GetInput("file").Get()
            if bound_path != material_path or runtime_texture.as_posix() not in str(texture_value):
                raise RuntimeError(
                    f"Face material verification failed: bound={bound_path} texture={texture_value}"
                )
            self._face_trace(
                "17",
                f"VISIBLE PreviewSurface bound={bound_path} uv={uv_name} texture={texture_value}",
            )
            self._set_status(
                f"Worker 얼굴 매핑 완료: {portrait.name}\n"
                "표준 PreviewSurface 재질로 현재 worker_01에 표시되었습니다."
            )
            self._face_trace("18", "2D mapping VISIBLE MATERIAL SUCCESS")
            return True
        except ImportError:
            self._set_status(
                "얼굴 매핑에 필요한 Pillow가 없습니다.\n"
                "Isaac Sim Python에 pillow를 설치한 뒤 다시 실행하세요."
            )
            self._face_trace("19-FAIL", "Pillow import failed")
            return False
        except Exception as error:
            carb.log_error(f"[Worker Face] Mapping failed: {error}")
            self._face_trace(
                "19-FAIL",
                f"{type(error).__name__}: {error}\n{traceback.format_exc()}",
            )
            self._set_status(
                "Worker 얼굴 매핑에 실패했습니다.\n"
                f"{type(error).__name__}: {error}"
            )
            return False

    def _generate_3d_face(self):
        """Retained for old queued commands; 3D mapping is intentionally disabled."""
        self._set_status("3D 얼굴 생성 기능은 제거되었습니다. Map 2D Face를 사용하세요.")
        self._face_trace("30-DISABLED", "3D command ignored in 2D-only build")

    async def _generate_3d_face_async(self, portrait):
        app = omni.kit.app.get_app()
        project_root = self._extension_root().parents[1]
        face_python = project_root / ".face3d_env" / "bin" / "python"
        generator = project_root / "tools" / "generate_face3d.py"
        model = self._extension_root() / "assets" / "face3d" / "face_landmarker.task"
        output_root = (
            self._extension_root()
            / "assets"
            / "characters"
            / "Worker_Custom_Face_Prim"
            / "Worker_Custom"
            / "GeneratedFace"
        )
        mesh_json = output_root / "face_mesh.json"
        texture = output_root / f"face_photo_{time.time_ns()}.jpg"
        self._face_trace(
            "31",
            f"runtime files: python={face_python.is_file()} generator={generator.is_file()} "
            f"model={model.is_file()} output={output_root}",
        )

        if not face_python.is_file() or not model.is_file():
            self._face3d_progress = ""
            self._set_status(
                "3D 얼굴 모듈을 먼저 한 번 설치하세요.\n"
                "프로젝트 폴더에서: chmod +x install_face3d.sh && ./install_face3d.sh"
            )
            self._face_trace("31-FAIL", "Face3D venv python or model file is missing")
            return
        if not generator.is_file():
            self._face3d_progress = ""
            self._set_status(f"3D 얼굴 생성 코드를 찾을 수 없습니다:\n{generator}")
            self._face_trace("31-FAIL", "generate_face3d.py is missing")
            return

        self._face3d_progress = "1/3 · 2D 얼굴 텍스처 매핑 중"
        self._update_busy_indicator()
        self._set_status("1/3 · 선택한 사진을 Worker 얼굴 텍스처에 매핑 중입니다...")
        await app.next_update_async()
        if not await self._apply_selected_face_async():
            self._face3d_progress = ""
            self._face_trace("32-FAIL", "2D prerequisite mapping returned False")
            return
        self._face_trace("32", "2D prerequisite mapping completed")

        self._face3d_progress = "2/3 · 3D 얼굴 랜드마크 생성 중"
        self._update_busy_indicator()
        self._set_status("2/3 · 사진에서 3D 얼굴 랜드마크와 깊이를 생성 중입니다...")
        await app.next_update_async()
        command = [
            str(face_python),
            str(generator),
            "--photo", str(portrait),
            "--model", str(model),
            "--output-json", str(mesh_json),
            "--output-texture", str(texture),
        ]
        self._face_trace("33", "subprocess start: " + " ".join(command))
        try:
            completed = await asyncio.to_thread(
                subprocess.run,
                command,
                capture_output=True,
                text=True,
                timeout=120,
                check=False,
            )
            self._face_trace(
                "34",
                f"subprocess returncode={completed.returncode} "
                f"stdout={(completed.stdout or '').strip()[-500:]!r} "
                f"stderr={(completed.stderr or '').strip()[-500:]!r}",
            )
            if completed.returncode != 0:
                details = (completed.stderr or completed.stdout or "unknown error").strip()
                raise RuntimeError(details[-800:])
            payload = json.loads(mesh_json.read_text(encoding="utf-8"))
            self._face_trace(
                "35",
                f"mesh JSON loaded: points={payload.get('point_count')} "
                f"triangles={payload.get('triangle_count')} texture={texture.is_file()}",
            )
            self._face3d_progress = "3/3 · 3D 머리 Mesh 적용 중"
            self._update_busy_indicator()
            self._set_status("3/3 · 목 경계가 고정된 3D 머리를 Worker에 적용 중입니다...")
            await app.next_update_async()
            self._face_trace("36", "creating live USD GeneratedHead3D")
            self._create_generated_face_mesh(payload, texture)
            face_mesh_path = (
                f"{WORKER_PATH}/ModelVisuals/WorkerAsset/ManRoot/Debra/"
                "GeneratedHead3D/HeadMesh"
            )
            face_mesh_prim = self._stage.GetPrimAtPath(face_mesh_path)
            if not face_mesh_prim.IsValid():
                raise RuntimeError(f"HeadMesh verification failed: {face_mesh_path}")
            self._face_trace("37", f"3D mapping SUCCESS: {face_mesh_path}")
            self._set_status(
                "3D 머리 생성 완료\n"
                f"Landmarks={payload['point_count']} · "
                f"Triangles={payload['triangle_count']}\n"
                "Debra 목 경계와 방향을 그대로 사용하는 머리 전체 Mesh입니다."
            )
        except asyncio.CancelledError:
            self._face_trace("39-FAIL", "Face3D task cancelled")
            raise
        except Exception as error:
            carb.log_error(f"[Worker Face3D] Generation failed: {error}")
            self._face_trace(
                "39-FAIL",
                f"{type(error).__name__}: {error}\n{traceback.format_exc()}",
            )
            self._set_status(
                "3D 얼굴 생성에 실패했습니다. 정면·단일 얼굴 사진을 사용하세요.\n"
                f"{type(error).__name__}: {error}"
            )
        finally:
            self._face3d_progress = ""

    def _create_generated_face_mesh(self, payload, texture_path):
        """Compatibility entry point: build a full Debra-compatible head."""
        return self._create_generated_head_mesh(payload, texture_path)

    def _create_generated_head_mesh(self, payload, texture_path):
        """Clone Debra's complete skin-head topology and texture its front.

        The former 468-vertex overlay was a detached facial mask.  Even small
        differences between MediaPipe and Debra coordinates could therefore
        place it on an ear, the cheek, or the scalp.  This implementation uses
        Debra's own ``Std_Skin_Head`` faces, so the neck seam, scale and forward
        axis are identical by construction.  MediaPipe is used only to align
        the selected photograph on the front of that stable head.
        """
        stage = self._stage
        asset_path = f"{WORKER_PATH}/ModelVisuals/WorkerAsset"
        asset = stage.GetPrimAtPath(asset_path) if stage else None
        if asset is None or not asset.IsValid():
            raise RuntimeError("WorkerAsset is not available")

        debra_path = f"{asset_path}/ManRoot/Debra"
        body_path = f"{debra_path}/CC_Base_Body/CC_Base_Body/CC_Base_Body"
        body_prim = stage.GetPrimAtPath(body_path)
        body_mesh = UsdGeom.Mesh(body_prim)
        head_subset_prim = stage.GetPrimAtPath(f"{body_path}/Std_Skin_Head")
        head_subset = UsdGeom.Subset(head_subset_prim)
        if not body_prim.IsValid() or not head_subset_prim.IsValid():
            raise RuntimeError("Debra Std_Skin_Head topology is unavailable")

        source_points = body_mesh.GetPointsAttr().Get()
        source_counts = body_mesh.GetFaceVertexCountsAttr().Get()
        source_indices = body_mesh.GetFaceVertexIndicesAttr().Get()
        selected_faces = head_subset.GetIndicesAttr().Get()
        if not source_points or not source_counts or not source_indices or not selected_faces:
            raise RuntimeError("Debra head topology contains no geometry")

        face_offsets = []
        cursor = 0
        for count in source_counts:
            face_offsets.append(cursor)
            cursor += int(count)

        source_vertex_ids = []
        source_to_new = {}
        new_counts = []
        new_indices = []
        source_face_ids = []
        for source_face in selected_faces:
            face_id = int(source_face)
            begin = face_offsets[face_id]
            count = int(source_counts[face_id])
            source_face_ids.append(face_id)
            new_counts.append(count)
            for source_id_value in source_indices[begin:begin + count]:
                source_id = int(source_id_value)
                if source_id not in source_to_new:
                    source_to_new[source_id] = len(source_vertex_ids)
                    source_vertex_ids.append(source_id)
                new_indices.append(source_to_new[source_id])
        points = [Gf.Vec3f(*map(float, source_points[index])) for index in source_vertex_ids]
        if not points:
            raise RuntimeError("Debra head vertex remapping failed")

        xs = [float(point[0]) for point in points]
        ys = [float(point[1]) for point in points]
        zs = [float(point[2]) for point in points]
        x_min, x_max = min(xs), max(xs)
        y_min, y_max = min(ys), max(ys)
        z_min, z_max = min(zs), max(zs)
        head_width = max(x_max - x_min, 1.0)
        head_height = max(z_max - z_min, 1.0)
        head_depth = max(y_max - y_min, 1.0)

        # Debra looks toward local -Y.  Find the actual nose from the most
        # forward vertices in the central, upper-middle part of the head.
        centre_x = (x_min + x_max) * 0.5
        nose_candidates = [
            point for point in points
            if abs(float(point[0]) - centre_x) <= head_width * 0.13
            and z_min + head_height * 0.38 <= float(point[2]) <= z_min + head_height * 0.72
        ]
        nose = min(nose_candidates or points, key=lambda point: float(point[1]))
        nose_x, nose_y, nose_z = map(float, nose)
        fitted_width = min(head_width * 0.78, head_height * 0.54)

        source_uvs = payload.get("texcoords") or []
        source_landmarks = payload.get("points") or []
        if len(source_uvs) <= 1 or len(source_landmarks) <= 1:
            raise RuntimeError("MediaPipe face landmarks/UVs are unavailable")
        nose_uv = source_uvs[1]
        uv_u_values = [float(uv[0]) for uv in source_uvs]
        uv_v_values = [float(uv[1]) for uv in source_uvs]
        uv_span = max(max(uv_u_values) - min(uv_u_values), 0.1)
        # MediaPipe landmark Z is positive upward in the generated payload.
        # Use its actual photo-space nose as the UV anchor.
        texcoords = []
        for point in points:
            u = float(nose_uv[0]) + ((float(point[0]) - nose_x) / fitted_width) * uv_span
            v = float(nose_uv[1]) + ((float(point[2]) - nose_z) / fitted_width) * uv_span
            texcoords.append(Gf.Vec2f(max(0.0, min(1.0, u)), max(0.0, min(1.0, v))))

        # Select only front facial polygons for the photo.  Scalp, ears, rear
        # head and neck keep a neutral skin material, so the rectangular photo
        # never appears in the hair or behind an ear.
        photo_faces = []
        index_cursor = 0
        for new_face_id, count_value in enumerate(new_counts):
            count = int(count_value)
            ids = new_indices[index_cursor:index_cursor + count]
            index_cursor += count
            face_points = [points[index] for index in ids]
            fx = sum(float(point[0]) for point in face_points) / count
            fy = sum(float(point[1]) for point in face_points) / count
            fz = sum(float(point[2]) for point in face_points) / count
            if (
                fy <= nose_y + head_depth * 0.34
                and abs(fx - nose_x) <= fitted_width * 0.53
                and nose_z - fitted_width * 0.67 <= fz <= nose_z + fitted_width * 0.56
            ):
                photo_faces.append(new_face_id)
        if len(photo_faces) < 20:
            raise RuntimeError(f"Debra front-face classification failed ({len(photo_faces)} faces)")

        root_path = f"{debra_path}/GeneratedHead3D"
        root = UsdGeom.Xform.Define(stage, root_path).GetPrim()
        # A whole cloned head is already at the exact Debra coordinates.  Old
        # mask offsets/yaw must never migrate into this new representation.
        placement_version = int(payload.get("placement_transform_version", 0) or 0)
        if placement_version >= 5:
            offset = payload.get("placement_offset_cm", [0.0, 0.0, 0.0])
            yaw = float(payload.get("placement_yaw_deg", 0.0))
        else:
            offset = [0.0, 0.0, 0.0]
            yaw = 0.0
        if not isinstance(offset, (list, tuple)) or len(offset) != 3:
            offset = [0.0, 0.0, 0.0]
        offset = [float(value) for value in offset]
        self._set_generated_face_transform(
            root, offset, yaw, pivot_cm=Gf.Vec3d(nose_x, nose_y, nose_z)
        )

        head_mesh = UsdGeom.Mesh.Define(stage, f"{root_path}/HeadMesh")
        head_mesh.CreatePointsAttr(points)
        head_mesh.CreateFaceVertexCountsAttr(new_counts)
        head_mesh.CreateFaceVertexIndicesAttr(new_indices)
        head_mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        head_mesh.CreateDoubleSidedAttr(True)
        st = UsdGeom.PrimvarsAPI(head_mesh).CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex
        )
        st.Set(texcoords)

        skin_material = UsdShade.Material.Define(stage, f"{root_path}/SkinMaterial")
        skin_surface = UsdShade.Shader.Define(stage, f"{root_path}/SkinMaterial/Surface")
        skin_surface.CreateIdAttr("UsdPreviewSurface")
        skin_surface.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(0.50, 0.34, 0.27)
        )
        skin_surface.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.56)
        skin_surface.CreateOutput("surface", Sdf.ValueTypeNames.Token)
        skin_material.CreateSurfaceOutput().ConnectToSource(
            skin_surface.ConnectableAPI(), "surface"
        )
        UsdShade.MaterialBindingAPI(head_mesh.GetPrim()).Bind(skin_material)

        photo_subset = UsdGeom.Subset.Define(stage, f"{root_path}/HeadMesh/PhotoFace")
        photo_subset.CreateElementTypeAttr().Set(UsdGeom.Tokens.face)
        photo_subset.CreateFamilyNameAttr().Set("materialBind")
        photo_subset.CreateIndicesAttr().Set(photo_faces)
        photo_material = UsdShade.Material.Define(stage, f"{root_path}/PhotoMaterial")
        photo_surface = UsdShade.Shader.Define(stage, f"{root_path}/PhotoMaterial/Surface")
        photo_surface.CreateIdAttr("UsdPreviewSurface")
        photo_surface.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.52)
        reader = UsdShade.Shader.Define(stage, f"{root_path}/PhotoMaterial/PrimvarReader")
        reader.CreateIdAttr("UsdPrimvarReader_float2")
        reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)
        texture = UsdShade.Shader.Define(stage, f"{root_path}/PhotoMaterial/Texture")
        texture.CreateIdAttr("UsdUVTexture")
        texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(
            Sdf.AssetPath(Path(texture_path).as_posix())
        )
        texture.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
        texture.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("clamp")
        texture.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("clamp")
        texture.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
            reader.ConnectableAPI(), "result"
        )
        texture.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
        photo_surface.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
            texture.ConnectableAPI(), "rgb"
        )
        photo_surface.CreateOutput("surface", Sdf.ValueTypeNames.Token)
        photo_material.CreateSurfaceOutput().ConnectToSource(
            photo_surface.ConnectableAPI(), "surface"
        )
        UsdShade.MaterialBindingAPI.Apply(photo_subset.GetPrim()).Bind(photo_material)

        # Hide only Debra's original skin-head polygons.  Hair remains visible.
        hidden_material = UsdShade.Material.Define(stage, f"{root_path}/HiddenOriginalHead")
        hidden_surface = UsdShade.Shader.Define(stage, f"{root_path}/HiddenOriginalHead/Surface")
        hidden_surface.CreateIdAttr("UsdPreviewSurface")
        hidden_surface.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(0.0, 0.0, 0.0)
        )
        hidden_surface.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.0)
        hidden_surface.CreateInput("opacityThreshold", Sdf.ValueTypeNames.Float).Set(0.1)
        hidden_surface.CreateOutput("surface", Sdf.ValueTypeNames.Token)
        hidden_material.CreateSurfaceOutput().ConnectToSource(
            hidden_surface.ConnectableAPI(), "surface"
        )
        UsdShade.MaterialBindingAPI.Apply(head_subset_prim).Bind(hidden_material)

        old_mask = stage.GetPrimAtPath(f"{debra_path}/GeneratedFace3D")
        if old_mask.IsValid():
            UsdGeom.Imageable(old_mask).CreateVisibilityAttr().Set(UsdGeom.Tokens.invisible)

        # The lab moves the Worker parent rather than playing facial/body
        # animation.  Freeze Debra's internal animation so cloned and original
        # rest-space geometry cannot diverge at runtime.
        skeleton_prim = next(
            (prim for prim in Usd.PrimRange(stage.GetPrimAtPath(debra_path))
             if prim.IsA(UsdSkel.Skeleton)),
            None,
        )
        if skeleton_prim is not None:
            animation_rel = skeleton_prim.GetRelationship("skel:animationSource")
            if animation_rel and animation_rel.IsValid():
                animation_rel.SetTargets([])

        self._face_trace(
            "50",
            f"GeneratedHead3D SUCCESS vertices={len(points)} faces={len(new_counts)} "
            f"photo_faces={len(photo_faces)} nose=({nose_x:.2f},{nose_y:.2f},{nose_z:.2f})",
        )

        # Persist representation version immediately so automatic restore uses
        # the same zero transform on the next launch.
        self._save_generated_face_placement(offset, yaw)

    def _create_generated_face_mesh_legacy(self, payload, texture_path):
        """Legacy 468-point overlay retained only for source compatibility."""
        stage = self._stage
        asset_path = f"{WORKER_PATH}/ModelVisuals/WorkerAsset"
        asset = stage.GetPrimAtPath(asset_path) if stage else None
        self._face_trace(
            "40",
            f"mesh entry: stage={stage is not None} asset_valid={bool(asset and asset.IsValid())}",
        )
        if asset is None or not asset.IsValid():
            raise RuntimeError("WorkerAsset is not available")

        debra_surface_points = self._debra_head_points_cm()
        x_min, y_min, z_min, x_max, y_max, z_max = self._debra_head_bounds_cm(
            debra_surface_points
        )
        head_width = max(x_max - x_min, 12.0)
        head_height = max(z_max - z_min, 28.0)
        fitted_width = min(head_width * 0.82, head_height * 0.52)
        self._face_trace(
            "41",
            f"Debra head bounds=({x_min:.3f},{y_min:.3f},{z_min:.3f})-"
            f"({x_max:.3f},{y_max:.3f},{z_max:.3f}) fitted_width={fitted_width:.3f}",
        )

        # Landmark 1 is MediaPipe's nose tip.  Anchor it to Debra's measured
        # nose-tip vertex instead of centering against the full skin-head
        # subset (which includes the scalp and previously lifted the mask into
        # the hair).  Values are measured from the shipped Debra rest mesh;
        # use proportional fallbacks only if a different character is loaded.
        nose_landmark = payload["points"][1]
        debra_nose_x = 0.1554 if head_width < 30.0 else (x_min + x_max) * 0.5
        debra_nose_y = y_min
        debra_nose_z = 164.9975 if 145.0 < z_min < 155.0 else z_min + head_height * 0.5625

        normalized = payload["points"]
        raw_depths = [float(point[1]) * fitted_width * 0.65 for point in normalized]
        # Reallusion Debra faces local -Y.  Match the most protruding point of
        # the generated face (normally the nose tip: minimum local Y) to
        # Debra's front-most head point.  The earlier max-depth alignment put
        # the entire reconstructed surface in front of the nose and made it
        # look like a floating mask.  Keep only a 0.8 mm clearance to prevent
        # z-fighting while the cheeks and jaw settle toward the head surface.
        surface_clearance_cm = 0.08
        nose_depth = float(nose_landmark[1]) * fitted_width * 0.65
        x_shift = debra_nose_x - float(nose_landmark[0]) * fitted_width
        z_shift = debra_nose_z - float(nose_landmark[2]) * fitted_width
        depth_shift = debra_nose_y - surface_clearance_cm - nose_depth
        provisional_points = [
            (
                x_shift + float(point[0]) * fitted_width,
                depth_shift + raw_depth,
                z_shift + float(point[2]) * fitted_width,
            )
            for point, raw_depth in zip(normalized, raw_depths)
        ]
        if debra_surface_points:
            # Conform every generated vertex to Debra's actual front surface.
            # A single nose-depth offset lets cheeks/forehead fall inside the
            # head when the two face shapes differ.  For each generated X/Z,
            # use the front-most nearby Debra vertices (local -Y) and retain a
            # small 1.2 mm clearance to prevent z-fighting.
            surface_radius_cm = max(0.75, fitted_width * 0.045)
            surface_radius_sq = surface_radius_cm * surface_radius_cm
            surface_clearance_cm = 0.12
            projected = []
            for px, _py, pz in provisional_points:
                nearby_y = [
                    float(source[1])
                    for source in debra_surface_points
                    if (
                        (float(source[0]) - px) ** 2
                        + (float(source[2]) - pz) ** 2
                    ) <= surface_radius_sq
                ]
                if not nearby_y:
                    nearest = min(
                        debra_surface_points,
                        key=lambda source: (
                            (float(source[0]) - px) ** 2
                            + (float(source[2]) - pz) ** 2
                        ),
                    )
                    nearby_y = [float(nearest[1])]
                front_values = sorted(nearby_y)[:4]
                surface_y = sum(front_values) / len(front_values)
                projected.append(
                    Gf.Vec3f(px, surface_y - surface_clearance_cm, pz)
                )
            points = projected
            self._face_trace(
                "42S",
                f"surface projection applied: vertices={len(points)} "
                f"radius={surface_radius_cm:.2f}cm clearance={surface_clearance_cm:.2f}cm",
            )
        else:
            points = [Gf.Vec3f(px, py, pz) for px, py, pz in provisional_points]
        faces = payload["faces"]
        face_counts = [3] * len(faces)
        face_indices = [int(index) for face in faces for index in face]
        texcoords = [Gf.Vec2f(float(uv[0]), float(uv[1])) for uv in payload["texcoords"]]
        self._face_trace(
            "42",
            f"geometry prepared: points={len(points)} triangles={len(faces)} texcoords={len(texcoords)}",
        )

        skel_root_path = f"{asset_path}/ManRoot/Debra"
        root_path = f"{skel_root_path}/GeneratedFace3D"
        # Do not remove/recreate the prim during automatic startup restore.
        # Hydra/Fabric may still be composing the referenced Debra hierarchy;
        # updating the stable path in place avoids removePath races.
        face_root = UsdGeom.Xform.Define(stage, root_path).GetPrim()
        placement_version = int(payload.get("placement_transform_version", 0) or 0)
        placement_offset = (
            payload.get("placement_offset_cm", [0.0, 0.0, -8.0])
            if placement_version >= 4
            else [0.0, 0.0, -8.0]
        )
        if not isinstance(placement_offset, (list, tuple)) or len(placement_offset) != 3:
            placement_offset = [0.0, 0.0, -8.0]
        placement_offset = [float(value) for value in placement_offset]
        # The generated vertices are already expressed in Debra's rest-space,
        # so zero yaw is the aligned/default pose.  The old +90 degree default
        # rotated the mask from the face centre toward the right ear.
        placement_yaw = (
            float(payload.get("placement_yaw_deg", 0.0))
            if placement_version >= 4
            else 0.0
        )
        face_pivot = Gf.Vec3d(
            float(points[1][0]), float(points[1][1]), float(points[1][2])
        )
        self._set_generated_face_transform(
            face_root, placement_offset, placement_yaw, pivot_cm=face_pivot
        )
        self._face_trace(
            "42A",
            "placement offset cm=" + ", ".join(f"{value:.1f}" for value in placement_offset)
            + f" yaw={placement_yaw:.1f} deg",
        )
        mesh = UsdGeom.Mesh.Define(stage, f"{root_path}/FaceMesh")
        mesh.CreatePointsAttr(points)
        mesh.CreateFaceVertexCountsAttr(face_counts)
        mesh.CreateFaceVertexIndicesAttr(face_indices)
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        mesh.CreateDoubleSidedAttr(True)
        st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex
        )
        st.Set(texcoords)

        # The generated points share the original body mesh's rest coordinate
        # system.  Copy the skin weights from each nearest Debra head vertex.
        # Binding the whole mask 100% to the Head joint rotated/translated the
        # mask differently from the actual face and could place it near an ear.
        # Matching the body's per-vertex (six-influence) binding makes both
        # surfaces follow exactly the same skeleton deformation.
        skeleton_path = f"{skel_root_path}/Debra"
        skeleton_prim = stage.GetPrimAtPath(skeleton_path)
        if not skeleton_prim.IsValid() or not skeleton_prim.IsA(UsdSkel.Skeleton):
            skeleton_prim = next(
                (
                    prim for prim in Usd.PrimRange(stage.GetPrimAtPath(skel_root_path))
                    if prim.IsA(UsdSkel.Skeleton)
                ),
                None,
            )
            if skeleton_prim is not None:
                skeleton_path = str(skeleton_prim.GetPath())
        self._face_trace(
            "43",
            f"skeleton candidate={skeleton_path} valid={bool(skeleton_prim and skeleton_prim.IsValid())}",
        )
        skeleton = UsdSkel.Skeleton(skeleton_prim) if skeleton_prim else None
        joints = skeleton.GetJointsAttr().Get() if skeleton else None
        if not joints:
            raise RuntimeError("Debra skeleton joints are unavailable")
        joint_names = [str(joint) for joint in joints]
        self._face_trace("44", "joints=" + ", ".join(joint_names))
        head_joint = next(
            (
                index
                for index, joint in enumerate(joint_names)
                if joint.replace("\\", "/").split("/")[-1].lower() in {
                    "head", "cc_base_head", "head_joint"
                }
                or joint.lower().endswith("head")
            ),
            None,
        )
        if head_joint is None:
            raise RuntimeError("Debra Head joint was not found")
        self._face_trace("45", f"Head joint index={head_joint} name={joint_names[head_joint]}")

        # Debra ships with an internal skeletal animation.  A Mesh authored at
        # runtime is not guaranteed to enter Hydra's already-populated UsdSkel
        # cache in the same frame, which leaves the generated mask at the bind
        # pose while the original head moves away.  This lab moves/rotates the
        # Worker through its parent Prim, so freeze the internal animation and
        # keep both surfaces in the same rest coordinate system.
        animation_rel = skeleton_prim.GetRelationship("skel:animationSource")
        previous_animation_targets = (
            [str(path) for path in animation_rel.GetTargets()]
            if animation_rel and animation_rel.IsValid()
            else []
        )
        if animation_rel and animation_rel.IsValid():
            animation_rel.SetTargets([])
        self._face_trace(
            "45C",
            "Debra internal animation frozen; previous targets="
            + (", ".join(previous_animation_targets) or "none"),
        )

        binding = UsdSkel.BindingAPI.Apply(mesh.GetPrim())
        binding.CreateSkeletonRel().SetTargets([Sdf.Path(skeleton_path)])
        binding.CreateGeomBindTransformAttr().Set(Gf.Matrix4d(1.0))
        skin_weights_copied = False
        try:
            body_path = (
                f"{asset_path}/ManRoot/Debra/CC_Base_Body/"
                "CC_Base_Body/CC_Base_Body"
            )
            body_prim = stage.GetPrimAtPath(body_path)
            body_mesh = UsdGeom.Mesh(body_prim)
            head_subset = UsdGeom.Subset(
                stage.GetPrimAtPath(f"{body_path}/Std_Skin_Head")
            )
            body_points = body_mesh.GetPointsAttr().Get()
            body_counts = body_mesh.GetFaceVertexCountsAttr().Get()
            body_indices = body_mesh.GetFaceVertexIndicesAttr().Get()
            head_faces = head_subset.GetIndicesAttr().Get()
            body_binding = UsdSkel.BindingAPI(body_prim)
            source_joint_indices = body_binding.GetJointIndicesPrimvar()
            source_joint_weights = body_binding.GetJointWeightsPrimvar()
            flat_joint_indices = source_joint_indices.Get()
            flat_joint_weights = source_joint_weights.Get()
            element_size = int(source_joint_indices.GetElementSize())
            if (
                body_points and body_counts and body_indices and head_faces
                and flat_joint_indices and flat_joint_weights and element_size > 0
            ):
                face_offsets = []
                offset = 0
                for count in body_counts:
                    face_offsets.append(offset)
                    offset += int(count)
                head_vertex_ids = set()
                for face_index in head_faces:
                    face_index = int(face_index)
                    begin = face_offsets[face_index]
                    count = int(body_counts[face_index])
                    head_vertex_ids.update(
                        int(index) for index in body_indices[begin:begin + count]
                    )
                head_vertex_ids = tuple(head_vertex_ids)
                mapped_joint_indices = []
                mapped_joint_weights = []
                for point in points:
                    nearest = min(
                        head_vertex_ids,
                        key=lambda vertex_id: (
                            (float(body_points[vertex_id][0]) - float(point[0])) ** 2
                            + (float(body_points[vertex_id][1]) - float(point[1])) ** 2
                            + (float(body_points[vertex_id][2]) - float(point[2])) ** 2
                        ),
                    )
                    begin = nearest * element_size
                    end = begin + element_size
                    mapped_joint_indices.extend(
                        int(value) for value in flat_joint_indices[begin:end]
                    )
                    mapped_joint_weights.extend(
                        float(value) for value in flat_joint_weights[begin:end]
                    )
                binding.CreateJointIndicesPrimvar(False, element_size).Set(
                    mapped_joint_indices
                )
                binding.CreateJointWeightsPrimvar(False, element_size).Set(
                    mapped_joint_weights
                )
                skin_weights_copied = True
                self._face_trace(
                    "45A",
                    f"copied Debra skin weights: vertices={len(points)} "
                    f"influences={element_size} source_head_vertices={len(head_vertex_ids)}",
                )
        except Exception as error:
            self._face_trace("45A-WARN", f"skin-weight copy failed: {error}")
        if not skin_weights_copied:
            binding.CreateJointIndicesPrimvar(True, 1).Set([int(head_joint)])
            binding.CreateJointWeightsPrimvar(True, 1).Set([1.0])
            self._face_trace("45B", "using fallback 100% Head-joint binding")

        material = UsdShade.Material.Define(stage, f"{root_path}/FaceMaterial")
        surface = UsdShade.Shader.Define(stage, f"{root_path}/FaceMaterial/Surface")
        surface.CreateIdAttr("UsdPreviewSurface")
        surface.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.52)
        surface.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)

        reader = UsdShade.Shader.Define(stage, f"{root_path}/FaceMaterial/PrimvarReader")
        reader.CreateIdAttr("UsdPrimvarReader_float2")
        reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)

        texture = UsdShade.Shader.Define(stage, f"{root_path}/FaceMaterial/Texture")
        texture.CreateIdAttr("UsdUVTexture")
        texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(
            Sdf.AssetPath(Path(texture_path).as_posix())
        )
        texture.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
        texture.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
            reader.ConnectableAPI(), "result"
        )
        texture.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
        surface.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
            texture.ConnectableAPI(), "rgb"
        )
        surface.CreateOutput("surface", Sdf.ValueTypeNames.Token)
        material.CreateSurfaceOutput().ConnectToSource(
            surface.ConnectableAPI(), "surface"
        )
        UsdShade.MaterialBindingAPI(mesh.GetPrim()).Bind(material)
        self._face_trace(
            "46",
            f"material bound; mesh_valid={mesh.GetPrim().IsValid()} texture={texture_path}",
        )

    def _generated_face_root_path(self):
        return (
            f"{WORKER_PATH}/ModelVisuals/WorkerAsset/ManRoot/Debra/GeneratedHead3D"
        )

    def _set_generated_face_transform(self, root, offset_cm, yaw_deg, pivot_cm=None):
        """Place the face and rotate it around its nose/face centre.

        Rotating the root around Debra's character origin makes a small yaw
        change orbit the mask into the hair or head.  A paired USD pivot op
        keeps the mask attached while allowing small alignment corrections.
        """
        self._author_clean_transform(
            root,
            position=(float(offset_cm[0]), float(offset_cm[1]), float(offset_cm[2])),
            scale=(1.0, 1.0, 1.0),
        )
        pivot_attr = root.GetAttribute("lab:facePivotCm")
        if pivot_cm is None and pivot_attr and pivot_attr.HasAuthoredValueOpinion():
            pivot_cm = pivot_attr.Get()
        if pivot_cm is None:
            # This fallback is used only for a face Prim created by an older
            # build before the next automatic restore authors the exact pivot.
            pivot_cm = Gf.Vec3d(0.1554, 0.0, 164.9975)
        pivot_cm = Gf.Vec3d(
            float(pivot_cm[0]), float(pivot_cm[1]), float(pivot_cm[2])
        )
        pivot = root.CreateAttribute(
            "xformOp:translate:facePivot", Sdf.ValueTypeNames.Double3, custom=False
        )
        pivot.Set(pivot_cm)
        root.GetAttribute("xformOpOrder").Set([
            "xformOp:translate",
            "xformOp:translate:facePivot",
            "xformOp:orient",
            "xformOp:scale",
            "!invert!xformOp:translate:facePivot",
        ])
        half_angle = math.radians(float(yaw_deg)) * 0.5
        root.GetAttribute("xformOp:orient").Set(
            Gf.Quatf(
                math.cos(half_angle),
                Gf.Vec3f(0.0, 0.0, math.sin(half_angle)),
            )
        )
        root.CreateAttribute("lab:faceOffsetCm", Sdf.ValueTypeNames.Double3).Set(
            Gf.Vec3d(float(offset_cm[0]), float(offset_cm[1]), float(offset_cm[2]))
        )
        root.CreateAttribute("lab:faceYawDeg", Sdf.ValueTypeNames.Double).Set(
            float(yaw_deg)
        )
        root.CreateAttribute("lab:facePivotCm", Sdf.ValueTypeNames.Double3).Set(
            pivot_cm
        )

    def _adjust_generated_face(self, dx_cm, dy_cm, dz_cm):
        """Move the generated mask in one-centimetre character-local steps."""
        if self._stage is None:
            self._set_status("USD Stage가 없습니다.")
            return
        root = self._stage.GetPrimAtPath(self._generated_face_root_path())
        if not root.IsValid():
            self._set_status("먼저 Generate 3D Head를 실행하세요.")
            return
        attr = root.GetAttribute("lab:faceOffsetCm")
        current = attr.Get() if attr and attr.HasAuthoredValueOpinion() else None
        if current is None:
            current = Gf.Vec3d(0.0, 0.0, 0.0)
        yaw_attr = root.GetAttribute("lab:faceYawDeg")
        yaw_deg = (
            float(yaw_attr.Get())
            if yaw_attr and yaw_attr.HasAuthoredValueOpinion()
            else 0.0
        )
        updated = Gf.Vec3d(
            float(current[0]) + float(dx_cm),
            float(current[1]) + float(dy_cm),
            float(current[2]) + float(dz_cm),
        )
        self._set_generated_face_transform(root, updated, yaw_deg)
        self._save_generated_face_placement(updated, yaw_deg)
        self._set_status(
            "3D 얼굴 위치(cm): "
            f"좌우 {updated[0]:+.1f}, 깊이 {updated[1]:+.1f}, "
            f"높이 {updated[2]:+.1f}, Yaw {yaw_deg:+.1f}°"
        )
        self._face_trace(
            "47",
            f"manual placement offset cm=({updated[0]:.1f}, {updated[1]:.1f}, {updated[2]:.1f})",
        )

    def _rotate_generated_face(self, delta_yaw_deg):
        if self._stage is None:
            self._set_status("USD Stage가 없습니다.")
            return
        root = self._stage.GetPrimAtPath(self._generated_face_root_path())
        if not root.IsValid():
            self._set_status("먼저 Generate 3D Head를 실행하세요.")
            return
        offset_attr = root.GetAttribute("lab:faceOffsetCm")
        offset = (
            offset_attr.Get()
            if offset_attr and offset_attr.HasAuthoredValueOpinion()
            else Gf.Vec3d(0.0, 0.0, 0.0)
        )
        yaw_attr = root.GetAttribute("lab:faceYawDeg")
        current_yaw = (
            float(yaw_attr.Get())
            if yaw_attr and yaw_attr.HasAuthoredValueOpinion()
            else 0.0
        )
        updated_yaw = current_yaw + float(delta_yaw_deg)
        self._set_generated_face_transform(root, offset, updated_yaw)
        self._save_generated_face_placement(offset, updated_yaw)
        self._set_status(f"3D 얼굴 Yaw 회전: {updated_yaw:+.1f}°")
        self._face_trace("48", f"manual face yaw={updated_yaw:.1f} deg")

    def _reset_generated_face_position(self):
        root = self._stage.GetPrimAtPath(self._generated_face_root_path()) if self._stage else None
        if root is None or not root.IsValid():
            self._set_status("먼저 Generate 3D Head를 실행하세요.")
            return
        default = Gf.Vec3d(0.0, 0.0, 0.0)
        default_yaw = 0.0
        self._set_generated_face_transform(root, default, default_yaw)
        self._save_generated_face_placement(default, default_yaw)
        self._set_status("3D 머리 위치를 Debra 목 경계 기본값(0cm, Yaw 0°)으로 복원했습니다.")
        self._face_trace("47", "head placement reset to (0.0, 0.0, 0.0) cm, yaw=0 deg")

    def _save_generated_face_placement(self, offset, yaw_deg):
        mesh_json = (
            self._extension_root()
            / "assets" / "characters" / "Worker_Custom_Face_Prim"
            / "Worker_Custom" / "GeneratedFace" / "face_mesh.json"
        )
        try:
            if not mesh_json.is_file():
                return
            payload = json.loads(mesh_json.read_text(encoding="utf-8"))
            payload["placement_offset_cm"] = [float(value) for value in offset]
            payload["placement_yaw_deg"] = float(yaw_deg)
            payload["placement_transform_version"] = 5
            mesh_json.write_text(
                json.dumps(payload, ensure_ascii=False, separators=(",", ":")),
                encoding="utf-8",
            )
        except Exception as error:
            carb.log_warn(f"[Worker Face3D] offset save skipped: {error}")

    def _restore_generated_face_if_available(self):
        # 2D-only build: automatic startup geometry restoration is disabled.
        return False
        
        output_root = (
            self._extension_root()
            / "assets"
            / "characters"
            / "Worker_Custom_Face_Prim"
            / "Worker_Custom"
            / "GeneratedFace"
        )
        mesh_json = output_root / "face_mesh.json"
        if not mesh_json.is_file():
            return
        try:
            payload = json.loads(mesh_json.read_text(encoding="utf-8"))
            texture = Path(payload.get("texture", ""))
            if not texture.is_file():
                texture = output_root / texture.name
            if texture.is_file():
                # A new file path for every session prevents Hydra from showing
                # a stale or partially decoded startup texture.
                runtime_dir = output_root / "Runtime"
                runtime_dir.mkdir(parents=True, exist_ok=True)
                runtime_texture = runtime_dir / f"face_restore_{time.time_ns()}{texture.suffix or '.jpg'}"
                shutil.copy2(texture, runtime_texture)
                self._create_generated_face_mesh(payload, runtime_texture)
                return True
        except Exception as error:
            carb.log_warn(f"[Worker Face3D] Restore skipped: {error}")
        return False

    def _schedule_generated_face_restore(self):
        """Compatibility no-op: this release supports 2D texture mapping only."""
        return

    async def _restore_generated_face_when_ready(self):
        try:
            app = omni.kit.app.get_app()
            # Referenced character payloads can compose a few frames after the
            # Worker parent is authored.  Waiting for real head bounds avoids
            # fitting the restored mask with fallback dimensions.
            ready_frames = 0
            for _ in range(180):
                await app.next_update_async()
                if self._is_shutting_down or self._stage is None:
                    return
                if self._worker_face_asset_ready():
                    ready_frames += 1
            # Referenced head geometry can report valid attributes before
            # its composed points have settled.  Require a stable
            # half-second-equivalent window before fitting the
                    # saved mask; this matches the reliable manual regenerate
                    # path used after the scene has finished loading.
                    if ready_frames >= 30:
                        if self._restore_generated_face_if_available():
                            carb.log_info(
                                "[Worker Face3D] restored after 30 stable Debra frames"
                            )
                        return
                else:
                    ready_frames = 0
            carb.log_warn("[Worker Face3D] startup restore timed out waiting for Debra")
        except asyncio.CancelledError:
            raise
        except Exception as error:
            carb.log_warn(f"[Worker Face3D] delayed restore skipped: {error}")

    def _worker_face_asset_ready(self):
        body_path = (
            f"{WORKER_PATH}/ModelVisuals/WorkerAsset/ManRoot/Debra/"
            "CC_Base_Body/CC_Base_Body/CC_Base_Body"
        )
        body_prim = self._stage.GetPrimAtPath(body_path)
        subset_prim = self._stage.GetPrimAtPath(f"{body_path}/Std_Skin_Head")
        if not body_prim.IsValid() or not subset_prim.IsValid():
            return False
        points = UsdGeom.Mesh(body_prim).GetPointsAttr().Get()
        head_faces = UsdGeom.Subset(subset_prim).GetIndicesAttr().Get()
        return bool(points and head_faces)

    def _debra_head_points_cm(self):
        """Return rest-pose vertices used by Debra's skin-head subset."""
        body_path = (
            f"{WORKER_PATH}/ModelVisuals/WorkerAsset/ManRoot/Debra/"
            "CC_Base_Body/CC_Base_Body/CC_Base_Body"
        )
        body = UsdGeom.Mesh(self._stage.GetPrimAtPath(body_path))
        subset = UsdGeom.Subset(
            self._stage.GetPrimAtPath(f"{body_path}/Std_Skin_Head")
        )
        points = body.GetPointsAttr().Get()
        counts = body.GetFaceVertexCountsAttr().Get()
        indices = body.GetFaceVertexIndicesAttr().Get()
        selected_faces = subset.GetIndicesAttr().Get()
        if not points or not counts or not indices or not selected_faces:
            self._face_trace(
                "41-FALLBACK",
                f"head geometry incomplete: points={bool(points)} counts={bool(counts)} "
                f"indices={bool(indices)} selected_faces={bool(selected_faces)}",
            )
            return []

        offsets = [0]
        for count in counts:
            offsets.append(offsets[-1] + int(count))
        vertex_ids = set()
        for face_index in selected_faces:
            face_index = int(face_index)
            if 0 <= face_index < len(counts):
                vertex_ids.update(
                    int(value)
                    for value in indices[offsets[face_index]:offsets[face_index + 1]]
                )
        selected_points = [points[index] for index in vertex_ids if index < len(points)]
        if not selected_points:
            self._face_trace("41-FALLBACK", "head subset produced no selected vertices")
        return selected_points

    def _debra_head_bounds_cm(self, selected_points=None):
        """Return the rest-pose bounds of Debra's Std_Skin_Head GeomSubset."""
        if selected_points is None:
            selected_points = self._debra_head_points_cm()
        if not selected_points:
            return (-10.0, -7.0, 132.0, 10.0, 7.0, 180.0)
        return (
            min(float(point[0]) for point in selected_points),
            min(float(point[1]) for point in selected_points),
            min(float(point[2]) for point in selected_points),
            max(float(point[0]) for point in selected_points),
            max(float(point[1]) for point in selected_points),
            max(float(point[2]) for point in selected_points),
        )

    def _start_worker_motion(self):
        if is_ai_graduate_room({'kind':self._room_kind,'usd_path':self._room_source}):
            self._set_status('AI대학원동은 AGV 주행 전용입니다. 습도 미션·자동체험·Worker는 다른 실습 공간에서 사용하세요.')
            return
        worker = self._stage.GetPrimAtPath(WORKER_PATH) if self._stage else None
        if worker is None or not worker.IsValid():
            self._set_status("Worker가 없습니다. Lab 1-B를 먼저 실행하세요.")
            return
        self._worker_auto_active = True
        self._timeline.play()
        self._set_status(
            "Worker AUTO WALK 시작\n"
            f"Y={self.WORKER_MIN_Y:.1f}~{self.WORKER_MAX_Y:.1f} m 왕복 · "
            f"속도={self.WORKER_AUTO_SPEED:.2f} m/s"
        )

    def _orient_worker_toward_active_camera(self):
        """Turn Debra's visual front (-Y) toward the current main camera."""
        fallback_yaw = 180.0
        try:
            if self._stage is None:
                raise RuntimeError("USD Stage is unavailable")
            viewport = get_active_viewport()
            camera_path = str(getattr(viewport, "camera_path", "") or "")
            camera = self._stage.GetPrimAtPath(camera_path)
            worker = self._stage.GetPrimAtPath(WORKER_PATH)
            asset = self._stage.GetPrimAtPath(f"{WORKER_PATH}/ModelVisuals/WorkerAsset")
            if not camera.IsValid() or not worker.IsValid() or not asset.IsValid():
                raise RuntimeError(f"camera or Worker is invalid: {camera_path}")

            cache = UsdGeom.XformCache(Usd.TimeCode.Default())
            camera_position = cache.GetLocalToWorldTransform(camera).ExtractTranslation()
            worker_position = cache.GetLocalToWorldTransform(worker).ExtractTranslation()
            dx = float(camera_position[0] - worker_position[0])
            dy = float(camera_position[1] - worker_position[1])
            if math.hypot(dx, dy) < 0.05:
                raise RuntimeError("camera is directly above the Worker")

            # Debra's authored forward vector is local -Y.  After a Z yaw,
            # Rz(yaw) * (0,-1) = (sin(yaw),-cos(yaw)).
            yaw_deg = math.degrees(math.atan2(dx, -dy))
            orient = self._find_xform_op(
                UsdGeom.Xformable(asset), UsdGeom.XformOp.TypeOrient
            )
            if orient is None:
                raise RuntimeError("Worker visual orientation op is unavailable")
            half_angle = math.radians(yaw_deg) * 0.5
            orient.Set(
                Gf.Quatf(
                    math.cos(half_angle),
                    Gf.Vec3f(0.0, 0.0, math.sin(half_angle)),
                )
            )
            self._worker_visual_yaw_deg = yaw_deg % 360.0
            carb.log_info(
                f"[Worker] initial front aligned to camera: yaw={self._worker_visual_yaw_deg:.1f} deg"
            )
            return True
        except Exception as error:
            asset = (
                self._stage.GetPrimAtPath(f"{WORKER_PATH}/ModelVisuals/WorkerAsset")
                if self._stage else None
            )
            if asset is not None and asset.IsValid():
                orient = self._find_xform_op(
                    UsdGeom.Xformable(asset), UsdGeom.XformOp.TypeOrient
                )
                if orient is not None:
                    orient.Set(Gf.Quatf(0.0, Gf.Vec3f(0.0, 0.0, 1.0)))
            self._worker_visual_yaw_deg = fallback_yaw
            carb.log_warn(f"[Worker] camera-front alignment fallback: {error}")
            return False

    def _rotate_worker_45(self):
        """Rotate only the Worker visual by 45 degrees per button press."""
        if self._stage is None:
            self._set_status("USD Stage가 없습니다. Lab 1-B를 먼저 실행하세요.")
            return
        asset_path = f"{WORKER_PATH}/ModelVisuals/WorkerAsset"
        asset = self._stage.GetPrimAtPath(asset_path)
        if not asset.IsValid():
            self._set_status("Worker가 없습니다. Lab 1-B를 먼저 실행하세요.")
            return
        orient = self._find_xform_op(
            UsdGeom.Xformable(asset), UsdGeom.XformOp.TypeOrient
        )
        if orient is None:
            self._set_status("Worker 회전 속성을 찾을 수 없습니다.")
            return
        self._worker_visual_yaw_deg = (self._worker_visual_yaw_deg + 45.0) % 360.0
        half_angle = math.radians(self._worker_visual_yaw_deg) * 0.5
        orient.Set(
            Gf.Quatf(
                math.cos(half_angle),
                Gf.Vec3f(0.0, 0.0, math.sin(half_angle)),
            )
        )
        self._set_status(
            f"Worker를 45° 회전했습니다. 현재 Z 회전: "
            f"{self._worker_visual_yaw_deg:.0f}°"
        )

    def _stop_worker_motion(self):
        self._worker_auto_active = False
        self._set_status("Worker 이동을 정지했습니다.")

    def _scale_worker(self, factor):
        """Uniformly scale the Worker root, including face and capsule."""
        worker = self._stage.GetPrimAtPath(WORKER_PATH) if self._stage else None
        if worker is None or not worker.IsValid():
            self._set_status("Worker가 없습니다. Lab 1-B를 먼저 실행하세요.")
            return
        scale_op = self._find_xform_op(
            UsdGeom.Xformable(worker), UsdGeom.XformOp.TypeScale
        )
        if scale_op is None:
            self._set_status("Worker 크기 속성을 찾을 수 없습니다.")
            return
        current = scale_op.Get()
        current_scale = float(current[0]) if current is not None else 1.0
        new_scale = max(0.5, min(2.0, current_scale * float(factor)))
        scale_op.Set(Gf.Vec3f(new_scale, new_scale, new_scale))
        worker.CreateAttribute("lab:workerScale", Sdf.ValueTypeNames.Float).Set(new_scale)
        self._set_status(
            f"Worker 크기를 {new_scale:.2f}배로 변경했습니다. "
            "얼굴·몸체·충돌체가 함께 변경됩니다."
        )

    def _reset_worker_motion(self):
        self._worker_auto_active = False
        self._worker_direction = 1.0
        worker = self._stage.GetPrimAtPath(WORKER_PATH) if self._stage else None
        if worker is None or not worker.IsValid():
            self._set_status("Worker가 없습니다. Lab 1-B를 먼저 실행하세요.")
            return
        translate = self._find_xform_op(
            UsdGeom.Xformable(worker), UsdGeom.XformOp.TypeTranslate
        )
        if translate is not None:
            translate.Set(
                Gf.Vec3d(self.WORKER_START_X, self.WORKER_START_Y, 0.0)
            )
        scale_op = self._find_xform_op(
            UsdGeom.Xformable(worker), UsdGeom.XformOp.TypeScale
        )
        if scale_op is not None:
            scale_op.Set(Gf.Vec3f(1.0, 1.0, 1.0))
        worker.CreateAttribute("lab:workerScale", Sdf.ValueTypeNames.Float).Set(1.0)
        asset = self._stage.GetPrimAtPath(f"{WORKER_PATH}/ModelVisuals/WorkerAsset")
        if asset.IsValid():
            orient = self._find_xform_op(
                UsdGeom.Xformable(asset), UsdGeom.XformOp.TypeOrient
            )
            if orient is not None:
                orient.Set(Gf.Quatf(0.0, Gf.Vec3f(0.0, 0.0, 1.0)))
        self._worker_visual_yaw_deg = 180.0
        self._orient_worker_toward_active_camera()
        self._set_status(
            "Worker를 AGV 온도미션 통로 밖의 기본 위치 "
            f"({self.WORKER_START_X:.1f}, {self.WORKER_START_Y:.1f}, 0.0), "
            "크기 1.0배·정면 방향으로 복원했습니다."
        )

    def _update_worker_motion(self, dt):
        if not self._worker_auto_active or self._stage is None:
            return
        worker = self._stage.GetPrimAtPath(WORKER_PATH)
        if not worker.IsValid():
            self._worker_auto_active = False
            return
        translate = self._find_xform_op(
            UsdGeom.Xformable(worker), UsdGeom.XformOp.TypeTranslate
        )
        if translate is None:
            self._worker_auto_active = False
            return
        position = translate.Get()
        next_y = float(position[1]) + (
            self.WORKER_AUTO_SPEED * self._worker_direction * float(dt)
        )
        if next_y >= self.WORKER_MAX_Y:
            next_y = self.WORKER_MAX_Y
            self._worker_direction = -1.0
        elif next_y <= self.WORKER_MIN_Y:
            next_y = self.WORKER_MIN_Y
            self._worker_direction = 1.0
        translate.Set(Gf.Vec3d(float(position[0]), next_y, float(position[2])))

    def _start_learned_service_manager(self):
        if os.name == "nt":
            carb.log_info("Linux NoMaD 서비스는 Windows에서 자동 시작하지 않습니다.")
            return
        project = self._extension_root().parents[1]
        runtime = project / ".learned_runtime"
        python = runtime / "env/bin/python"
        if not python.is_file():
            carb.log_info("학습 모델 미설치: 자동 실행을 건너뜁니다.")
            return
        try:
            logdir = runtime / "logs"
            logdir.mkdir(parents=True, exist_ok=True)
            env = dict(os.environ, PYTHONUNBUFFERED="1")
            for key in ("PYTHONPATH", "PYTHONHOME", "LD_LIBRARY_PATH"):
                env.pop(key, None)
            with (logdir / "manager.log").open("a") as log:
                self._learned_service_manager = subprocess.Popen(
                    [str(python), str(project / "tools/navigation/manage_learned_service.py"),
                     "--owner-pid", str(os.getpid())], cwd=project, env=env,
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            carb.log_info("학습 모델 서비스 자동 시작 · Isaac 종료 시 자동 종료")
        except Exception as error:
            carb.log_warn(f"학습 모델 자동 시작 실패: {error}")

    def _start_nav2_service_manager(self):
        if os.name == "nt":
            carb.log_info("Linux Nav2 서비스는 Windows에서 자동 시작하지 않습니다.")
            return
        project = self._extension_root().parents[1]
        runtime = project / ".nav2_runtime"
        python = runtime / "env/bin/python"
        if not python.is_file():
            carb.log_info("Nav2 미설치: 자동 실행을 건너뜁니다.")
            return
        try:
            logdir = runtime / "logs"
            logdir.mkdir(parents=True, exist_ok=True)
            env = dict(os.environ, PYTHONUNBUFFERED="1")
            for key in ("PYTHONPATH", "PYTHONHOME", "LD_LIBRARY_PATH"):
                env.pop(key, None)
            with (logdir / "manager.log").open("a") as log:
                self._nav2_service_manager = subprocess.Popen(
                    [str(python), str(project / "tools/navigation/manage_nav2_service.py"),
                     "--owner-pid", str(os.getpid())], cwd=project, env=env,
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            carb.log_info("Nav2 서비스 자동 시작 · Isaac 종료 시 자동 종료")
        except Exception as error:
            carb.log_warn(f"Nav2 자동 시작 실패: {error}")

    def _set_agv_body_red(self):
        body = self._stage.GetPrimAtPath(f"{ROBOT_PATH}/ModelVisuals/Body") if self._stage else None
        if body and body.IsValid():
            UsdGeom.Gprim(body).CreateDisplayColorAttr([Gf.Vec3f(0.85, 0.08, 0.06)])
            self._set_status("AGV 차체를 빨간색으로 변경했습니다.")

    def _apply_default_agv_size(self, robot):
        """Migrate old lab AGVs once; snapshots retain their size version."""
        if robot.GetAttribute("lab:robotModel").Get() != "Educational PhysX AGV":
            return
        if robot.GetAttribute("lab:agvSizeVersion").Get() == 1:
            return
        for name in ("xformOp:scale", "lab:initialScale"):
            attr = robot.GetAttribute(name)
            value = attr.Get()
            if value is not None:
                attr.Set(value * self.AGV_SIZE_SCALE)
        for name in ("xformOp:translate", "lab:initialPosition"):
            attr = robot.GetAttribute(name)
            value = attr.Get()
            if value is not None:
                value[2] *= self.AGV_SIZE_SCALE
                attr.Set(value)
        robot.CreateAttribute("lab:agvSizeVersion", Sdf.ValueTypeNames.Int).Set(1)

    async def _resize_existing_agv(self):
        self._stop_robot()
        robot = self._stage.GetPrimAtPath(ROBOT_PATH)
        if not robot.IsValid():
            return
        pose = {name: robot.GetAttribute(name).Get() for name in ("xformOp:translate", "xformOp:orient")}
        self._timeline.stop()
        self._dynamic_robot_handle = _dynamic_control.INVALID_HANDLE
        app = omni.kit.app.get_app()
        await app.next_update_async()
        await app.next_update_async()
        for name, value in pose.items():
            if value is not None: robot.GetAttribute(name).Set(value)
        self._apply_default_agv_size(robot)
        self._ensure_robot_camera_stream()
        self._narrate("done", "AGV 크기를 기존의 70%로 적용했습니다. 가로·세로·높이와 충돌 형상이 함께 축소됐습니다.")

    async def _install_agv_async(self):
        """Create a standalone dynamic AGV with separate collision and visuals."""
        self._timeline.stop()
        self._dynamic_robot_handle = _dynamic_control.INVALID_HANDLE
        app = omni.kit.app.get_app()
        stage = self._stage
        # Finish Stop's physics/USD restoration before authoring a new body.
        await app.next_update_async()
        await app.next_update_async()
        if stage is None or self._stage != stage or not self._room_ready():
            raise RuntimeError("AGV 설치 중 공간이 변경됐습니다. 창고를 완성한 뒤 C를 다시 누르세요.")
        # Saved/reference layers may still contribute the old robot after
        # RemovePrim. Author the repaired body in the strongest, private layer;
        # never delete or modify the referenced scene file.
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            await self._author_agv_async(stage, app)

    @staticmethod
    async def _wait_frames(app, count=18):
        """Keep each installation step visible while staying on Kit's update loop."""
        for _ in range(count):
            await app.next_update_async()

    def _ensure_physics_scene(self):
        """Create/repair the persistent USD+PhysX scene used by Isaac APIs."""
        stage = self._stage
        if stage is None:
            return False
        scene = UsdPhysics.Scene.Get(stage, PHYSICS_PATH)
        if not scene or not scene.GetPrim().IsValid():
            scene = UsdPhysics.Scene.Define(stage, PHYSICS_PATH)
        prim = scene.GetPrim()
        if not prim.IsValid():
            return False
        scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr(9.81)
        physx_scene = PhysxSchema.PhysxSceneAPI.Apply(prim)
        time_steps = physx_scene.GetTimeStepsPerSecondAttr()
        if not time_steps.IsValid():
            time_steps = physx_scene.CreateTimeStepsPerSecondAttr()
        value = time_steps.Get()
        if value is None or float(value) <= 0.0:
            time_steps.Set(60)
        carb.log_info(
            f"[Physics Scene] valid={prim.IsValid()} path={PHYSICS_PATH} "
            f"timeStepsPerSecond={time_steps.Get()}"
        )
        return bool(prim.IsValid() and physx_scene)

    def _play_simulation(self):
        if not self._ensure_physics_scene():
            self._set_status("PhysicsScene is invalid. Rebuild Lab 1-B before Play.")
            return False
        # Browser drive heartbeats arrive several times per second. Calling
        # play() again while already running can retrigger timeline/physics
        # state transitions on some Kit builds, so start it only once.
        if not self._timeline.is_playing():
            self._dynamic_robot_handle = _dynamic_control.INVALID_HANDLE
            self._timeline.play()
        return True

    def _agv_is_installed(self):
        robot = self._stage.GetPrimAtPath(ROBOT_PATH) if self._stage else None
        if not (robot and robot.IsValid() and robot.IsActive() and robot.GetAttribute("lab:isRobot").Get()
                and UsdPhysics.RigidBodyAPI(robot)):
            return False
        complete = robot.GetAttribute("lab:agvInstallComplete")
        if complete.IsValid() and not complete.Get():
            return False
        # Old saved scenes have no completion marker; verify actual geometry.
        required = ("CollisionShape", "ModelVisuals/Body", "ModelVisuals/Top",
                    "ModelVisuals/FrontBumper", "ModelVisuals/WheelFL",
                    "ModelVisuals/WheelFR", "ModelVisuals/WheelRL", "ModelVisuals/WheelRR")
        if not all((prim := self._stage.GetPrimAtPath(f"{ROBOT_PATH}/{name}"))
                   and prim.IsA(UsdGeom.Gprim) for name in required):
            return False
        camera = self._stage.GetPrimAtPath(f"{ROBOT_PATH}/LabFrontCamera")
        return bool(camera and camera.IsA(UsdGeom.Camera))

    def _robot_motion_prim(self):
        """Return the standalone AGV root used for pose and velocity control."""
        if self._stage is None:
            return None
        root = self._stage.GetPrimAtPath(ROBOT_PATH)
        if not root.IsValid() or not UsdPhysics.RigidBodyAPI(root):
            return None
        self._robot_motion_prim_path = ROBOT_PATH
        return root

    def _mobile_base_drives(self):
        """Discover Ridgeback's planar dummy joints without trusting USD axis alone.

        In the Isaac Sim 4.5 asset, a joint's local axis can remain X while its
        local joint frame supplies the Y/Z rotation.  The official dummy-joint
        names are therefore the primary identity; type/axis is only a fallback.
        """
        stage = self._stage
        if stage is None:
            return None
        if self._mobile_base_drive_paths:
            cached = {}
            for key, path in self._mobile_base_drive_paths.items():
                prim = stage.GetPrimAtPath(path)
                if not prim.IsValid():
                    return None
                cached[key] = prim
            return cached
        robot = stage.GetPrimAtPath(ROBOT_PATH)
        if not robot.IsValid():
            return None
        found = {}
        fallback = {}
        candidates = []
        for prim in Usd.PrimRange(robot):
            lower = str(prim.GetPath()).lower()
            if "panda" in lower or "franka" in lower or "finger" in lower:
                continue
            name = prim.GetName().lower()
            if prim.IsA(UsdPhysics.PrismaticJoint):
                axis = str(UsdPhysics.PrismaticJoint(prim).GetAxisAttr().Get()).upper()
                candidates.append((str(prim.GetPath()), "prismatic", axis))
                if "prismatic_x" in name or "prismatic_x" in lower:
                    found["x"] = prim
                elif "prismatic_y" in name or "prismatic_y" in lower:
                    found["y"] = prim
                elif axis == "X" and "x" not in fallback:
                    fallback["x"] = prim
                elif axis == "Y" and "y" not in fallback:
                    fallback["y"] = prim
            elif prim.IsA(UsdPhysics.RevoluteJoint):
                axis = str(UsdPhysics.RevoluteJoint(prim).GetAxisAttr().Get()).upper()
                candidates.append((str(prim.GetPath()), "revolute", axis))
                if ("revolute_z" in name or "revolute_z" in lower or
                        "dummy_base_yaw" in name):
                    found["yaw"] = prim
                elif axis == "Z" and "yaw" not in fallback:
                    fallback["yaw"] = prim
        for key in ("x", "y", "yaw"):
            if key not in found and key in fallback:
                found[key] = fallback[key]
        if set(found) != {"x", "y", "yaw"}:
            signature = tuple(candidates)
            if signature != self._mobile_base_drive_error_signature:
                self._mobile_base_drive_error_signature = signature
                carb.log_error(
                    f"[Mobile Base Drives] expected X/Y/Yaw, found="
                    f"{ {key: str(prim.GetPath()) for key, prim in found.items()} }; "
                    f"candidates={candidates}"
                )
            return None
        self._mobile_base_drive_error_signature = None
        self._mobile_base_drive_paths = {
            key: str(prim.GetPath()) for key, prim in found.items()
        }
        for key, prim in found.items():
            mode = UsdPhysics.Tokens.angular if key == "yaw" else UsdPhysics.Tokens.linear
            drive = UsdPhysics.DriveAPI.Get(prim, mode)
            if not drive:
                drive = UsdPhysics.DriveAPI.Apply(prim, mode)
            drive.CreateStiffnessAttr().Set(0.0)
            damping = drive.GetDampingAttr().Get()
            if damping is None or float(damping) <= 0.0:
                drive.CreateDampingAttr().Set(10000.0)
            drive.CreateTargetVelocityAttr().Set(0.0)
        carb.log_info(f"[Mobile Base Drives] {self._mobile_base_drive_paths}")
        return found

    def _robot_xy(self):
        motion_prim = self._robot_motion_prim()
        return self._prim_xy(motion_prim) if motion_prim is not None else None

    def _install_robot_camera(self, motion_prim):
        """Attach a forward perspective camera to the physical AGV base.

        USD cameras look along local -Z with local +Y as image-up.  The authored
        quaternion maps those axes to AGV +X (forward) and +Z (up), so the
        image stays upright while the physical base translates and rotates.
        """
        camera_path = f"{motion_prim.GetPath()}/LabFrontCamera"
        camera = UsdGeom.Camera.Define(self._stage, camera_path)
        prim = camera.GetPrim()
        # Compensate the robot's 0.7 scale: eye is 1.5 m above its support floor.
        matrix=UsdGeom.XformCache().GetLocalToWorldTransform(motion_prim)
        scale_z=matrix.TransformDir(Gf.Vec3d(0,0,1)).GetLength()
        floor_z=motion_prim.GetAttribute('lab:spawnFloorZ').Get()
        base_height=matrix.ExtractTranslation()[2]-floor_z if floor_z is not None else ROBOT_BASE_HEIGHT
        self._author_clean_transform(
            prim,
            position=(0.70, 0.0, camera_local_height(base_height, scale_z)),
            scale=(1.0, 1.0, 1.0),
        )
        prim.GetAttribute("xformOp:orient").Set(
            Gf.Quatf(0.5, Gf.Vec3f(0.5, -0.5, -0.5))
        )
        camera.CreateProjectionAttr(UsdGeom.Tokens.perspective)
        camera.CreateFocalLengthAttr(18.0)
        camera.CreateHorizontalApertureAttr(20.955)
        camera.CreateVerticalApertureAttr(15.2908)
        camera.CreateClippingRangeAttr(Gf.Vec2f(0.05, 1000.0))
        prim.CreateAttribute("lab:isRobotCamera", Sdf.ValueTypeNames.Bool).Set(True)
        self._robot_camera_path = camera_path
        self._start_camera_stream()
        carb.log_info(f"[Robot Camera] installed: {camera_path}")
        return prim

    def _start_camera_stream(self):
        if self._camera_stream_task is not None and not self._camera_stream_task.done():
            return
        frame_dir = self._extension_root() / "assets" / "web_control" / "camera"
        if os.name == "nt":
            # Kit's native image writer rejects some Unicode Windows paths.
            # Keep rendered JPEGs in a private, process-selected cache; Linux
            # continues using the original extension-relative camera folder.
            frame_dir = Path(os.environ.get("LAB_NATIVE_CAPTURE_DIR", str(
                Path(os.environ["LOCALAPPDATA"]) / "IsaacInteractivePhysicsLab" / "captures"))) / "agv"
        frame_dir.mkdir(parents=True, exist_ok=True)
        self._camera_frame_path = frame_dir / "latest.jpg"
        if self._external_control_server is not None:
            self._external_control_server.set_camera_frame_path(self._camera_frame_path)
        self._camera_stream_task = asyncio.ensure_future(self._run_camera_stream())

    def _ensure_robot_camera_stream(self):
        """Recover the existing AGV camera after extension reloads."""
        robot = self._robot_motion_prim()
        if robot is None:
            return False
        path = f"{robot.GetPath()}/LabFrontCamera"
        camera = self._stage.GetPrimAtPath(path)
        if camera.IsValid() and camera.IsA(UsdGeom.Camera):
            self._robot_camera_path = path
            self._start_camera_stream()
        else:
            self._install_robot_camera(robot)
        return True

    async def _run_camera_stream(self):
        """Capture the AGV viewport to an atomically replaced browser JPEG."""
        app = omni.kit.app.get_app()
        temporary = self._camera_frame_path.with_name("latest.tmp.jpg")
        try:
            while not self._is_shutting_down and self._robot_camera_path:
                stage = self._stage
                camera = stage.GetPrimAtPath(self._robot_camera_path) if stage else None
                if camera is None or not camera.IsValid():
                    await app.next_update_async()
                    continue
                if self._robot_viewport_window is None:
                    from .viewport_compat import create_viewport_for_camera
                    self._robot_viewport_window = create_viewport_for_camera(
                        viewport_name="AGV Front Camera",
                        camera_prim_path=self._robot_camera_path,
                        width=640,
                        height=360,
                        position_x=3400,
                        position_y=1800,
                    )
                    # Kit stops rendering hidden viewports. Keep the stream
                    # viewport live at the far edge of the desktop instead.
                    self._robot_viewport_window.visible = True
                viewport = getattr(self._robot_viewport_window, "viewport_api", None)
                if viewport is None:
                    raise RuntimeError("AGV camera viewport API is unavailable")
                viewport.camera_path = self._robot_camera_path
                capture = capture_viewport_to_file(viewport, file_path=str(temporary), is_hdr=False)
                if capture is not None:
                    await capture.wait_for_result()
                if temporary.is_file() and temporary.stat().st_size > 0:
                    os.replace(temporary, self._camera_frame_path)
                await asyncio.sleep(0.45)
        except asyncio.CancelledError:
            return
        except Exception as error:
            carb.log_error(f"[Browser Camera] stream failed: {type(error).__name__}: {error}")

    def _open_view_navigation(self):
        """Open compact focus, pan and zoom controls for the main viewport."""
        if self._view_nav_window is None:
            self._view_nav_window = ui.Window(
                "Warehouse View Navigation",
                width=390,
                height=185,
            )
            with self._view_nav_window.frame:
                with ui.VStack(spacing=5, height=0):
                    ui.Label(
                        "Select a Prim, then focus it. Pan/zoom keeps that center.",
                        height=22,
                    )
                    with ui.HStack(height=32, spacing=5):
                        self._button("Focus Selected", "robot", clicked_fn=self._focus_selected_view)
                        self._button("Warehouse Overview", "robot", clicked_fn=self._focus_warehouse_view)
                        self._button("Fullscreen (F7)", "robot", clicked_fn=self._toggle_viewport_fullscreen)
                    with ui.HStack(height=32, spacing=5):
                        self._button("← Pan", "robot", clicked_fn=lambda: self._pan_teaching_view(-1.0, 0.0))
                        self._button("→ Pan", "robot", clicked_fn=lambda: self._pan_teaching_view(1.0, 0.0))
                        self._button("↑ Pan", "robot", clicked_fn=lambda: self._pan_teaching_view(0.0, 1.0))
                        self._button("↓ Pan", "robot", clicked_fn=lambda: self._pan_teaching_view(0.0, -1.0))
                    with ui.HStack(height=32, spacing=5):
                        self._button("Zoom +", "robot", clicked_fn=lambda: self._zoom_teaching_view(0.75))
                        self._button("Zoom -", "robot", clicked_fn=lambda: self._zoom_teaching_view(1.333333))
                        self._button("Main View", "robot", clicked_fn=self._restore_previous_view)
        self._view_nav_window.visible = True
        self._set_status(
            "VIEW NAVIGATION: Stage/Viewport에서 객체를 선택한 뒤 "
            "Focus Selected를 누르세요."
        )

    def _selected_or_lab_prim(self, use_lab_root=False):
        stage = self._stage
        if stage is None:
            return None
        if use_lab_root:
            prim = stage.GetPrimAtPath(LAB_ROOT)
            return prim if prim.IsValid() else None
        selection = omni.usd.get_context().get_selection()
        paths = selection.get_selected_prim_paths() if selection else []
        for path in paths:
            prim = stage.GetPrimAtPath(path)
            if prim.IsValid():
                return prim
        return None

    def _focus_selected_view(self):
        prim = self._selected_or_lab_prim(use_lab_root=False)
        if prim is None:
            self._set_status("VIEW: Stage에서 확대할 객체 Prim을 먼저 선택하세요.")
            return
        self._frame_prim_with_teaching_camera(prim)

    def _focus_warehouse_view(self):
        prim = self._selected_or_lab_prim(use_lab_root=True)
        if self._room_kind == 'usd' and self._stage:
            room=self._stage.GetPrimAtPath(LAB_ROOT+'/Room')
            prim=room_focus_prim(self._stage,room) or prim
        if prim is None:
            self._set_status("VIEW: Lab 1-B Warehouse Scene을 먼저 생성하세요.")
            return
        self._frame_prim_with_teaching_camera(prim)

    def _frame_prim_with_teaching_camera(self, prim):
        """Frame one Prim using a movable teaching camera and a stable pivot."""
        try:
            purposes = [
                UsdGeom.Tokens.default_,
                UsdGeom.Tokens.render,
                UsdGeom.Tokens.proxy,
            ]
            cache = UsdGeom.BBoxCache(
                Usd.TimeCode.Default(), purposes, useExtentsHint=True
            )
            world_range = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            if world_range.IsEmpty():
                raise RuntimeError("selected Prim has no visible bounds")
            minimum = world_range.GetMin()
            maximum = world_range.GetMax()
            center = (minimum + maximum) * 0.5
            size = maximum - minimum
            largest = max(float(size[0]), float(size[1]), float(size[2]), 0.5)
            distance = max(2.2, largest * 2.25)
            eye = center + Gf.Vec3d(
                -0.85 * distance,
                -1.00 * distance,
                0.62 * distance,
            )
            if self._room_kind == 'usd':
                viewport=get_active_viewport()
                resolution=getattr(viewport,'resolution',(1280,720))
                aspect=float(resolution[0])/max(float(resolution[1]),1.0)
                eye=building_camera_eye(world_range,aspect)
            self._set_teaching_camera(eye, center)
            self._set_status(
                f"VIEW FOCUS: {prim.GetPath()} 중심으로 이동했습니다. "
                "Pan/Zoom 버튼 또는 Alt+가운데 마우스를 사용하세요."
            )
        except Exception as error:
            carb.log_error(f"[View Navigation] focus failed: {error}")
            self._set_status(f"VIEW FOCUS 실패: {type(error).__name__}: {error}")

    def _set_teaching_camera(self, eye, target):
        stage = self._stage
        viewport = get_active_viewport()
        if stage is None or viewport is None:
            raise RuntimeError("active main viewport is unavailable")
        current_path = str(getattr(viewport, "camera_path", "") or "")
        if current_path and current_path != TEACHING_CAMERA_PATH:
            self._previous_view_camera_path = current_path

        camera = UsdGeom.Camera.Define(stage, TEACHING_CAMERA_PATH)
        xform = UsdGeom.Xformable(camera.GetPrim())
        xform.ClearXformOpOrder()
        look_at = Gf.Matrix4d().SetLookAt(
            Gf.Vec3d(eye), Gf.Vec3d(target), Gf.Vec3d(0.0, 0.0, 1.0)
        ).GetInverse()
        xform.AddTransformOp().Set(look_at)
        camera.CreateProjectionAttr(UsdGeom.Tokens.perspective)
        camera.CreateFocalLengthAttr(26.0)
        camera.CreateHorizontalApertureAttr(20.955)
        camera.CreateVerticalApertureAttr(15.2908)
        camera.CreateClippingRangeAttr(Gf.Vec2f(0.05, 2000.0))
        self._teaching_camera_target = (
            float(target[0]), float(target[1]), float(target[2])
        )
        viewport.camera_path = TEACHING_CAMERA_PATH

    def _teaching_camera_eye(self):
        stage = self._stage
        camera = stage.GetPrimAtPath(TEACHING_CAMERA_PATH) if stage else None
        if camera is None or not camera.IsValid():
            return None
        matrix = UsdGeom.Xformable(camera).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )
        return matrix.ExtractTranslation()

    def _pan_teaching_view(self, horizontal, vertical):
        eye = self._teaching_camera_eye()
        target_tuple = self._teaching_camera_target
        if eye is None or target_tuple is None:
            self._focus_selected_view()
            return
        target = Gf.Vec3d(*target_tuple)
        view = target - eye
        distance = view.GetLength()
        if distance <= 1.0e-6:
            return
        view /= distance
        right = Gf.Cross(view, Gf.Vec3d(0.0, 0.0, 1.0))
        if right.GetLength() <= 1.0e-6:
            right = Gf.Vec3d(1.0, 0.0, 0.0)
        else:
            right.Normalize()
        screen_up = Gf.Cross(right, view)
        screen_up.Normalize()
        step = max(0.20, distance * 0.12)
        delta = right * (float(horizontal) * step) + screen_up * (float(vertical) * step)
        self._set_teaching_camera(eye + delta, target + delta)

    def _zoom_teaching_view(self, distance_factor):
        eye = self._teaching_camera_eye()
        target_tuple = self._teaching_camera_target
        if eye is None or target_tuple is None:
            self._focus_selected_view()
            return
        target = Gf.Vec3d(*target_tuple)
        offset = eye - target
        distance = offset.GetLength()
        if distance <= 1.0e-6:
            return
        new_distance = max(0.60, min(250.0, distance * float(distance_factor)))
        self._set_teaching_camera(target + offset * (new_distance / distance), target)

    def _restore_previous_view(self, silent=False):
        viewport = get_active_viewport()
        if viewport is None:
            return
        previous = self._previous_view_camera_path
        if previous:
            viewport.camera_path = previous
        self._teaching_camera_target = None
        if not silent:
            self._set_status("VIEW: 이전 메인 Viewport 카메라로 복귀했습니다.")

    def _toggle_viewport_fullscreen(self):
        """Execute Isaac Sim's fullscreen action (separate from panel hiding)."""
        try:
            import omni.kit.actions.core as actions_core

            registry = actions_core.get_action_registry()
            action = registry.get_action("omni.kit.ui.actions", "toggle_fullscreen")
            if action is None:
                raise RuntimeError("toggle_fullscreen action is unavailable")
            action.execute()
            carb.log_info("[View Navigation] toggled viewport fullscreen")
        except Exception as error:
            carb.log_error(f"[View Navigation] fullscreen action failed: {error}")
            self._set_status(
                "FULLSCREEN: 버튼 실행을 지원하지 않는 Kit 구성입니다. "
                "장면만 보기 버튼으로 편집 패널을 숨길 수 있습니다."
            )

    def _reveal_exit_ui(self):
        self._startup_view_cancelled = True
        """Make standard quit/save dialogs visible even when F7 hid Kit's UI."""
        window = omni.appwindow.get_default_app_window()
        if window.is_fullscreen():
            window.set_fullscreen(False)
        self._set_viewport_only(False)

    def _on_exit_requested(self, event):
        if event.type == omni.kit.app.POST_QUIT_EVENT_TYPE:
            try:
                self._reveal_exit_ui()
            except Exception as error:
                carb.log_warn(f"[Exit UI] Could not restore panels: {error}")
                # Keep the regular save/quit handler in control; never discard edits.
                carb.settings.get_settings().set("/app/window/hideUi", False)

    def _request_app_exit(self):
        self._reveal_exit_ui()
        omni.kit.app.get_app().post_quit()

    async def _startup_scene_only(self):
        app = omni.kit.app.get_app()
        try:
            for _ in range(120):
                await app.next_update_async()
            if self._is_shutting_down or self._startup_view_cancelled:return
            self._restore_startup_scene()
            self._ensure_robot_camera_stream()
            import omni.kit.actions.core as actions_core
            for _ in range(600):
                if self._is_shutting_down or self._startup_view_cancelled:
                    return
                if actions_core.get_action_registry().get_action("omni.kit.ui.actions", "toggle_ui"):
                    self._set_viewport_only(True)
                    return
                await app.next_update_async()
            self._set_status("자동 장면 보기 준비 시간이 초과되었습니다. 장면만 보기 버튼을 사용하세요.")
        except asyncio.CancelledError:
            return
        except Exception as error:
            self._set_status(f"자동 장면 보기 실패: {error}")

    def _narrate_manual_motion(self, linear, angular):
        if self._mission_active or self._agent_action_active:
            self._narrated_motion = None
            return
        motion = (1 if linear > 0 else -1 if linear < 0 else 0,
                  1 if angular > 0 else -1 if angular < 0 else 0)
        if motion == self._narrated_motion:
            return
        previous = self._narrated_motion
        self._narrated_motion = motion
        if motion == (0, 0):
            if previous and previous != (0, 0):
                self._narrate("action", "이동 명령이 해제되어 정지 명령을 보냈습니다.", transient=True)
            return
        parts = []
        if motion[0]: parts.append("전방으로 이동" if motion[0] > 0 else "후진")
        if motion[1]: parts.append("왼쪽으로 회전" if motion[1] > 0 else "오른쪽으로 회전")
        self._narrate("action", "조작 명령을 받았습니다. " + ", ".join(parts) + "하도록 요청합니다. 사람 접근 제한을 함께 적용합니다.", transient=True)

    def _update_narrator(self):
        while True:
            try:
                event_id, text, error = self._narrator.results.get_nowait()
            except queue.Empty:
                break
            if self._robot_messages.enrich(event_id, text):
                self._narrator_status = "Ollama 설명 연결됨" if not error else "기본 설명 유지 · " + error
        current = self._robot_messages.snapshot()["current"]
        if current and current["id"] != self._narrator_submitted_id and not self._agent_busy:
            self._narrator_submitted_id = current["id"]
            self._narrator.submit(current)

    def _narrate(self, category, message, **options):
        self._robot_messages.emit(category, message, **options)

    def _narrate_humidity(self, window, timestamp, humidity, previous_start, source, triggered):
        elapsed = timestamp - window.started_at
        rise = humidity - window.baseline
        if previous_start != window.started_at:
            prefix = "관찰 구간을 새로 시작합니다. " if previous_start is not None else ""
            target = window.baseline + window.rise
            target_message = (f"목표는 {target:.1f}%RH입니다." if target <= 100.0 else
                              "현재 기준에서는 +20%p 상승이 불가능합니다. 습도 연습을 다시 준비해 기준값을 초기화하세요.")
            self._narrate("sensor", f"{source}: {prefix}현재 습도 {humidity:.1f}%RH를 기준값으로 저장했습니다. {target_message}")
        elif triggered:
            self._humidity_trigger_description = f"{source}: {elapsed:.1f}초 동안 {rise:.1f}%p 상승했습니다."
            self._narrate("plan", self._humidity_trigger_description + " 발동 조건을 충족하여 장면과 미션 준비 상태를 확인합니다.")
        else:
            self._narrate("sensor", f"{source}: 현재 {humidity:.1f}%RH, {elapsed:.1f}초 동안 {rise:+.1f}%p 변화했습니다. +20%p 조건에 아직 도달하지 않았습니다.", key="humidity_" + source, interval=5)

    def _set_viewport_only(self, enabled):
        """Hide or restore Kit panels without changing the OS fullscreen state."""
        try:
            import carb.settings
            import omni.kit.actions.core as actions_core
            current = bool(carb.settings.get_settings().get("/app/window/hideUi"))
            if current != bool(enabled):
                action = actions_core.get_action_registry().get_action("omni.kit.ui.actions", "toggle_ui")
                if action is None:
                    raise RuntimeError("toggle_ui action is unavailable")
                action.execute()
            self._set_status("장면만 보기: 편집 패널을 숨겼습니다. 브라우저의 패널 복원으로 돌아오세요." if enabled else "편집 패널을 복원했습니다.")
        except Exception as error:
            self._set_status(f"화면 모드 변경 실패: {error}")

    def _start_homepage_agent(self, upstream):
        if os.environ.get("LAB_BROWSER_MODE") == "local":
            return
        project = self._extension_root().parents[1]
        config = project / ".web_runtime/homepage.json"
        if not config.is_file():
            return
        process = getattr(self, "_homepage_agent", None)
        if process is not None and process.poll() is None:
            return
        try:
            env = dict(os.environ)
            for key in ("PYTHONPATH", "PYTHONHOME", "LD_LIBRARY_PATH", "LD_PRELOAD"):
                env.pop(key, None)
            with (config.parent / "homepage.log").open("a") as log:
                self._homepage_agent = subprocess.Popen(
                    ["/usr/bin/python3", str(project / "web_app/homepage_agent.py"),
                     "--config", str(config), "--upstream", upstream,
                     "--owner-pid", str(os.getpid())], cwd=project, env=env,
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        except Exception as error:
            carb.log_warn(f"홈페이지 연결 시작 실패: {error}")

    def _start_remote_web_app(self, upstream):
        if os.environ.get("LAB_WEB_ENABLED", "0") != "1":
            return
        process = getattr(self, "_remote_web_app", None)
        if process is not None and process.poll() is None:
            return
        project = self._extension_root().parents[1]
        runtime = project / ".web_runtime"
        try:
            runtime.mkdir(exist_ok=True)
            env = dict(os.environ)
            for key in ("PYTHONPATH", "PYTHONHOME", "LD_LIBRARY_PATH", "LD_PRELOAD"):
                env.pop(key, None)
            with (runtime / "server.log").open("a") as log:
                self._remote_web_app = subprocess.Popen(
                    ["/usr/bin/python3", str(project / "web_app/server.py"),
                     "--runtime", str(runtime), "--upstream", upstream,
                     "--port", os.environ.get("LAB_WEB_PORT", "8080"),
                     "--owner-pid", str(os.getpid())],
                    cwd=project, env=env, stdout=log, stderr=subprocess.STDOUT,
                    start_new_session=True)
        except Exception as error:
            carb.log_warn(f"원격 웹 서버 시작 실패: {error}")

    def _homepage_browser_url(self, local_url):
        if os.environ.get("LAB_BROWSER_MODE") == "local":
            return local_url
        from urllib.parse import urlsplit
        config = self._extension_root().parents[1] / ".web_runtime/homepage.json"
        try:
            site = json.loads(config.read_text()).get("site_url", "")
            target = urlsplit(site)
            if (target.scheme == "https" and target.hostname and
                    not target.username and not target.password and
                    target.path in ("", "/") and not target.query and not target.fragment):
                return site.rstrip("/") + "/agv"
        except (OSError, ValueError, TypeError):
            pass
        return local_url

    def _open_external_control_panel(self):
        """Start the localhost bridge and open its panel in the OS browser."""
        try:
            if self._external_control_server is None:
                html_path = self._extension_root() / "assets" / "web_control" / "index.html"
                self._external_control_server = ExternalControlServer(html_path)
            url = self._external_control_server.start()
            self._start_remote_web_app(url)
            self._start_homepage_agent(url)
            url = self._homepage_browser_url(url)
            if sys.platform.startswith("win"):
                os.startfile(url)
            else:
                child_env = os.environ.copy()
                # Do not let Kit's bundled Python/shared-library/browser settings
                # leak into the desktop browser process.  In particular, xdg-open
                # can otherwise resolve to an in-Kit handler on some installations.
                for name in (
                    "PYTHONHOME", "PYTHONPATH", "LD_PRELOAD", "LD_LIBRARY_PATH",
                    "BROWSER",
                ):
                    child_env.pop(name, None)

                browser_command = None
                if sys.platform == "darwin":
                    opener = shutil.which("open")
                    if opener:
                        browser_command = [opener, "-a", "Safari", url]
                else:
                    # Prefer a real desktop-browser executable and explicitly ask
                    # for a new top-level window that is independent of Isaac Sim.
                    candidates = (
                        ("/usr/bin/firefox", ["--new-window"]),
                        ("/snap/bin/firefox", ["--new-window"]),
                        ("firefox", ["--new-window"]),
                        ("google-chrome", ["--new-window"]),
                        ("google-chrome-stable", ["--new-window"]),
                        ("chromium-browser", ["--new-window"]),
                        ("chromium", ["--new-window"]),
                    )
                    for executable, arguments in candidates:
                        resolved = executable if os.path.isfile(executable) else shutil.which(executable)
                        if resolved:
                            browser_command = [resolved, *arguments, url]
                            break
                    if browser_command is None:
                        opener = shutil.which("xdg-open")
                        if opener:
                            browser_command = [opener, url]

                if browser_command is None:
                    raise FileNotFoundError("No desktop browser executable was found")
                subprocess.Popen(
                    browser_command,
                    env=child_env,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    cwd=str(Path.home()),
                    start_new_session=True,
                    close_fds=True,
                )
            self._set_status(
                f"EXTERNAL CONTROL: {url} 독립 데스크톱 브라우저 창을 열었습니다. "
                "Isaac Sim 밖이나 두 번째 모니터로 이동해 사용하세요."
            )
        except Exception as error:
            carb.log_error(f"[External Control] open failed: {error}")
            self._set_status(f"EXTERNAL CONTROL 실패: {type(error).__name__}: {error}")

    def _drain_external_control_commands(self):
        server = self._external_control_server
        if server is None:
            return
        for action, value in server.drain():
            try:
                self._execute_external_control(action, value)
            except Exception as error:
                carb.log_error(f"[External Control] {action} failed: {error}")
                self._set_status(
                    f"EXTERNAL CONTROL {action} 실패: {type(error).__name__}: {error}"
                )

    def _navigation_observation(self, robot_xy):
        now = time.time()
        if now - self._navigation_geometry_time > .5:
            self._navigation_geometry = collision_rectangles(self._stage)
            self._navigation_geometry_time = now
        robot = self._stage.GetPrimAtPath(ROBOT_PATH) if self._stage else None
        scale = robot.GetAttribute("xformOp:scale").Get() if robot and robot.IsValid() else None
        half = [0.82 * abs(float(scale[0])), 0.55 * abs(float(scale[1]))] if scale is not None else [0.574, 0.385]
        body = self._stage.GetPrimAtPath(f"{ROBOT_PATH}/ModelVisuals/Body") if self._stage else None
        body_color = body.GetAttribute("primvars:displayColor").Get() if body and body.IsValid() else None
        return dict(timestamp=now, geometry_timestamp=self._navigation_geometry_time,
                    robot_body_color=list(body_color[0]) if body_color else None,
                    robot_half_extents=half, robot_clearance_radius=math.hypot(*half) + .05,
                    robot_scale=None if scale is None else list(scale),
                    rectangles=self._navigation_geometry,
                    pose=None if robot_xy is None else [float(robot_xy[0]), float(robot_xy[1]), self._robot_heading()],
                    source="USD collision geometry and ground-truth pose")

    def _start_navigation_goal(self, value):
        if self._current_busy_operation() or self._external_drive_key or self._pressed_keys.intersection({"UP", "DOWN", "LEFT", "RIGHT"}):
            self._narrate("safe", "현재 작업을 정지한 뒤 목표점 주행을 시작하세요.", alert="navigation")
            return
        try:
            target = tuple(float(v) for v in value)
            if len(target) != 2 or any(not math.isfinite(v) or abs(v) > 10 for v in target):
                raise ValueError("목표 X/Y는 -10~10m의 유한한 값이어야 합니다.")
            if not self._agv_is_installed():
                raise ValueError("AGV를 먼저 설치하세요.")
            self._navigation.select(self._navigation.mode)
        except (TypeError, ValueError) as error:
            self._narrate("safe", str(error), alert="navigation")
            return
        self._robot_messages.resolve("navigation")
        self._agent_turn_task = asyncio.ensure_future(self._run_navigation_goal(target))

    async def _run_navigation_goal(self, target):
        self._agent_action_active = True
        epoch = self._agent_motion_epoch
        self._reset_obstacle_avoidance()
        self._narrate("plan", f"{self._navigation.snapshot()['label']}: 목표 ({target[0]:.2f}, {target[1]:.2f})m로 접근합니다.")
        try:
            if not self._play_simulation():
                raise RuntimeError("시뮬레이션 재생 실패")
            app = omni.kit.app.get_app()
            deadline = time.monotonic() + 120
            while not self._is_shutting_down and epoch == self._agent_motion_epoch:
                if self._drive_robot_to(target, .6, .25):
                    self._narrate("done", "실제 AGV 위치가 목표점의 0.25m 범위 안에 도착했습니다.")
                    return
                if self._avoidance_state == "FAILED":
                    raise RuntimeError(self._avoidance_failure)
                if time.monotonic() > deadline:
                    raise RuntimeError("목표점 주행 120초 시간 초과")
                await app.next_update_async()
        except asyncio.CancelledError:
            self._narrate("action", "목표점 주행을 취소했습니다.")
        except Exception as error:
            self._narrate("error", str(error), alert="navigation")
        finally:
            self._navigation.cancel()
            self._command_robot_velocity(0.0, 0.0)
            self._agent_action_active = False

    def _publish_external_control_status(self):
        server = self._external_control_server
        if server is None:
            return
        try:
            selected_face = self._resolve_selected_face()
            selected_face_name = selected_face.name if selected_face is not None else "NONE"
            if self._face_path_model is not None:
                # Keep the native panel synchronized even when the portrait
                # was selected through the external browser control.
                self._face_path_model.set_value(
                    selected_face_name if selected_face_name != "NONE" else "No face image selected"
                )
            stage = self._stage
            robot = stage.GetPrimAtPath(ROBOT_PATH) if stage else None
            worker = stage.GetPrimAtPath(WORKER_PATH) if stage else None
            target = stage.GetPrimAtPath(TARGET_PATH) if stage else None
            robot_xy = self._robot_xy() if robot and robot.IsValid() else None
            worker_xy = self._prim_xy(worker) if worker and worker.IsValid() else None
            box_xy = self._prim_xy(target) if target and target.IsValid() else None
            moved = self._distance_xy(box_xy, self._box_initial_xy)
            worker_scale = 1.0
            if worker and worker.IsValid():
                scale_attr = worker.GetAttribute("lab:workerScale")
                if scale_attr.IsValid() and scale_attr.Get() is not None:
                    worker_scale = float(scale_attr.Get())
            sensor_armed = False
            if target and target.IsValid():
                sensor_attr = target.GetAttribute("lab:temperatureSensorTarget")
                sensor_armed = bool(
                    self._lab2_prepared
                    and sensor_attr.IsValid()
                    and sensor_attr.Get()
                )
            robot_speed = None
            handle = self._dynamic_robot_handle
            if handle != _dynamic_control.INVALID_HANDLE and self._timeline.is_playing():
                try:
                    velocity = self._dynamic_control.get_rigid_body_linear_velocity(handle)
                    robot_speed = math.sqrt(float(velocity.x) ** 2 + float(velocity.y) ** 2)
                except Exception:
                    robot_speed = None
            busy_operation = self._current_busy_operation()
            server.update_status({
                "navigation_observation": self._navigation_observation(robot_xy),
                "navigation": self._navigation.snapshot(),
                "robot_messages": self._robot_messages.snapshot(),
                "narrator_status": self._narrator_status,
                **self._object_editor.snapshot(),
                "saved_rooms": self._saved_rooms,
                "room_library": getattr(self, "_room_library", []),
                "object_library": getattr(self, "_object_library", []),
                "scene_dirty": self._scene_dirty,
                "startup_scene_file": self._startup_scene_file,
                "room_reset_available": getattr(self, "_room_start_recipe", None) is not None,
                "room_kind": self._room_kind,
                "room_branding": self._room_branding,
                "room_source": self._room_source,
                "viewport_only": bool(carb.settings.get_settings().get("/app/window/hideUi")),
                "window_fullscreen": bool(omni.appwindow.get_default_app_window().is_fullscreen()),
                "temperature_c": round(float(self._temperature), 1),
                "threshold_c": round(float(self._temperature_threshold), 1),
                "hardware_temperature_c": self._hardware_temperature,
                "hardware_sensor_error": self._hardware_sensor_error,
                "hardware_trigger": "humidity >= 20 percentage points rise within 20 s",
                "hardware_humidity_pct": self._hardware_humidity,
                "humidity_baseline_pct": self._hardware_rise.baseline,
                "humidity_target_pct": (self._hardware_rise.baseline + 20.0) if self._hardware_rise.baseline is not None and self._hardware_rise.baseline <= 80.0 else None,
                "humidity_pct": self._humidity,
                "ui_humidity_delta_pct": self._ui_humidity_delta,
                "hardware_humidity_delta_pct": self._hardware_humidity_delta,
                "ui_humidity_baseline_pct": self._ui_humidity_rise.baseline,
                "ui_humidity_target_pct": self._ui_humidity_rise.baseline + 20.0 if self._ui_humidity_rise.baseline is not None and self._ui_humidity_rise.baseline <= 80.0 else None,
                "hardware_sample_age_s": time.monotonic() - self._hardware_sample_timestamp if self._hardware_sample_timestamp is not None else None,
                "hardware_sample_timestamp": self._hardware_sample_timestamp,
                "hardware_sample_interval_s": self._hardware_sample_interval,
                "hardware_sample_count": self._hardware_sample_count,
                "sensor_mode": str(self._sensor_mode),
                "sensor_armed": sensor_armed,
                "navigation_timeout_s": self.NAVIGATION_TIMEOUT_SEC,
                "mission_total_timeout_s": self.MISSION_TOTAL_TIMEOUT_SEC,
                "mission_state": str(self._mission_state),
                "mission_active": bool(self._mission_active),
                "push_count": int(self._push_count),
                "box_position": None if box_xy is None else [round(box_xy[0], 2), round(box_xy[1], 2)],
                "box_moved_m": round(float(moved), 3),
                "robot_ready": bool(robot and robot.IsValid()),
                "robot_position": None if robot_xy is None else [round(robot_xy[0], 2), round(robot_xy[1], 2)],
                "robot_heading_deg": None if robot_xy is None else round(math.degrees(self._robot_heading()), 1),
                "robot_speed_mps": None if robot_speed is None else round(robot_speed, 3),
                "drive_backend": str(self._last_drive_backend),
                "background_simulation": "ENABLED" if self._background_simulation_enabled else "FAILED",
                "control_loop": (
                    "ASYNC ACTIVE"
                    if self._control_loop_task is not None and not self._control_loop_task.done()
                    else "STOPPED"
                ),
                "browser_drive_task": (
                    "ACTIVE"
                    if self._external_drive_task is not None and not self._external_drive_task.done()
                    else "IDLE"
                ),
                "mission_task": (
                    "ACTIVE"
                    if self._temperature_mission_task is not None and not self._temperature_mission_task.done()
                    else "IDLE"
                ),
                "requested_linear_mps": round(float(self._last_requested_linear_speed), 3),
                "requested_angular_dps": round(float(self._last_requested_angular_speed_deg), 1),
                "worker_position": None if worker_xy is None else [round(worker_xy[0], 2), round(worker_xy[1], 2)],
                "worker_scale": round(worker_scale, 2),
                "person_safety": str(self._person_safety_mode),
                "timeline": "PLAY" if self._timeline.is_playing() else "STOP",
                "manual_keys": sorted(self._pressed_keys.intersection({"UP", "DOWN", "LEFT", "RIGHT"})),
                "external_drive": self._external_drive_key,
                "operation_busy": bool(busy_operation),
                "operation_label": busy_operation,
                "selected_face": selected_face_name,
                "face3d_progress": self._face3d_progress or "IDLE",
                "experience_active": bool(self._experience_active),
                "experience_name": self._experience_name,
                "popup_id": int(self._popup_id),
                "popup_phase": self._popup_phase,
                "popup_title": self._popup_title,
                "popup_message": self._popup_message,
                "avoidance_state": self._avoidance_state,
                "avoidance_retry_count": int(self._avoidance_retry_count),
                "avoidance_max_retries": int(self.OBSTACLE_MAX_RETRIES),
                "message": str(self._last_status_text),
            })
        except Exception as error:
            carb.log_warn(f"[External Control] status snapshot skipped: {error}")

    def _execute_external_control(self, action, value=None):
        """Execute browser commands only from Kit's main update thread."""
        if action == "robot_body_red":
            self._set_agv_body_red()
            return
        if action == "robot_resize_default":
            if self._current_busy_operation():
                self._set_status("현재 작업을 정지한 뒤 AGV 크기를 변경하세요.")
                return
            self._agv_install_task = asyncio.ensure_future(self._resize_existing_agv())
            return
        if action == "navigation_goal":
            self._start_navigation_goal(value)
            return
        if action == "navigation_refresh":
            self._ensure_robot_camera_stream()
            self._navigation.probe()
            return
        if action == "navigation_select":
            busy = bool(self._current_busy_operation() or self._external_drive_key or self._pressed_keys.intersection({"UP", "DOWN", "LEFT", "RIGHT"}))
            try:
                self._navigation.select(str(value), busy=busy)
                self._reset_obstacle_avoidance()
                self._robot_messages.resolve("navigation")
                self._command_robot_velocity(0.0, 0.0)
                self._narrate("plan", self._navigation.snapshot()["label"] + "을 선택했습니다. 목표점 접근에 적용하며, 수동 조작·밀기·사람 접근 제한은 공통 제어를 사용합니다.")
            except ValueError as error:
                self._narrate("safe", str(error), alert="navigation")
            return
        if action == "message_ack":
            self._robot_messages.acknowledge()
            return
        if action in ("view_only", "view_panels", "app_exit"):
            self._startup_view_cancelled = True
        if action == "room_save":
            self._save_current_room(value)
            return
        if action == "room_load":
            self._load_saved_room(value)
            return
        if action == "room_refresh":
            self._room_library = room_library(self)
            self._object_library = room_library(self, "object")
            self._saved_rooms = saved_rooms(self)
            return
        if action == "room_discard":
            self._decline_scene_save()
            return
        if action == "room_reset":
            self._reset_current_room(value)
            return
        if action == "room_change":
            self._change_room(value)
            return
        if action in ("object_select", "object_add", "object_apply", "object_delete"):
            self._object_editor.handle(action, value)
            return
        if action == "face_select_uploaded":
            self._select_uploaded_face(value)
            return
        if action == "run_experience":
            self._start_experience(value)
            return
        direct = {
            "scene_minimal": self._create_minimal_scene,
            "scene_warehouse": self._create_demo_scene,
            "scene_warehouse_dongshin": self._create_dongshin_scene,
            "add_agv": self._add_agv_to_twin,
            "worker_start": self._start_worker_motion,
            "worker_stop": self._stop_worker_motion,
            "worker_reset": self._reset_worker_motion,
            "worker_rotate": self._rotate_worker_45,
            "face_map_2d": self._start_2d_face_mapping,
            "humidity_prepare": self._prepare_temperature_mission,
            "humidity_up": lambda: self._adjust_humidity(5.0),
            "humidity_down": lambda: self._adjust_humidity(-5.0),
            "temperature_prepare": self._prepare_temperature_mission,
            "temperature_up": lambda: self._adjust_temperature(5.0),
            "temperature_down": lambda: self._adjust_temperature(-5.0),
            "robot_stop": self._stop_robot,
            "robot_reset": self._reset_robot,
            "prepare_next_experience": self._prepare_next_experience,
            "agv_camera": self._show_robot_camera_view,
            "close_camera": self._show_main_view,
            "app_exit": self._request_app_exit,
            "view_only": lambda: self._set_viewport_only(True),
            "view_panels": lambda: self._set_viewport_only(False),
            "view_overview": self._focus_warehouse_view,
            "view_pan_left": lambda: self._pan_teaching_view(-1.0, 0.0),
            "view_pan_right": lambda: self._pan_teaching_view(1.0, 0.0),
            "view_pan_up": lambda: self._pan_teaching_view(0.0, 1.0),
            "view_pan_down": lambda: self._pan_teaching_view(0.0, -1.0),
            "view_zoom_in": lambda: self._zoom_teaching_view(0.75),
            "view_zoom_out": lambda: self._zoom_teaching_view(1.333333),
            "view_fullscreen": self._toggle_viewport_fullscreen,
            "view_restore": self._restore_previous_view,
            "fixed": lambda: self._set_mobility("fixed"),
            "movable": lambda: self._set_mobility("movable"),
            "kinematic": lambda: self._set_mobility("kinematic"),
            "mass": self._cycle_mass,
            "smaller": lambda: self._scale_selected(0.8),
            "larger": lambda: self._scale_selected(1.25),
            "color": self._cycle_color,
            "model": self._cycle_model,
        }
        handler = direct.get(action)
        if handler is not None:
            handler()
            return
        if action == "worker_smaller":
            self._scale_worker(0.8)
            return
        if action == "worker_larger":
            self._scale_worker(1.25)
            return
        movement = {
            "robot_forward": "UP",
            "robot_backward": "DOWN",
            "robot_left": "LEFT",
            "robot_right": "RIGHT",
        }
        if action in movement:
            if self._mission_active or self._agent_action_active:
                self._set_status("EXTERNAL CONTROL: 자동/Agentic 주행 중에는 수동 키가 잠깁니다.")
                return
            now = time.monotonic()
            next_key = movement[action]
            if self._external_drive_key != next_key:
                self._external_drive_started_at = now
            self._external_drive_key = next_key
            # Browser refreshes this lease while a key/button is held.  If the
            # browser closes unexpectedly, motion expires automatically.
            self._external_drive_lease_until = now + (0.45 if isinstance(value, dict) and value.get('tap_ms') == 450 else 0.90)
            if not self._play_simulation():
                self._external_drive_key = None
                return
            # Apply the browser command immediately on the Kit main thread.
            # The normal update loop keeps applying it between heartbeats.
            immediate_commands = {
                "UP": (self.ROBOT_LINEAR_SPEED, 0.0),
                "DOWN": (-self.ROBOT_LINEAR_SPEED, 0.0),
                "LEFT": (0.0, self.ROBOT_ANGULAR_SPEED_DEG),
                "RIGHT": (0.0, -self.ROBOT_ANGULAR_SPEED_DEG),
            }
            self._command_robot_velocity(*immediate_commands[next_key])
            if self._external_drive_task is None or self._external_drive_task.done():
                self._external_drive_task = asyncio.ensure_future(
                    self._run_external_drive()
                )
            self._set_status(f"EXTERNAL DRIVE: {self._external_drive_key}")
            return
        if action == "robot_release":
            self._external_drive_key = None
            self._external_drive_lease_until = 0.0
            if self._external_drive_task is not None and not self._external_drive_task.done():
                self._external_drive_task.cancel()
            self._external_drive_task = None
            self._command_robot_velocity(0.0, 0.0)
            return
        if action == "agent_prompt":
            prompt = str(value or "").strip()[:2000]
            self._submit_agent_prompt(prompt)

    def _show_robot_camera_view(self):
        self._ensure_robot_camera_stream()
        stage = self._stage
        camera = stage.GetPrimAtPath(self._robot_camera_path) if stage and self._robot_camera_path else None
        if camera is None or not camera.IsValid() or not camera.IsA(UsdGeom.Camera):
            self._set_status("Robot camera is unavailable. Complete Lab 1-C first.")
            return False
        try:
            from .viewport_compat import create_viewport_for_camera

            window = self._robot_viewport_window
            if window is None:
                window = create_viewport_for_camera(
                    viewport_name="AGV Front Camera",
                    camera_prim_path=self._robot_camera_path,
                    width=720,
                    height=480,
                    position_x=420,
                    position_y=160,
                )
                self._robot_viewport_window = window
            window.visible = True
            viewport = getattr(window, "viewport_api", None)
            if viewport is None:
                raise RuntimeError("Independent Viewport API is unavailable.")
            viewport.camera_path = self._robot_camera_path
            self._set_status(
                "ROBOT CAMERA VIEW: 독립 AGV 전방 카메라 창을 열었습니다. "
                "메인 Viewport는 외부 시점을 유지합니다."
            )
            return True
        except Exception as error:
            carb.log_error(f"[Robot Camera Window] open failed: {error}")
            self._robot_viewport_window = None
            self._set_status(
                f"독립 Robot Camera 창을 열지 못했습니다: "
                f"{type(error).__name__}: {error}"
            )
            return False

    def _show_main_view(self, silent=False):
        window = self._robot_viewport_window
        if window is not None:
            try:
                window.visible = False
                # Merely hiding an independent viewport leaves its camera
                # observer alive. Destroy it before a scene rebuild removes
                # LabFrontCamera, otherwise PhysX UI queries the deleted prim
                # every frame and floods the log with camera_state errors.
                destroy = getattr(window, "destroy", None)
                if callable(destroy):
                    destroy()
            except Exception as error:
                carb.log_warn(f"[Robot Camera Window] close failed: {error}")
            self._robot_viewport_window = None
        if not silent:
            self._set_status("ROBOT CAMERA CLOSED: 메인 외부 시점은 그대로 유지됩니다.")
        return True

    def _is_robot_camera_view_active(self):
        window = self._robot_viewport_window
        return bool(window is not None and getattr(window, "visible", False))

    def _set_franka_pose(self, pose_name, report=True):
        poses = {
            "home": (0.0, -20.0, 0.0, -120.0, 0.0, 100.0, 45.0),
            "observe": (0.0, -45.0, 0.0, -95.0, 0.0, 65.0, 45.0),
        }
        targets = poses.get(pose_name)
        robot = self._stage.GetPrimAtPath(ROBOT_PATH) if self._stage else None
        if targets is None or not robot or not robot.IsValid():
            if report:
                self._set_status("Install Ridgeback + Franka with Lab 1-C first.")
            return False
        found = 0
        for prim in Usd.PrimRange(robot):
            name = prim.GetName().lower()
            for index, target in enumerate(targets, start=1):
                if name == f"panda_joint{index}" or name.endswith(f"panda_joint{index}"):
                    drive = UsdPhysics.DriveAPI.Get(prim, UsdPhysics.Tokens.angular)
                    if not drive:
                        drive = UsdPhysics.DriveAPI.Apply(prim, UsdPhysics.Tokens.angular)
                    drive.CreateTargetPositionAttr().Set(float(target))
                    found += 1
                    break
        if found and report:
            self._timeline.play()
        if found == 7:
            self._last_arm_pose = pose_name
        if report:
            self._set_status(
                f"Franka pose → {pose_name.upper()} ({found}/7 arm joints found)"
            )
        return found == 7

    def _toggle_gripper(self):
        self._set_gripper_state(not self._gripper_open)

    def _set_gripper_state(self, is_open, report=True):
        """Drive the Franka gripper's finger joints to a specific open/closed state.

        Shared by the manual Gripper button (which toggles) and the Lab 3
        set_gripper agent tool (which wants an explicit target state so a
        repeated "그리퍼 열어줘" is idempotent instead of flipping back shut).
        """
        robot = self._stage.GetPrimAtPath(ROBOT_PATH) if self._stage else None
        if not robot or not robot.IsValid():
            if report:
                self._set_status("Install Ridgeback + Franka with Lab 1-C first.")
            return 0
        self._gripper_open = bool(is_open)
        target = 0.04 if self._gripper_open else 0.0
        found = 0
        for prim in Usd.PrimRange(robot):
            name = prim.GetName().lower()
            if "finger_joint" not in name:
                continue
            drive = UsdPhysics.DriveAPI.Get(prim, UsdPhysics.Tokens.linear)
            if not drive:
                drive = UsdPhysics.DriveAPI.Apply(prim, UsdPhysics.Tokens.linear)
            drive.CreateTargetPositionAttr().Set(float(target))
            found += 1
        if found:
            self._timeline.play()
        if report:
            state = "OPEN" if self._gripper_open else "CLOSED"
            self._set_status(f"Franka gripper → {state} ({found} finger joints found)")
        return found

    def _create_gist_wall_sign(self):
        """Place the existing logo on all three inner walls and the floor."""
        signs = (
            ("wall_right", GIST_SIGN_PATH, (5.0, -5.895, 1.5), (-1.0, 0.0, 0.0)),
            ("wall_left", f"{LAB_ROOT}/Branding/GISTWallSignLeft", (5.0, 5.895, 1.5), (1.0, 0.0, 0.0)),
            ("wall_back", f"{LAB_ROOT}/Branding/GISTWallSignBack", (10.895, 0.0, 1.5), (0.0, -1.0, 0.0)),
        )
        created = 0
        for wall_name, path, center, right in signs:
            if self._stage.GetPrimAtPath(f"{OBJECT_ROOT}/{wall_name}").IsValid():
                created += bool(self._create_gist_sign_on_wall(path, center, right))
        floor_created = False
        if self._stage.GetPrimAtPath(GROUND_PATH).IsValid():
            floor_created = self._create_gist_sign_on_wall(
                f"{LAB_ROOT}/Branding/GISTFloorSign", (-0.5, 0.0, 0.005),
                (0.0, -1.0, 0.0), up=(1.0, 0.0, 0.0), width=4.8, height=3.6,
            )
        carb.log_info(f"[GIST Sign] Created {created}/3 inner-wall signs and floor={floor_created}")
        return created == len(signs) and floor_created

    def _create_gist_sign_on_wall(self, sign_path, center, right, up=(0.0, 0.0, 1.0), width=3.2, height=2.4):
        """Create a wall or floor sign, preserving a supplied image's aspect ratio."""
        texture_path = (
            self._extension_root()
            / "assets"
            / "branding"
            / "gist_wall_logo.png"
        )
        supplied_logo = wall_logo_asset(self._extension_root() / 'assets')
        if supplied_logo is not None:
            texture_path, image_width, image_height = supplied_logo
            height = width * image_height / image_width
        if not texture_path.is_file():
            carb.log_warn(f"[GIST Sign] Texture not found: {texture_path}")
            return False

        stage = self._stage
        UsdGeom.Xform.Define(stage, f"{LAB_ROOT}/Branding")
        sign_root = UsdGeom.Xform.Define(stage, sign_path).GetPrim()
        sign_root.CreateAttribute("lab:objectType", Sdf.ValueTypeNames.String).Set(
            "wall_sign"
        )
        sign_root.CreateAttribute("lab:source", Sdf.ValueTypeNames.String).Set(
            "assets/branding/" + texture_path.name
        )

        center = Gf.Vec3f(*center)
        horizontal = Gf.Vec3f(*right) * (width / 2.0)
        vertical = Gf.Vec3f(*up) * (height / 2.0)
        points = [center - horizontal - vertical, center + horizontal - vertical,
                  center + horizontal + vertical, center - horizontal + vertical]
        mesh = UsdGeom.Mesh.Define(stage, f"{sign_path}/LogoMesh")
        mesh.CreatePointsAttr(points)
        mesh.CreateFaceVertexCountsAttr([4])
        mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        mesh.CreateDoubleSidedAttr(True)
        mesh.CreateExtentAttr([
            Gf.Vec3f(*(min(p[axis] for p in points) for axis in range(3))),
            Gf.Vec3f(*(max(p[axis] for p in points) for axis in range(3))),
        ])
        st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex
        )
        st.Set(
            [
                Gf.Vec2f(0.0, 0.0),
                Gf.Vec2f(1.0, 0.0),
                Gf.Vec2f(1.0, 1.0),
                Gf.Vec2f(0.0, 1.0),
            ]
        )

        material_path = f"{sign_path}/LogoMaterial"
        material = UsdShade.Material.Define(stage, material_path)
        surface = UsdShade.Shader.Define(stage, f"{material_path}/Surface")
        surface.CreateIdAttr("UsdPreviewSurface")
        surface.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.45)
        surface.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)

        reader = UsdShade.Shader.Define(stage, f"{material_path}/PrimvarReader")
        reader.CreateIdAttr("UsdPrimvarReader_float2")
        reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)

        texture = UsdShade.Shader.Define(stage, f"{material_path}/Texture")
        texture.CreateIdAttr("UsdUVTexture")
        texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(
            Sdf.AssetPath(texture_path.as_posix())
        )
        texture.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
        texture.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("clamp")
        texture.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("clamp")
        texture.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
            reader.ConnectableAPI(), "result"
        )
        texture.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
        surface.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
            texture.ConnectableAPI(), "rgb"
        )
        surface.CreateOutput("surface", Sdf.ValueTypeNames.Token)
        material.CreateSurfaceOutput().ConnectToSource(
            surface.ConnectableAPI(), "surface"
        )
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
        carb.log_info(
            f"[GIST Sign] Created {width}m x {height}m sign at {sign_path} from {texture_path}"
        )
        return True

    async def _load_objects_from_csv_async(self, app, max_objects=None, room_config=None):
        csv_path = self._extension_root() / "data" / "warehouse_objects.csv"
        if not csv_path.exists():
            raise FileNotFoundError(f"Object data not found: {csv_path}")

        count = 0
        with csv_path.open("r", encoding="utf-8-sig", newline="") as file:
            for row in csv.DictReader(file):
                if not row.get("object_id") or row["object_id"].startswith("#"):
                    continue
                if not include_csv_object(row, room_config):
                    continue
                width = float(row["width_m"])
                depth = float(row["depth_m"])
                height = float(row["height_m"])
                z_value = float(row["z_m"]) if row.get("z_m") else height / 2.0
                prim = self._create_cube(
                    f"{OBJECT_ROOT}/{row['object_id']}",
                    position=(float(row["x_m"]), float(row["y_m"]), z_value),
                    scale=(width / 2.0, depth / 2.0, height / 2.0),
                    color=(float(row["color_r"]), float(row["color_g"]), float(row["color_b"])),
                    mobility=row["mobility"].strip().lower(),
                    mass=float(row["mass_kg"]),
                )
                prim.CreateAttribute("lab:objectType", Sdf.ValueTypeNames.String).Set(
                    row.get("object_type", "unknown")
                )
                prim.CreateAttribute("lab:source", Sdf.ValueTypeNames.String).Set(
                    "warehouse_objects.csv"
                )
                count += 1
                # Keep Kit responsive while USD/PhysX notices are processed.
                # This is especially important with Isaac Sim 4.5 / USD 22.x.
                if count % 3 == 0:
                    self._set_status(f"CSV 객체 생성 중: {count}")
                    await app.next_update_async()
                if max_objects is not None and count >= max_objects:
                    break
        return count

    def _create_cube(self, path, position, scale, color, mobility, mass):
        if path.startswith(f"{OBJECT_ROOT}/"):
            return self._create_model_object(path, position, scale, color, mobility, mass)
        cube = UsdGeom.Cube.Define(self._stage, path)
        cube.CreateSizeAttr(2.0)
        cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        prim = cube.GetPrim()
        xform = self._author_clean_transform(
            prim,
            position=position,
            scale=scale,
        )
        prim.CreateAttribute("lab:initialPosition", Sdf.ValueTypeNames.Double3).Set(
            Gf.Vec3d(*position)
        )
        prim.CreateAttribute("lab:initialScale", Sdf.ValueTypeNames.Float3).Set(
            Gf.Vec3f(*scale)
        )
        prim.CreateAttribute("lab:initialOrientation", Sdf.ValueTypeNames.Quatf).Set(
            Gf.Quatf(1.0, Gf.Vec3f(0.0))
        )
        prim.CreateAttribute("lab:mobility", Sdf.ValueTypeNames.String).Set(mobility)
        prim.CreateAttribute("lab:massKg", Sdf.ValueTypeNames.Float).Set(float(mass))
        UsdPhysics.CollisionAPI.Apply(prim)
        self._apply_mobility(prim, mobility)
        self._set_mass_attr(prim, mass)
        return prim

    def _create_model_object(self, path, position, scale, color, mobility, mass):
        """Create a physics parent with separate collision and visual children."""
        parent = UsdGeom.Xform.Define(self._stage, path).GetPrim()
        self._author_clean_transform(parent, position, scale)
        parent.CreateAttribute("lab:initialPosition", Sdf.ValueTypeNames.Double3).Set(
            Gf.Vec3d(*position)
        )
        parent.CreateAttribute("lab:initialScale", Sdf.ValueTypeNames.Float3).Set(
            Gf.Vec3f(*scale)
        )
        parent.CreateAttribute("lab:initialOrientation", Sdf.ValueTypeNames.Quatf).Set(
            Gf.Quatf(1.0, Gf.Vec3f(0.0))
        )
        parent.CreateAttribute("lab:mobility", Sdf.ValueTypeNames.String).Set(mobility)
        parent.CreateAttribute("lab:massKg", Sdf.ValueTypeNames.Float).Set(float(mass))
        parent.CreateAttribute("lab:modelIndex", Sdf.ValueTypeNames.Int).Set(0)
        parent.CreateAttribute("lab:color", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(*color)
        )

        collision = UsdGeom.Cube.Define(self._stage, f"{path}/CollisionShape")
        collision.CreateSizeAttr(2.0)
        collision.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
        UsdPhysics.CollisionAPI.Apply(collision.GetPrim())

        visual_path = f"{path}/ModelVisuals"
        UsdGeom.Xform.Define(self._stage, visual_path)
        box = self._create_visual_cube(
            f"{visual_path}/Box", (0.0, 0.0, 0.0),
            (1.0, 1.0, 1.0), color,
        )
        box.CreateAttribute("lab:tintable", Sdf.ValueTypeNames.Bool).Set(True)

        self._apply_mobility(parent, mobility)
        self._set_mass_attr(parent, mass)
        return parent

    def _create_visual_cube(self, path, position, scale, color):
        cube = UsdGeom.Cube.Define(self._stage, path)
        cube.CreateSizeAttr(2.0)
        cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        # Character parts are children of an invisible collision Cube.
        # Author opacity locally so the parent's opacity=0 is not inherited.
        cube.CreateDisplayOpacityAttr([1.0])
        self._author_clean_transform(cube.GetPrim(), position, scale)
        return cube.GetPrim()

    @staticmethod
    def _author_clean_transform(prim, position, scale):
        """Author transform attributes directly for USD 22.x and newer.

        Isaac Sim 4.5 can retain xformOp property specs while a Prim is being
        rebuilt. Calling AddXformOp then fails because the named property already
        exists. Create or reuse the attributes directly and author one explicit
        xformOpOrder instead.
        """
        translate = prim.CreateAttribute(
            "xformOp:translate", Sdf.ValueTypeNames.Double3, custom=False
        )
        orient = prim.CreateAttribute(
            "xformOp:orient", Sdf.ValueTypeNames.Quatf, custom=False
        )
        scale_attr = prim.CreateAttribute(
            "xformOp:scale", Sdf.ValueTypeNames.Float3, custom=False
        )
        order = prim.CreateAttribute(
            "xformOpOrder", Sdf.ValueTypeNames.TokenArray, custom=False
        )

        translate.Set(Gf.Vec3d(*position))
        orient.Set(Gf.Quatf(1.0, Gf.Vec3f(0.0)))
        scale_attr.Set(Gf.Vec3f(*scale))
        order.Set([
            "xformOp:translate",
            "xformOp:orient",
            "xformOp:scale",
        ])
        return UsdGeom.Xformable(prim)

    def _editor_selection_paths(self):
        selection = omni.usd.get_context().get_selection()
        return selection.get_selected_prim_paths() if selection else []

    def _editor_clear_selection(self):
        omni.usd.get_context().get_selection().set_selected_prim_paths([], True)

    def _get_selected_lab_prim(self):
        selection = omni.usd.get_context().get_selection()
        paths = selection.get_selected_prim_paths()
        if not paths:
            self._set_status("먼저 실험 객체를 클릭해 선택하세요.")
            return None
        prim = self._stage.GetPrimAtPath(paths[0])
        if not prim.IsValid() or not str(prim.GetPath()).startswith(OBJECT_ROOT):
            self._set_status("/World/InteractivePhysicsLab/Objects 아래 객체를 선택하세요.")
            return None
        # Viewport clicks select the visible child (for example ModelVisuals/Box).
        # Resolve it back to the direct object parent that owns physics/state.
        while prim.GetParent().IsValid() and str(prim.GetParent().GetPath()) != OBJECT_ROOT:
            prim = prim.GetParent()
        return prim

    def _select_path(self, path):
        omni.usd.get_context().get_selection().set_selected_prim_paths([path], True)

    def _set_mobility(self, mobility):
        prim = self._get_selected_lab_prim()
        if prim is None:
            return
        self._safe_edit(lambda: self._apply_mobility(prim, mobility))
        self._report_selected(prim, f"Mobility → {mobility.upper()}")

    def _apply_mobility(self, prim, mobility):
        collision_prim = prim.GetChild("CollisionShape")
        if not collision_prim.IsValid():
            collision_prim = prim
        collision = UsdPhysics.CollisionAPI(collision_prim)
        if not collision:
            UsdPhysics.CollisionAPI.Apply(collision_prim)

        rigid = UsdPhysics.RigidBodyAPI(prim)
        if not rigid:
            rigid = UsdPhysics.RigidBodyAPI.Apply(prim)

        rigid.CreateRigidBodyEnabledAttr(True).Set(True)

        # On Isaac Sim 4.5, removing RigidBodyAPI while the PhysX stage is
        # already active can leave the previous dynamic actor alive.  Lock
        # Fixed objects as kinematic actors instead: they still collide, but
        # collision impulses cannot move them.  Kinematic remains available as
        # the code-driven movable state used by the lesson.
        is_locked = mobility in ("fixed", "kinematic")
        rigid.CreateKinematicEnabledAttr(is_locked).Set(is_locked)

        # Clear velocity left from an earlier Movable collision immediately.
        rigid.CreateVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        rigid.CreateAngularVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))

        prim.CreateAttribute("lab:mobility", Sdf.ValueTypeNames.String).Set(mobility)

    def _cycle_mass(self):
        prim = self._get_selected_lab_prim()
        if prim is None:
            return
        current_attr = prim.GetAttribute("lab:massKg")
        current = float(current_attr.Get() or self.MASS_LEVELS[0])
        next_mass = self.MASS_LEVELS[0]
        for value in self.MASS_LEVELS:
            if value > current + 1e-6:
                next_mass = value
                break
        self._safe_edit(lambda: self._set_mass_attr(prim, next_mass))
        self._report_selected(prim, f"Mass → {next_mass:g} kg")

    def _set_mass_attr(self, prim, mass):
        mass_api = UsdPhysics.MassAPI(prim)
        if not mass_api:
            mass_api = UsdPhysics.MassAPI.Apply(prim)
        mass_api.CreateMassAttr().Set(float(mass))
        prim.CreateAttribute("lab:massKg", Sdf.ValueTypeNames.Float).Set(float(mass))

    def _cycle_color(self):
        prim = self._get_selected_lab_prim()
        if prim is None:
            return
        current_index = self._current_color_index(prim)
        next_index = (current_index + 1) % len(self.COLOR_LEVELS)

        self._safe_edit(lambda: self._apply_color_index(prim, next_index))
        self._report_selected(prim, f"Color → {self.COLOR_LEVELS[next_index][0]}")

    def _current_color_index(self, prim):
        index_attr = prim.GetAttribute("lab:colorIndex")
        if index_attr.IsValid() and index_attr.Get() is not None:
            return int(index_attr.Get())
        stored = prim.GetAttribute("lab:color").Get()
        if stored is None:
            return -1
        current = stored
        return min(
            range(len(self.COLOR_LEVELS)),
            key=lambda i: sum(
                (float(current[j]) - float(self.COLOR_LEVELS[i][1][j])) ** 2
                for j in range(3)
            ),
        )

    def _apply_color_index(self, prim, index):
        """Author lab:color/lab:colorIndex and retint any tintable visual child.

        Shared by the manual C-key cycle and the Lab 3 set_color agent tool
        so both paths stay in exact sync.
        """
        color_name, color = self.COLOR_LEVELS[index]
        prim.CreateAttribute("lab:color", Sdf.ValueTypeNames.Color3f).Set(color)
        prim.CreateAttribute("lab:colorIndex", Sdf.ValueTypeNames.Int).Set(index)
        visual_root = self._stage.GetPrimAtPath(f"{prim.GetPath()}/ModelVisuals")
        if visual_root.IsValid():
            for child in Usd.PrimRange(visual_root):
                if child.GetAttribute("lab:tintable").Get():
                    child_color = UsdGeom.Gprim(child).GetDisplayColorAttr()
                    if child_color.IsValid():
                        child_color.Set([color])
        return color_name

    def _cycle_model(self):
        prim = self._get_selected_lab_prim()
        if prim is None:
            return

        model_attr = prim.GetAttribute("lab:modelIndex")
        current_index = int(model_attr.Get()) if model_attr.IsValid() and model_attr.Get() is not None else 0
        next_index = (current_index + 1) % len(self.MODEL_LEVELS)

        self._safe_edit(lambda: self._apply_model_index(prim, next_index))
        # H replaces the selected visual subtree. Re-select the persistent
        # physics parent so repeated H presses continue cycling models instead
        # of leaving selection on a deleted child prim.
        self._select_path(str(prim.GetPath()))
        self._report_selected(prim, f"Model → {self.MODEL_LEVELS[next_index][0]}")

    def _apply_model_index(self, prim, index):
        """Replace ModelVisuals with the given MODEL_LEVELS entry.

        Shared by the manual H-key cycle and the Lab 3 set_model agent tool.
        """
        model_name, model_key = self.MODEL_LEVELS[index]
        visual_path = f"{prim.GetPath()}/ModelVisuals"
        if self._stage.GetPrimAtPath(visual_path).IsValid():
            self._stage.RemovePrim(visual_path)
        if model_key == "box":
            UsdGeom.Xform.Define(self._stage, visual_path)
            stored_color = prim.GetAttribute("lab:color").Get()
            if stored_color is None:
                stored_color = Gf.Vec3f(0.10, 0.32, 0.85)
            box = self._create_visual_cube(
                f"{visual_path}/Box", (0.0, 0.0, 0.0),
                (1.0, 1.0, 1.0), stored_color,
            )
            box.CreateAttribute("lab:tintable", Sdf.ValueTypeNames.Bool).Set(True)
        else:
            self._create_character_model(prim, model_key)
        prim.CreateAttribute("lab:modelIndex", Sdf.ValueTypeNames.Int).Set(index)
        prim.CreateAttribute("lab:editorModel", Sdf.ValueTypeNames.String).Set(model_key)
        prim.CreateAttribute("lab:editorUsd", Sdf.ValueTypeNames.String).Set("")
        if str(prim.GetPath()) != WORKER_PATH:
            prim.CreateAttribute("lab:semanticLabel", Sdf.ValueTypeNames.String).Set("person" if model_key == "worker" else "object")
        return model_name

    def _create_character_model(self, parent_prim, model_key):
        root_path = f"{parent_prim.GetPath()}/ModelVisuals"
        UsdGeom.Xform.Define(self._stage, root_path)
        blue = parent_prim.GetAttribute("lab:color").Get()
        if blue is None:
            blue = Gf.Vec3f(0.10, 0.32, 0.85)
        dark = Gf.Vec3f(0.04, 0.05, 0.07)
        skin = Gf.Vec3f(0.82, 0.58, 0.38)
        yellow = Gf.Vec3f(0.96, 0.72, 0.05)
        silver = Gf.Vec3f(0.52, 0.56, 0.62)

        def cube(name, pos, scale, color, tint=False):
            child = self._create_visual_cube(f"{root_path}/{name}", pos, scale, color)
            child.CreateAttribute("lab:tintable", Sdf.ValueTypeNames.Bool).Set(tint)

        def sphere(name, pos, radius, color, tint=False):
            geom = UsdGeom.Sphere.Define(self._stage, f"{root_path}/{name}")
            geom.CreateRadiusAttr(radius)
            geom.CreateDisplayColorAttr([color])
            geom.CreateDisplayOpacityAttr([1.0])
            self._author_clean_transform(geom.GetPrim(), pos, (1.0, 1.0, 1.0))
            geom.GetPrim().CreateAttribute("lab:tintable", Sdf.ValueTypeNames.Bool).Set(tint)

        def cylinder(name, pos, radius, height, color, tint=False):
            geom = UsdGeom.Cylinder.Define(self._stage, f"{root_path}/{name}")
            geom.CreateRadiusAttr(radius)
            geom.CreateHeightAttr(height)
            geom.CreateAxisAttr(UsdGeom.Tokens.z)
            geom.CreateDisplayColorAttr([color])
            geom.CreateDisplayOpacityAttr([1.0])
            self._author_clean_transform(geom.GetPrim(), pos, (1.0, 1.0, 1.0))
            geom.GetPrim().CreateAttribute("lab:tintable", Sdf.ValueTypeNames.Bool).Set(tint)

        if model_key == "worker":
            sphere("Head", (0.0, 0.0, 0.68), 0.22, skin)
            cylinder("Helmet", (0.0, 0.0, 0.86), 0.25, 0.12, yellow)
            cube("Torso", (0.0, 0.0, 0.18), (0.34, 0.20, 0.42), blue, True)
            cube("ArmL", (0.0, 0.32, 0.18), (0.12, 0.10, 0.40), blue, True)
            cube("ArmR", (0.0, -0.32, 0.18), (0.12, 0.10, 0.40), blue, True)
            cube("LegL", (0.0, 0.17, -0.55), (0.14, 0.13, 0.34), dark)
            cube("LegR", (0.0, -0.17, -0.55), (0.14, 0.13, 0.34), dark)
        elif model_key == "robot":
            cube("Body", (0.0, 0.0, 0.10), (0.42, 0.36, 0.42), blue, True)
            cube("Head", (0.0, 0.0, 0.67), (0.30, 0.28, 0.20), silver)
            sphere("EyeL", (-0.29, 0.13, 0.70), 0.06, Gf.Vec3f(0.0, 0.95, 1.0))
            sphere("EyeR", (-0.29, -0.13, 0.70), 0.06, Gf.Vec3f(0.0, 0.95, 1.0))
            cube("ArmL", (0.0, 0.50, 0.08), (0.13, 0.12, 0.38), silver)
            cube("ArmR", (0.0, -0.50, 0.08), (0.13, 0.12, 0.38), silver)
            cube("Base", (0.0, 0.0, -0.48), (0.48, 0.42, 0.16), dark)
        elif model_key == "forklift":
            cube("Body", (0.15, 0.0, -0.15), (0.60, 0.52, 0.42), yellow, True)
            cube("Cabin", (0.15, 0.0, 0.42), (0.38, 0.42, 0.30), dark)
            cube("MastL", (-0.58, 0.30, 0.18), (0.08, 0.07, 0.78), silver)
            cube("MastR", (-0.58, -0.30, 0.18), (0.08, 0.07, 0.78), silver)
            cube("ForkL", (-0.78, 0.27, -0.62), (0.48, 0.07, 0.06), silver)
            cube("ForkR", (-0.78, -0.27, -0.62), (0.48, 0.07, 0.06), silver)
            cylinder("WheelFL", (-0.25, 0.55, -0.48), 0.20, 0.16, dark)
            cylinder("WheelFR", (-0.25, -0.55, -0.48), 0.20, 0.16, dark)
            cylinder("WheelRL", (0.55, 0.55, -0.48), 0.20, 0.16, dark)
            cylinder("WheelRR", (0.55, -0.55, -0.48), 0.20, 0.16, dark)

    def _scale_selected(self, factor):
        prim = self._get_selected_lab_prim()
        if prim is None:
            return
        self._safe_edit(lambda: self._apply_scale_factor(prim, factor))
        self._report_selected(prim, f"Scale × {factor:g}")

    def _apply_scale_factor(self, prim, factor):
        """Multiply the prim's current xformOp:scale by factor (min 0.1 per axis).

        Shared by the manual [ / ] keys and the Lab 3 scale_object agent tool.
        """
        xform = UsdGeom.Xformable(prim)
        scale_op = self._find_xform_op(xform, UsdGeom.XformOp.TypeScale)
        old = scale_op.Get() if scale_op else Gf.Vec3f(1.0)
        new = Gf.Vec3f(
            max(0.1, old[0] * factor),
            max(0.1, old[1] * factor),
            max(0.1, old[2] * factor),
        )
        if scale_op is None:
            scale_op = UsdGeom.XformOp(
                prim.CreateAttribute(
                    "xformOp:scale",
                    Sdf.ValueTypeNames.Float3,
                    custom=False,
                )
            )
        scale_op.Set(new)
        return new

    @staticmethod
    def _find_xform_op(xform, op_type):
        for op in xform.GetOrderedXformOps():
            if op.GetOpType() == op_type:
                return op
        return None

    def _initialize_mission_from_stage(self, armed=False):
        target = self._stage.GetPrimAtPath(TARGET_PATH) if self._stage else None
        self._temperature = 25.0
        self._hardware_rise.clear()
        self._ui_humidity_rise.clear()
        self._humidity = 50.0
        self._ui_humidity_delta = None
        self._humidity_armed_at = time.monotonic()
        self._mission_state = "IDLE"
        self._mission_active = False
        self._mission_latched = False
        self._lab2_prepared = bool(armed) and not is_ai_graduate_room({'kind':self._room_kind,'usd_path':self._room_source})
        self._sensor_unarmed_warning_latched = False
        self._push_count = 0
        self._state_elapsed = 0.0
        self._mission_total_elapsed = 0.0
        self._settle_elapsed = 0.0
        self._mission_debug_elapsed = 0.0
        self._approach_xy = None
        self._push_start_xy = None
        self._push_box_start_xy = None
        self._push_contact_detected = False
        self._push_contact_elapsed = 0.0
        self._backoff_start_xy = None
        self._reset_obstacle_avoidance()
        self._box_initial_xy = self._prim_xy(target) if target and target.IsValid() else None
        self._box_current_xy = self._box_initial_xy
        self._box_original_color = None
        if target and target.IsValid():
            color = target.GetAttribute("lab:color").Get()
            if color is not None:
                self._box_original_color = Gf.Vec3f(float(color[0]), float(color[1]), float(color[2]))
            target.CreateAttribute("lab:temperatureSensorTarget", Sdf.ValueTypeNames.Bool).Set(bool(armed))
            self._apply_mobility(target, "movable")
        self._refresh_mission_ui()

    def _process_temperature(self, value, source="Keyboard Simulation"):
        """UI and keyboard temperature input retains its absolute threshold."""
        self._temperature = max(-50.0, min(250.0, float(value)))
        if self._temperature < self._temperature_threshold:
            self._sensor_unarmed_warning_latched = False
        self._sensor_mode = str(source)
        self._refresh_mission_ui()
        # Isaac Sim 4.5 occasionally postpones a StringField repaint while a
        # button callback is still returning.  Refresh once more on the next
        # Kit update without moving any USD/PhysX work off the main thread.
        if self._temperature_refresh_task is None or self._temperature_refresh_task.done():
            self._temperature_refresh_task = asyncio.ensure_future(
                self._refresh_temperature_ui_next_update()
            )
        carb.log_info(
            f"[Lab 2 Temperature] {self._temperature:.1f} C "
            f"(threshold {self._temperature_threshold:.1f} C, source={self._sensor_mode})"
        )
        # Temperature is now informational; humidity alone starts Lab 2.

    def _drain_hardware_temperature(self):
        while True:
            try:
                timestamp, value, error = self._hardware_reader.events.get_nowait()
            except queue.Empty:
                break
            previous_error = self._hardware_sensor_error
            self._hardware_sensor_error = error or ""
            if error or time.monotonic() - timestamp > 3.0:
                if self._lab2_prepared and self._hardware_rise.baseline is not None:
                    self._narrate("error", "USB 측정 오류 또는 오래된 데이터로 USB 기준만 초기화했습니다. 버튼 연습 기준은 유지됩니다. " + str(error or "측정 지연"), key="sensor_error", interval=15, alert="sensor")
                self._hardware_rise.clear()
                continue
            if previous_error:
                self._robot_messages.resolve("sensor")
                self._narrate("sensor", "USB 센서의 유효한 측정이 다시 들어왔습니다.")
            self._robot_messages.resolve("sensor")
            self._hardware_temperature, humidity = value
            self._hardware_humidity_delta = None if self._hardware_humidity is None else humidity - self._hardware_humidity
            self._hardware_humidity = humidity
            if self._hardware_sample_timestamp is not None:
                self._hardware_sample_interval = timestamp - self._hardware_sample_timestamp
            self._hardware_sample_timestamp = timestamp
            self._hardware_sample_count += 1
            if timestamp < getattr(self, "_humidity_armed_at", 0.0):
                continue
            if not self._lab2_prepared or self._mission_active or self._mission_latched or self._agent_action_active:
                self._hardware_rise.clear()
                continue
            previous_start = self._hardware_rise.started_at
            triggered = self._hardware_rise.add(timestamp, humidity)
            self._narrate_humidity(self._hardware_rise, timestamp, humidity, previous_start, "USB 실측", triggered)
            self._try_temperature_trigger(triggered, "USB humidity rose >= 20 percentage points within 20 s")

    def _adjust_humidity(self, delta):
        if self._agent_action_active or self._mission_active or self._mission_latched:
            message = ("습도 미션이 끝났습니다. ‘습도 미션 준비 · 50%로 초기화’를 눌러 다시 시작하세요."
                       if self._mission_latched and not self._mission_active else
                       "로봇 작업이 진행 중입니다. 작업을 마친 뒤 습도를 조절하세요.")
            self._set_status(message)
            return
        before = self._humidity
        self._humidity = max(0.0, min(100.0, before + float(delta)))
        self._ui_humidity_delta = self._humidity - before
        if not self._lab2_prepared:
            self._ui_humidity_rise.clear()
            self._set_status("습도만 변경했습니다. 미션을 실행하려면 먼저 ‘습도 미션 준비 · 50%로 초기화’를 누르세요.")
            self._refresh_mission_ui()
            return
        timestamp = time.monotonic()
        window = self._ui_humidity_rise
        if (window.started_at is None or timestamp < window.started_at
                or timestamp - window.started_at > window.seconds):
            # Count the first click's increase, even after waiting at READY.
            window.clear()
            window.add(timestamp, before)
            self._narrate_humidity(window, timestamp, before, None, "UI 시뮬레이션", False)
        previous_start = window.started_at
        triggered = window.add(timestamp, self._humidity)
        self._narrate_humidity(window, timestamp, self._humidity, previous_start, "UI 시뮬레이션", triggered)
        self._try_temperature_trigger(triggered, "UI humidity rose >= 20 percentage points within 20 s")
        self._refresh_mission_ui()

    def _try_temperature_trigger(self, triggered, reason):
        if is_ai_graduate_room({'kind':self._room_kind,'usd_path':self._room_source}):return
        if (
            triggered
            and not self._agent_action_active
            and not self._mission_active
            and not self._mission_latched
        ):
            target = self._stage.GetPrimAtPath(TARGET_PATH) if self._stage else None
            robot = self._stage.GetPrimAtPath(ROBOT_PATH) if self._stage else None
            if not target or not target.IsValid() or not robot or not robot.IsValid():
                self._set_status("Lab 2 scene is not ready. Click Lab 2 first.")
                return
            sensor_attr = target.GetAttribute("lab:temperatureSensorTarget")
            if (
                (not sensor_attr.IsValid() or not bool(sensor_attr.Get()))
                and self._lab2_prepared
            ):
                sensor_attr = target.CreateAttribute(
                    "lab:temperatureSensorTarget", Sdf.ValueTypeNames.Bool
                )
                sensor_attr.Set(True)
                carb.log_warn(
                    "[Lab 2 Trigger] sensor attribute was false and has been restored "
                    "from the prepared Lab 2 state"
                )
            if not sensor_attr.IsValid() or not bool(sensor_attr.Get()):
                if not self._sensor_unarmed_warning_latched:
                    self._sensor_unarmed_warning_latched = True
                    carb.log_warn(
                        "[Lab 2 Trigger] threshold reached but sensor is not armed"
                    )
                    self._set_status(
                        "Temperature changed, but Lab 2 is not armed. Click Lab 2 first."
                    )
                return
            if not self._play_simulation():
                carb.log_error(
                    "[Lab 2 Trigger] threshold reached but PhysicsScene could not start"
                )
                self._set_status(
                    "습도 조건을 충족했지만 PhysicsScene을 시작하지 못했습니다. "
                    "Lab 1-B를 다시 생성하세요."
                )
                return
            self._set_target_alarm_color(True)
            self._pressed_keys.difference_update({"UP", "DOWN", "LEFT", "RIGHT"})
            self._external_drive_key = None
            self._external_drive_lease_until = 0.0
            if self._external_drive_task is not None and not self._external_drive_task.done():
                self._external_drive_task.cancel()
            self._external_drive_task = None
            self._mission_active = True
            self._mission_latched = True
            self._mission_total_elapsed = 0.0
            self._mission_person_blocked_elapsed = 0.0
            self._transition_mission("TEMPERATURE_ALARM")
            self._temperature_mission_task = asyncio.ensure_future(
                self._run_temperature_mission()
            )
            self._set_status(
                f"AUTO MISSION STARTED: {reason}. "
                "Manual arrow-key control is locked until completion."
            )
            # The dedicated task advances the state on the next Kit frame.

    async def _refresh_temperature_ui_next_update(self):
        try:
            await omni.kit.app.get_app().next_update_async()
            if not self._is_shutting_down:
                self._refresh_mission_ui()
        except asyncio.CancelledError:
            return
        except Exception as exc:
            carb.log_warn(f"[Lab 2 UI] deferred temperature refresh failed: {exc}")

    def _on_temperature_increase(self, *args):
        carb.log_info("[Lab 2 Button] Temperature +5 clicked")
        self._leave_agent_input()
        self._adjust_temperature(5.0)

    def _on_temperature_decrease(self, *args):
        carb.log_info("[Lab 2 Button] Temperature -5 clicked")
        self._leave_agent_input()
        self._adjust_temperature(-5.0)

    def _adjust_temperature(self, delta):
        """Shared UI-button/keyboard simulation input for the sensor boundary."""
        if self._agent_busy or self._agent_action_active:
            self._set_status(
                "OLLAMA ACTION ACTIVE: temperature input is locked until the "
                "current LLM request or robot action finishes."
            )
            carb.log_warn("[Lab 2 Button] temperature input rejected while Ollama action is active")
            return
        before = self._temperature
        self._process_temperature(before + float(delta), "Keyboard Simulation")
        if not self._mission_active:
            target = self._stage.GetPrimAtPath(TARGET_PATH) if self._stage else None
            armed_attr = target.GetAttribute("lab:temperatureSensorTarget") if target and target.IsValid() else None
            armed = bool(armed_attr and armed_attr.IsValid() and armed_attr.Get())
            self._set_status(
                f"TEMPERATURE INPUT: {before:.1f} -> {self._temperature:.1f} °C | "
                f"Lab 2 armed={'YES' if armed else 'NO'}"
            )
        self._refresh_mission_ui()

    def _transition_mission(self, new_state):
        old_state = self._mission_state
        self._mission_state = new_state
        self._state_elapsed = 0.0
        carb.log_info(
            f"[Lab 2 State] {old_state} -> {new_state} "
            f"(push {self._push_count}/2, temperature {self._temperature:.1f} C)"
        )
        messages = {
            "TEMPERATURE_ALARM": ("plan", self._humidity_trigger_description + " 자동 미션을 시작합니다."),
            "TURN_TO_APPROACH": ("action", "계산한 접근점을 향해 방향을 맞춥니다."),
            "ALIGN_TO_BOX": ("action", "접근점에 도착했습니다. box_02를 향해 정렬합니다."),
            "BACK_OFF": ("action", "두 번째 접근을 위해 뒤로 물러나 회전할 공간을 확보합니다."),
            "WAIT_BOX_SETTLE": ("sensor", "상자의 속도를 확인하며 움직임이 안정될 때까지 기다립니다."),
            "COMPLETED": ("done", "두 번 밀기와 최종 후퇴를 완료했습니다. 작업 결과를 확인하세요."),
        }
        if new_state in messages:
            self._narrate(*messages[new_state])
        progress_messages = {
            "DRIVE_TO_APPROACH": "box_02의 최신 위치로 접근 중입니다.",
            "PUSH_1": "첫 번째 밀기 동작을 수행합니다.",
            "PLAN_PUSH_2": "box_02의 이동 위치를 다시 읽고 두 번째 경로를 계산합니다.",
            "PUSH_2": "두 번째 밀기 동작을 수행합니다.",
            "FINAL_BACK_OFF": "작업 완료 후 안전거리까지 후퇴합니다.",
        }
        if new_state in progress_messages:
            self._show_experience_popup("진행", "Lab 2 · Humidity", progress_messages[new_state])
        self._refresh_mission_ui()

    def _update_temperature_mission(self, dt):
        target = self._stage.GetPrimAtPath(TARGET_PATH)
        robot = self._stage.GetPrimAtPath(ROBOT_PATH)
        if not target.IsValid() or not robot.IsValid():
            self._abort_mission("Target or AGV prim disappeared")
            return
        self._state_elapsed += dt
        self._mission_total_elapsed += dt
        self._mission_debug_elapsed += dt
        self._box_current_xy = self._prim_xy(target)
        state = self._mission_state

        # Person avoidance must not leave the automatic mission holding the
        # manual-control lock forever.  Give a moving worker a short window to
        # clear the route; if the AGV remains stopped/turning near the person,
        # abort cleanly and return arrow/button control immediately.
        blocking_modes = {"EMERGENCY_STOP", "STOP_TURN", "MISSION_BLOCK"}
        if self._person_safety_mode in blocking_modes:
            self._mission_person_blocked_elapsed += dt
        else:
            self._mission_person_blocked_elapsed = 0.0
        if self._mission_person_blocked_elapsed >= self.PERSON_MISSION_BLOCK_TIMEOUT_SEC:
            self._abort_mission(
                "worker blocked the AGV route for "
                f"{self._mission_person_blocked_elapsed:.1f}s; manual control unlocked"
            )
            return

        # A mission must never keep manual driving locked indefinitely.  Every
        # individual navigation state already has a timeout; this is the final
        # whole-mission watchdog for any unexpected state-machine stall.
        if self._mission_total_elapsed >= self.MISSION_TOTAL_TIMEOUT_SEC:
            self._abort_mission(
                f"whole mission timeout in {state} "
                f"({self._mission_total_elapsed:.1f}s)"
            )
            return

        # Fixed warehouse safety boundary.  wall_back is at x=11.0; stopping
        # well before it prevents a failed state from driving the box and AGV
        # into the wall indefinitely.
        if (
            state not in {"IDLE", "COMPLETED"}
            and self._box_current_xy[0] >= self.BOX_SAFETY_MAX_X
        ):
            self._command_robot_velocity(0.0, 0.0)
            self._abort_mission(
                f"box_02 reached safety boundary x={self._box_current_xy[0]:.2f}"
            )
            return

        if self._mission_debug_elapsed >= 1.0:
            self._mission_debug_elapsed = 0.0
            robot_xy = self._robot_xy()
            linear, angular = self._rigid_speed(robot)
            carb.log_info(
                f"[Lab 2 Heartbeat] state={state}, playing={self._timeline.is_playing()}, "
                f"robot=({robot_xy[0]:.3f},{robot_xy[1]:.3f}), "
                f"box=({self._box_current_xy[0]:.3f},{self._box_current_xy[1]:.3f}), "
                f"speed=({linear:.3f}m/s,{angular:.3f}deg/s)"
            )

        if state == "TEMPERATURE_ALARM":
            self._transition_mission("PLAN_PUSH_1")
        elif state == "PLAN_PUSH_1":
            # Always use the target's latest World position; never reuse push 1 data.
            self._box_current_xy = self._prim_xy(target)
            self._approach_xy = (
                self._box_current_xy[0] - self.APPROACH_CLEARANCE,
                self._box_current_xy[1],
            )
            carb.log_info(
                f"[Lab 2 Plan] box=({self._box_current_xy[0]:.3f}, "
                f"{self._box_current_xy[1]:.3f}), approach=({self._approach_xy[0]:.3f}, "
                f"{self._approach_xy[1]:.3f})"
            )
            self._narrate("plan", f"box_02 위치 ({self._box_current_xy[0]:.2f}, {self._box_current_xy[1]:.2f})m를 확인했습니다. 첫 번째 접근점 ({self._approach_xy[0]:.2f}, {self._approach_xy[1]:.2f})m를 계산했습니다.")
            self._transition_mission("TURN_TO_APPROACH")
        elif state == "PLAN_PUSH_2":
            # The second plan starts from a freshly read World pose, then the
            # AGV first reverses to create a safe turning/re-approach gap.
            self._box_current_xy = self._prim_xy(target)
            self._approach_xy = (
                self._box_current_xy[0] - self.APPROACH_CLEARANCE,
                self._box_current_xy[1],
            )
            self._narrate("plan", f"상자 위치를 다시 읽었습니다. 새 위치 ({self._box_current_xy[0]:.2f}, {self._box_current_xy[1]:.2f})m 기준 두 번째 접근점은 ({self._approach_xy[0]:.2f}, {self._approach_xy[1]:.2f})m입니다.")
            self._backoff_start_xy = self._robot_xy()
            self._transition_mission("BACK_OFF")
        elif state == "TURN_TO_APPROACH":
            if self._turn_robot_toward(self._approach_xy):
                self._transition_mission("DRIVE_TO_APPROACH")
            elif self._state_elapsed >= self.NAVIGATION_TIMEOUT_SEC:
                self._abort_mission("TURN_TO_APPROACH timeout")
        elif state == "DRIVE_TO_APPROACH":
            # DRIVE_TO_APPROACH is navigation only.  If the AGV gets close
            # enough to touch box_02 before reaching the planned clearance,
            # stop immediately instead of accidentally using navigation speed
            # as an unlimited pushing command.
            robot_box_distance = self._distance_xy(
                self._robot_xy(), self._box_current_xy
            )
            if robot_box_distance <= self.APPROACH_CONTACT_GUARD:
                self._command_robot_velocity(0.0, 0.0)
                self._abort_mission(
                    "unexpected box contact during DRIVE_TO_APPROACH "
                    f"(distance={robot_box_distance:.2f}m)"
                )
                return
            if self._drive_robot_to(self._approach_xy, self.AUTO_LINEAR_SPEED, 0.12):
                self._transition_mission("ALIGN_TO_BOX")
            elif self._avoidance_state == "FAILED":
                self._abort_mission(self._avoidance_failure)
            elif self._state_elapsed >= self.NAVIGATION_TIMEOUT_SEC:
                self._abort_mission("DRIVE_TO_APPROACH timeout")
        elif state == "ALIGN_TO_BOX":
            if self._turn_robot_toward(self._box_current_xy):
                self._push_start_xy = self._robot_xy()
                self._push_box_start_xy = self._prim_xy(target)
                self._push_contact_detected = False
                self._push_contact_elapsed = 0.0
                self._transition_mission("PUSH_1" if self._push_count == 0 else "PUSH_2")
            elif self._state_elapsed >= self.NAVIGATION_TIMEOUT_SEC:
                self._abort_mission("ALIGN_TO_BOX timeout")
        elif state in {"PUSH_1", "PUSH_2"}:
            self._command_robot_velocity(self.AUTO_PUSH_SPEED, 0.0)
            box_xy = self._prim_xy(target)
            box_moved = self._distance_xy(box_xy, self._push_box_start_xy)
            box_linear, _ = self._rigid_speed(target)

            # Treat actual box motion as physical contact.  Measuring only the
            # AGV travel let the velocity command remain active while the AGV
            # and box were already touching, which could push them to a wall.
            if not self._push_contact_detected and (
                box_moved >= self.PUSH_CONTACT_MOVE_EPS
                or box_linear >= self.PUSH_CONTACT_SPEED_EPS
            ):
                self._narrate("sensor", f"box_02의 움직임으로 접촉 반응을 확인했습니다. 변위 {box_moved:.3f}m, 속도 {box_linear:.2f}m/s입니다.")
                self._push_contact_detected = True
                self._push_contact_elapsed = 0.0
                carb.log_info(
                    f"[Lab 2 Contact] {state} box motion detected "
                    f"(moved={box_moved:.3f} m, speed={box_linear:.3f} m/s)"
                )

            # The nominal pulse includes travel across the approach gap.
            # Allow a late contact to produce measurable displacement, but
            # bound the entire attempt independently of contact telemetry.
            robot_travel = self._distance_xy(self._robot_xy(), self._push_start_xy)
            limit_reached = (self._state_elapsed >= self.PUSH_MAX_DURATION_SEC
                             or robot_travel >= self.PUSH_MAX_TRAVEL)
            confirmed = box_moved >= self.PUSH_MIN_BOX_MOVE
            if limit_reached or (self._state_elapsed >= self.PUSH_PULSE_SEC and confirmed):
                self._command_robot_velocity(0.0, 0.0)
                if box_moved < self.PUSH_MIN_BOX_MOVE:
                    self._abort_mission(
                        f"{state} ended without confirmed box displacement "
                        f"(elapsed={self._state_elapsed:.2f}s, robot_travel={robot_travel:.3f}m, "
                        f"box_moved={box_moved:.3f}m, contact={self._push_contact_detected})"
                    )
                    return
                self._push_count += 1
                self._narrate("done", f"{self._push_count}번째 밀기를 완료했습니다. 이번 밀기에서 상자가 {box_moved:.2f}m 이동했습니다.")
                self._settle_elapsed = 0.0
                carb.log_info(
                    f"[Lab 2 Push Stop] push={self._push_count}/2, "
                    f"pulse={self._state_elapsed:.2f}s, "
                    f"box_moved={box_moved:.3f}m"
                )
                self._transition_mission("WAIT_BOX_SETTLE")
        elif state == "WAIT_BOX_SETTLE":
            self._command_robot_velocity(0.0, 0.0)
            linear, angular = self._rigid_speed(target)
            if linear <= self.SETTLE_LINEAR_LIMIT and angular <= self.SETTLE_ANGULAR_LIMIT:
                self._settle_elapsed += dt
            else:
                self._settle_elapsed = 0.0
            required_pause = self.INTER_PUSH_WAIT_SEC if self._push_count == 1 else 0.0
            if (
                self._settle_elapsed >= self.SETTLE_REQUIRED_SEC
                and self._state_elapsed >= required_pause
            ):
                if self._push_count >= 2:
                    self._backoff_start_xy = self._robot_xy()
                    self._transition_mission("FINAL_BACK_OFF")
                else:
                    self._transition_mission("READ_NEW_BOX_POSITION")
            elif self._state_elapsed >= max(required_pause, self.SETTLE_TIMEOUT_SEC):
                # PhysX may report tiny residual box motion forever.  It is not
                # useful motion for this teaching demo, so stop the live actor
                # and continue from its latest measured pose instead of leaving
                # mission_active=True and locking manual control indefinitely.
                try:
                    box_handle = self._dynamic_control.get_rigid_body(str(target.GetPath()))
                    if box_handle != _dynamic_control.INVALID_HANDLE:
                        self._dynamic_control.wake_up_rigid_body(box_handle)
                        self._dynamic_control.set_rigid_body_linear_velocity(
                            box_handle, carb.Float3(0.0, 0.0, 0.0)
                        )
                        self._dynamic_control.set_rigid_body_angular_velocity(
                            box_handle, carb.Float3(0.0, 0.0, 0.0)
                        )
                except Exception as error:
                    carb.log_warn(f"[Lab 2 Settle] runtime zero failed: {error}")
                target_rigid = UsdPhysics.RigidBodyAPI(target)
                target_rigid.CreateVelocityAttr().Set(Gf.Vec3f(0.0))
                target_rigid.CreateAngularVelocityAttr().Set(Gf.Vec3f(0.0))
                carb.log_warn(
                    f"[Lab 2 Settle] residual motion timeout after "
                    f"{self._state_elapsed:.2f}s; continuing from latest box pose"
                )
                if self._push_count >= 2:
                    self._backoff_start_xy = self._robot_xy()
                    self._transition_mission("FINAL_BACK_OFF")
                else:
                    self._transition_mission("READ_NEW_BOX_POSITION")
        elif state == "READ_NEW_BOX_POSITION":
            self._box_current_xy = self._prim_xy(target)
            carb.log_info(
                f"[Lab 2 Replan] new box_02 World position = "
                f"({self._box_current_xy[0]:.3f}, {self._box_current_xy[1]:.3f})"
            )
            self._transition_mission("PLAN_PUSH_2")
        elif state == "BACK_OFF":
            self._command_robot_velocity(-self.AUTO_LINEAR_SPEED, 0.0)
            if self._distance_xy(self._robot_xy(), self._backoff_start_xy) >= self.BACK_OFF_DISTANCE:
                self._command_robot_velocity(0.0, 0.0)
                self._transition_mission("TURN_TO_APPROACH")
            elif self._state_elapsed >= self.BACK_OFF_TIMEOUT_SEC:
                self._abort_mission("BACK_OFF timeout")
        elif state == "FINAL_BACK_OFF":
            # After PUSH_2 and box settling, leave a visible safety gap before
            # declaring the closed-loop task complete.
            self._command_robot_velocity(-self.AUTO_LINEAR_SPEED, 0.0)
            backed_off = self._distance_xy(
                self._robot_xy(), self._backoff_start_xy
            )
            if backed_off >= self.FINAL_BACK_OFF_DISTANCE:
                self._command_robot_velocity(0.0, 0.0)
                self._mission_active = False
                self._transition_mission("COMPLETED")
                self._set_status(
                    "TASK COMPLETED: box_02 was pushed twice, settled, "
                    "and the AGV backed away safely."
                )
                self._complete_experience("box_02를 정확히 두 번 밀고 안전 후퇴까지 완료했습니다.")
            elif self._state_elapsed >= self.BACK_OFF_TIMEOUT_SEC:
                self._abort_mission("FINAL_BACK_OFF timeout")
        else:
            self._abort_mission(f"unknown mission state: {state}")

        self._refresh_mission_ui()

    def _robot_forward_xy(self):
        """Unit forward vector (world XY) of the AGV base's local +X axis.

        Shared by _command_robot_velocity, _turn_robot_toward/_robot_heading and
        the Lab 3 drive_robot/turn_robot agent tools so "forward" always means
        exactly the same direction everywhere it is used.
        """
        robot = self._robot_motion_prim()
        if robot is None:
            return None
        matrix = UsdGeom.Xformable(robot).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        forward = matrix.TransformDir(Gf.Vec3d(1.0, 0.0, 0.0))
        length = forward.GetLength()
        if length > 1e-6:
            forward /= length
        return float(forward[0]), float(forward[1])

    def _robot_heading(self):
        """Current world-frame heading (radians) derived from the forward vector."""
        forward = self._robot_forward_xy()
        if forward is None:
            return 0.0
        return math.atan2(forward[1], forward[0])

    def _set_person_safety_mode(self, mode, distance=None):
        """Report a safety-state transition once instead of every physics frame."""
        if mode == self._person_safety_mode:
            return
        previous = self._person_safety_mode
        self._person_safety_mode = mode
        if mode == "CLEAR":
            self._robot_messages.resolve("person")
            self._narrate("safe", "작업자 접근 제한이 해제되었습니다. 현재 주행 명령을 다시 적용합니다.")
        else:
            descriptions = {
                "SLOW_TURN": "속도를 줄이고 작업자 반대쪽으로 회피합니다.",
                "STOP_TURN": "전진을 제한하고 회피 선회를 적용합니다.",
                "EMERGENCY_STOP": "작업자 쪽 이동을 제한합니다. 저속 회전과 멀어지는 이동만 허용합니다.",
                "MISSION_BLOCK": "작업자 접근으로 밀기 동작을 차단합니다.",
            }
            observed = "확인되지 않음" if distance is None else f"{distance:.2f}m"
            self._narrate("safe", f"작업자와의 XY 중심 거리는 {observed}입니다. " + descriptions.get(mode, mode), alert="person")
        if mode == "CLEAR":
            if previous != "CLEAR":
                carb.log_info("[Person Safety] path clear; requested drive restored")
            return
        distance_text = "--" if distance is None else f"{distance:.2f} m"
        messages = {
            "SLOW_TURN": "PERSON SAFETY: 감속하고 작업자 반대쪽으로 회피합니다.",
            "STOP_TURN": "PERSON SAFETY: 작업자 앞에서 정지·회피 선회합니다.",
            "EMERGENCY_STOP": "PERSON SAFETY: 최소 안전거리—AGV를 즉시 정지했습니다.",
            "MISSION_BLOCK": "PERSON SAFETY: 작업자 접근으로 밀기 동작을 차단했습니다.",
        }
        message = f"{messages.get(mode, 'PERSON SAFETY ACTIVE')} 거리={distance_text}"
        carb.log_warn(f"[Person Safety] {mode}, distance={distance_text}")
        self._set_status(message)

    def _person_radius_adjustment(self, worker):
        scale_attr = worker.GetAttribute("lab:workerScale")
        scale = float(scale_attr.Get()) if scale_attr.IsValid() and scale_attr.Get() is not None else 1.0
        # Editor's basic Worker retains a size-2 cube collider. Custom Worker
        # uses a radius-.28 capsule. Include the cube's corners at any yaw.
        radius = 0.28
        collision = self._stage.GetPrimAtPath(str(worker.GetPath()) + "/CollisionShape")
        if collision.IsValid():
            if collision.GetTypeName() == "Cube":
                size = collision.GetAttribute("size").Get()
                radius = float(size if size is not None else 2.0) / math.sqrt(2.0)
            elif collision.GetTypeName() in ("Capsule", "Cylinder", "Sphere"):
                value = collision.GetAttribute("radius").Get()
                if value is not None:
                    radius = float(value)
        return max(0.0, radius * scale - 0.28)

    def _apply_person_safety_filter(self, linear_speed, angular_speed_deg):
        """Apply one person-safety shield to manual, mission and LLM motion.

        Worker and AGV colliders remain the physical backstop. This predictive
        layer slows and steers before contact, and refuses every motion inside
        the emergency center-to-center distance.
        """
        linear_speed = float(linear_speed)
        angular_speed_deg = float(angular_speed_deg)
        stage = self._stage
        worker = stage.GetPrimAtPath(WORKER_PATH) if stage else None
        robot_xy = self._robot_xy()
        if stage and robot_xy is not None:
            objects = stage.GetPrimAtPath(OBJECT_ROOT)
            people = [p for p in objects.GetChildren() if p.GetName() == "worker_01" or p.GetAttribute("lab:semanticLabel").Get() == "person"] if objects.IsValid() else []
            if people:
                forward = self._robot_forward_xy()
                travel_sign = 1.0 if linear_speed >= 0.0 else -1.0
                def risk(person):
                    xy = self._prim_xy(person)
                    dx, dy = xy[0] - robot_xy[0], xy[1] - robot_xy[1]
                    distance = math.hypot(dx, dy)
                    extra = self._person_radius_adjustment(person)
                    emergency = distance <= self.PERSON_EMERGENCY_DISTANCE + extra
                    if forward is None:
                        return (0 if emergency else 3, distance - extra)
                    along = (dx * forward[0] + dy * forward[1]) * travel_sign
                    across = (forward[0] * dy - forward[1] * dx) * travel_sign
                    in_path = (abs(linear_speed) > 1.0e-4 and along > -0.10
                               and abs(across) <= self.PERSON_PATH_HALF_WIDTH + extra
                               and distance <= self.PERSON_SLOW_DISTANCE + extra)
                    # A nearby person behind/beside the AGV must not mask a
                    # different worker directly in its commanded travel path.
                    priority = (0 if emergency and along > 0 and abs(linear_speed) > 1.0e-4
                                else 1 if in_path else 2 if emergency else 3)
                    return priority, distance - extra
                worker = min(people, key=risk)
        if worker is None or not worker.IsValid() or robot_xy is None:
            self._set_person_safety_mode("CLEAR")
            return linear_speed, angular_speed_deg

        worker_xy = self._prim_xy(worker)
        dx = worker_xy[0] - robot_xy[0]
        dy = worker_xy[1] - robot_xy[1]
        distance = math.hypot(dx, dy)
        radius_adjustment = self._person_radius_adjustment(worker)
        emergency_distance = self.PERSON_EMERGENCY_DISTANCE + radius_adjustment
        stop_distance = self.PERSON_STOP_DISTANCE + radius_adjustment
        slow_distance = self.PERSON_SLOW_DISTANCE + radius_adjustment

        # Inside the emergency radius, never translate toward the person.  The
        # old implementation also blocked every rotation and every escape
        # direction, so an AGV starting 1.2 m from the Worker became trapped
        # forever.  Permit only low-speed rotation and translation whose world
        # direction points away from the Worker.
        if distance <= emergency_distance:
            self._person_avoidance_until = time.monotonic() + self.PERSON_AVOIDANCE_HOLD_SEC
            self._set_person_safety_mode("EMERGENCY_STOP", distance)
            limited_turn = max(
                -self.PERSON_EMERGENCY_TURN_SPEED_DEG,
                min(self.PERSON_EMERGENCY_TURN_SPEED_DEG, angular_speed_deg),
            )
            if abs(linear_speed) <= 1.0e-4:
                return 0.0, limited_turn
            forward = self._robot_forward_xy()
            if forward is None:
                return 0.0, limited_turn
            travel_sign = 1.0 if linear_speed > 0.0 else -1.0
            travel_x = forward[0] * travel_sign
            travel_y = forward[1] * travel_sign
            toward_person = dx * travel_x + dy * travel_y
            if toward_person <= 0.0:
                escape_speed = math.copysign(
                    min(abs(linear_speed), self.PERSON_EMERGENCY_ESCAPE_SPEED),
                    linear_speed,
                )
                self._set_person_safety_mode("EMERGENCY_ESCAPE", distance)
                return escape_speed, limited_turn
            if abs(angular_speed_deg) <= 1.0e-4:
                lateral = travel_x * dy - travel_y * dx
                limited_turn = (-1.0 if lateral >= 0.0 else 1.0) * self.PERSON_EMERGENCY_TURN_SPEED_DEG
            return 0.0, limited_turn

        # A stationary AGV is already safe. Turning in place remains available
        # outside the emergency radius, which lets navigation find a clear path.
        if abs(linear_speed) <= 1.0e-4:
            self._set_person_safety_mode("CLEAR")
            return linear_speed, angular_speed_deg

        forward = self._robot_forward_xy()
        if forward is None:
            return 0.0, 0.0
        travel_sign = 1.0 if linear_speed > 0.0 else -1.0
        travel_x = forward[0] * travel_sign
        travel_y = forward[1] * travel_sign
        longitudinal = dx * travel_x + dy * travel_y
        lateral = travel_x * dy - travel_y * dx
        person_in_path = (
            longitudinal > -0.10
            and abs(lateral) <= self.PERSON_PATH_HALF_WIDTH + radius_adjustment
            and distance <= slow_distance
        )
        if not person_in_path:
            self._set_person_safety_mode("CLEAR")
            return linear_speed, angular_speed_deg

        # Positive angular velocity turns left. Turn away from the side occupied
        # by the worker; when centered, choose right deterministically.
        avoid_sign = -1.0 if lateral >= 0.0 else 1.0
        avoid_turn = avoid_sign * self.PERSON_AVOID_TURN_SPEED_DEG
        self._person_avoidance_until = time.monotonic() + self.PERSON_AVOIDANCE_HOLD_SEC

        # Never continue a box-push pulse while a person occupies its path.
        if self._mission_active and self._mission_state in {"PUSH_1", "PUSH_2"}:
            self._set_person_safety_mode("MISSION_BLOCK", distance)
            return 0.0, 0.0

        if distance <= stop_distance:
            self._set_person_safety_mode("STOP_TURN", distance)
            return 0.0, avoid_turn

        span = slow_distance - stop_distance
        clearance = (distance - stop_distance) / max(span, 1.0e-6)
        clearance = max(0.0, min(1.0, clearance))
        limited_magnitude = min(
            abs(linear_speed),
            0.20 + abs(linear_speed) * 0.55 * clearance,
        )
        blended_turn = avoid_turn * (1.0 - 0.45 * clearance)
        self._set_person_safety_mode("SLOW_TURN", distance)
        return limited_magnitude * travel_sign, blended_turn

    def _command_robot_velocity(self, linear_speed, angular_speed_deg):
        robot = self._robot_motion_prim()
        if robot is None:
            return
        self._last_requested_linear_speed = float(linear_speed)
        self._last_requested_angular_speed_deg = float(angular_speed_deg)
        self._narrate_manual_motion(linear_speed, angular_speed_deg)
        linear_speed, angular_speed_deg = self._apply_person_safety_filter(
            linear_speed, angular_speed_deg
        )
        forward = self._robot_forward_xy()
        if forward is None:
            return
        vx = float(forward[0] * linear_speed)
        vy = float(forward[1] * linear_speed)
        angular_radians = math.radians(float(angular_speed_deg))

        # USD attributes describe authored state, but changing them from a Kit
        # update callback is not a reliable immediate command once the PhysX
        # actor already exists. Dynamic Control writes to that live actor and
        # explicitly wakes it. A handle can be unavailable on the very first
        # frame after Play, so retry lazily on each held-key heartbeat.
        runtime_applied = False
        try:
            if self._timeline.is_playing():
                handle = self._dynamic_robot_handle
                if handle == _dynamic_control.INVALID_HANDLE:
                    handle = self._dynamic_control.get_rigid_body(str(robot.GetPath()))
                    self._dynamic_robot_handle = handle
                if handle != _dynamic_control.INVALID_HANDLE:
                    self._dynamic_control.wake_up_rigid_body(handle)
                    linear_ok = self._dynamic_control.set_rigid_body_linear_velocity(
                        handle, carb.Float3(vx, vy, 0.0)
                    )
                    angular_ok = self._dynamic_control.set_rigid_body_angular_velocity(
                        handle, carb.Float3(0.0, 0.0, angular_radians)
                    )
                    runtime_applied = bool(linear_ok and angular_ok)
                    if runtime_applied:
                        self._dynamic_control_error_signature = None
                        self._last_drive_backend = "PHYSX RUNTIME"
        except Exception as error:
            signature = f"{type(error).__name__}: {error}"
            if signature != self._dynamic_control_error_signature:
                carb.log_warn(f"[AGV Runtime Drive] {signature}")
                self._dynamic_control_error_signature = signature
            self._dynamic_robot_handle = _dynamic_control.INVALID_HANDLE

        # Keep authored velocity as a fallback and as inspectable USD state.
        rigid = UsdPhysics.RigidBodyAPI(robot)
        rigid.CreateVelocityAttr().Set(Gf.Vec3f(vx, vy, 0.0))
        rigid.CreateAngularVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, float(angular_speed_deg)))
        if not runtime_applied:
            self._last_drive_backend = "USD FALLBACK"

    def _turn_robot_toward(self, target_xy):
        robot_xy = self._robot_xy()
        if robot_xy is None:
            return False
        dx, dy = target_xy[0] - robot_xy[0], target_xy[1] - robot_xy[1]
        desired = math.atan2(dy, dx)
        return self._turn_robot_to_heading(desired)

    def _turn_robot_to_heading(self, desired_rad):
        """Angular-velocity-servo the robot toward an absolute world heading.

        Generalizes _turn_robot_toward (which derives its target angle from a
        world-space point) to an explicit angle, so the Lab 3 turn_robot agent
        tool ("turn left 90 degrees") can reuse the exact same servo loop.
        Returns True once the heading is within tolerance and velocity is zeroed.
        """
        if self._robot_motion_prim() is None:
            return False
        current = self._robot_heading()
        error = math.atan2(math.sin(desired_rad - current), math.cos(desired_rad - current))
        if abs(error) <= math.radians(3.0):
            self._command_robot_velocity(0.0, 0.0)
            return True
        speed = self.AUTO_TURN_SPEED_DEG if error > 0.0 else -self.AUTO_TURN_SPEED_DEG
        self._command_robot_velocity(0.0, speed)
        return False

    def _drive_robot_to(self, target_xy, speed, tolerance):
        robot_xy = self._robot_xy()
        if robot_xy is None:
            return False
        distance = self._distance_xy(robot_xy, target_xy)
        if self._navigation.mode != "builtin":
            if distance <= tolerance:
                self._command_robot_velocity(0.0, 0.0)
                self._navigation.cancel()
                self._narrate("done", f"{self._navigation.snapshot()['label']}: 실제 위치를 확인했습니다. 목표까지 {distance:.2f}m로 도착 범위 안입니다.")
                return True
            result, linear, angular, detail = self._navigation.step(
                target_xy, robot_xy, self._robot_heading(), speed, tolerance,
                str(self._extension_root() / "assets/web_control/camera/latest.jpg"))
            if result == "failed":
                self._avoidance_state = "FAILED"
                self._avoidance_failure = detail
                self._command_robot_velocity(0.0, 0.0)
                self._narrate("error", detail + " 기존 주행으로 자동 전환하지 않고 정지합니다.", key="navigation_failure", interval=5, alert="navigation")
                self._navigation.cancel()
            else:
                self._avoidance_state = "EXTERNAL"
                self._command_robot_velocity(linear, angular)
                self._narrate("action", self._navigation.snapshot()['label'] + ": " + (detail or "목표점으로 이동 명령을 적용합니다."), key="navigation_motion", interval=5)
            return False
        if distance <= tolerance:
            self._command_robot_velocity(0.0, 0.0)
            self._reset_obstacle_avoidance()
            return True

        target_key = (round(float(target_xy[0]), 2), round(float(target_xy[1]), 2))
        if self._avoidance_target != target_key:
            self._reset_obstacle_avoidance(target_key, robot_xy)

        if self._avoidance_state == "FAILED":
            self._command_robot_velocity(0.0, 0.0)
            return False
        if self._avoidance_state == "BACK_OFF_0.8M":
            backed = self._distance_xy(robot_xy, self._avoidance_phase_start_xy)
            if backed < self.OBSTACLE_BACKOFF_DISTANCE:
                self._command_robot_velocity(-min(abs(float(speed)), self.AUTO_LINEAR_SPEED), 0.0)
                return False
            self._command_robot_velocity(0.0, 0.0)
            self._avoidance_waypoint, side = self._choose_bypass_waypoint(robot_xy, target_xy)
            self._avoidance_state = f"BYPASS_{side}"
            self._show_experience_popup(
                "장애물 우회",
                f"재시도 {self._avoidance_retry_count}/{self.OBSTACLE_MAX_RETRIES}",
                f"0.8m 후퇴 완료 · {side} 우회 경로 선택 · 목표를 다시 계산합니다.",
            )
            return False
        if self._avoidance_state.startswith("BYPASS_"):
            if self._drive_robot_to_raw(self._avoidance_waypoint, min(abs(float(speed)), 0.75), 0.16):
                self._avoidance_state = "RECALCULATE_TARGET"
                self._avoidance_last_xy = self._robot_xy()
                self._avoidance_last_progress_at = time.monotonic()
                self._show_experience_popup(
                    "목표 재계산", self._experience_name or "AGV Navigation",
                    "우회 지점에 도착했습니다. 현재 위치에서 원래 목표까지 경로를 다시 계산합니다.",
                )
            return False
        if self._avoidance_state == "RECALCULATE_TARGET":
            self._avoidance_state = "MONITORING"

        # After the shield selects an avoidance side, keep the evasive arc for
        # a short time. Without this hold, the goal-heading servo would turn
        # back toward the blocked path on the very next frame.
        if time.monotonic() < self._person_avoidance_until:
            avoidance_speed = math.copysign(
                min(abs(float(speed)), 0.38), float(speed)
            )
            self._command_robot_velocity(avoidance_speed, 0.0)
            return False

        # Closed-loop navigation: recalculate the desired bearing every frame.
        # The old implementation issued forward speed without correcting yaw;
        # after a small collision deflection it could drive away from the
        # approach point and continuously push the box while still reporting
        # DRIVE_TO_APPROACH.
        desired = math.atan2(
            target_xy[1] - robot_xy[1], target_xy[0] - robot_xy[0]
        )
        current = self._robot_heading()
        heading_error = math.atan2(
            math.sin(desired - current), math.cos(desired - current)
        )
        if abs(heading_error) > math.radians(5.0):
            turn_speed = (
                self.AUTO_TURN_SPEED_DEG
                if heading_error > 0.0
                else -self.AUTO_TURN_SPEED_DEG
            )
            # Turning in place is intentional navigation, not a linear stall.
            # Refresh the progress baseline so a long heading correction does
            # not immediately consume the next obstacle retry.
            self._avoidance_last_xy = robot_xy
            self._avoidance_last_progress_at = time.monotonic()
            self._command_robot_velocity(0.0, turn_speed)
            return False

        now = time.monotonic()
        if self._avoidance_last_xy is None:
            self._avoidance_last_xy = robot_xy
            self._avoidance_last_progress_at = now
        elif self._distance_xy(robot_xy, self._avoidance_last_xy) >= self.OBSTACLE_PROGRESS_EPS:
            self._avoidance_last_xy = robot_xy
            self._avoidance_last_progress_at = now
        elif now - self._avoidance_last_progress_at >= self.OBSTACLE_STALL_SEC:
            if self._avoidance_retry_count >= self.OBSTACLE_MAX_RETRIES:
                self._avoidance_state = "FAILED"
                self._avoidance_failure = "장애물 우회 최대 3회 재시도 실패"
                self._command_robot_velocity(0.0, 0.0)
                self._show_experience_popup("우회 실패", "AGV Navigation", self._avoidance_failure)
                return False
            self._avoidance_retry_count += 1
            self._avoidance_state = "BACK_OFF_0.8M"
            self._avoidance_phase_start_xy = robot_xy
            self._command_robot_velocity(-min(abs(float(speed)), self.AUTO_LINEAR_SPEED), 0.0)
            self._show_experience_popup(
                "경로 막힘 감지", "AGV Navigation",
                f"진행이 멈췄습니다. 0.8m 후퇴 후 좌우 우회 경로를 선택합니다. "
                f"재시도 {self._avoidance_retry_count}/{self.OBSTACLE_MAX_RETRIES}",
            )
            return False

        commanded_speed = min(float(speed), max(0.25, distance * self.AUTO_APPROACH_GAIN))
        self._command_robot_velocity(commanded_speed, 0.0)
        return False

    def _drive_robot_to_raw(self, target_xy, speed, tolerance):
        """Closed-loop drive without nested stall detection, used for bypass legs."""
        robot_xy = self._robot_xy()
        if robot_xy is None:
            return False
        distance = self._distance_xy(robot_xy, target_xy)
        if distance <= tolerance:
            self._command_robot_velocity(0.0, 0.0)
            return True
        desired = math.atan2(target_xy[1] - robot_xy[1], target_xy[0] - robot_xy[0])
        if speed < 0.0:
            desired += math.pi
        error = math.atan2(math.sin(desired - self._robot_heading()), math.cos(desired - self._robot_heading()))
        if abs(error) > math.radians(5.0):
            self._command_robot_velocity(0.0, self.AUTO_TURN_SPEED_DEG if error > 0.0 else -self.AUTO_TURN_SPEED_DEG)
        else:
            magnitude = min(abs(float(speed)), max(0.25, distance * self.AUTO_APPROACH_GAIN))
            self._command_robot_velocity(math.copysign(magnitude, float(speed)), 0.0)
        return False

    def _choose_bypass_waypoint(self, robot_xy, target_xy):
        dx, dy = target_xy[0] - robot_xy[0], target_xy[1] - robot_xy[1]
        length = max(math.hypot(dx, dy), 1.0e-6)
        left = (-dy / length, dx / length)
        forward = (dx / length, dy / length)
        candidates = {
            "LEFT": (robot_xy[0] + forward[0] * 0.8 + left[0] * self.OBSTACLE_BYPASS_LATERAL,
                     robot_xy[1] + forward[1] * 0.8 + left[1] * self.OBSTACLE_BYPASS_LATERAL),
            "RIGHT": (robot_xy[0] + forward[0] * 0.8 - left[0] * self.OBSTACLE_BYPASS_LATERAL,
                      robot_xy[1] + forward[1] * 0.8 - left[1] * self.OBSTACLE_BYPASS_LATERAL),
        }
        obstacles = []
        root = self._stage.GetPrimAtPath(OBJECT_ROOT) if self._stage else None
        if root and root.IsValid():
            for prim in root.GetChildren():
                if prim.IsValid() and str(prim.GetPath()) not in {ROBOT_PATH, TARGET_PATH}:
                    try:
                        obstacles.append(self._prim_xy(prim))
                    except Exception:
                        pass
        def clearance(point):
            return min((self._distance_xy(point, obstacle) for obstacle in obstacles), default=999.0)
        side = max(candidates, key=lambda name: (clearance(candidates[name]), name == ("LEFT" if self._avoidance_retry_count % 2 else "RIGHT")))
        return candidates[side], side

    def _reset_obstacle_avoidance(self, target=None, robot_xy=None):
        self._navigation.cancel()
        self._avoidance_state = "IDLE" if target is None else "MONITORING"
        self._avoidance_retry_count = 0
        self._avoidance_target = target
        self._avoidance_last_xy = robot_xy
        self._avoidance_last_progress_at = time.monotonic()
        self._avoidance_phase_start_xy = None
        self._avoidance_waypoint = None
        self._avoidance_failure = ""

    def _rigid_speed(self, prim):
        rigid = UsdPhysics.RigidBodyAPI(prim)
        velocity = rigid.GetVelocityAttr().Get() if rigid else None
        angular = rigid.GetAngularVelocityAttr().Get() if rigid else None
        linear_speed = math.sqrt(sum(float(v) ** 2 for v in velocity)) if velocity is not None else 0.0
        angular_speed = math.sqrt(sum(float(v) ** 2 for v in angular)) if angular is not None else 0.0
        return linear_speed, angular_speed

    def _prim_xy(self, prim):
        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        position = matrix.ExtractTranslation()
        return float(position[0]), float(position[1])

    @staticmethod
    def _distance_xy(a, b):
        if a is None or b is None:
            return 0.0
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def _set_target_alarm_color(self, enabled):
        target = self._stage.GetPrimAtPath(TARGET_PATH) if self._stage else None
        if not target or not target.IsValid():
            return
        color = self.ALARM_COLOR if enabled else self._box_original_color
        if color is None:
            return
        visual_root = self._stage.GetPrimAtPath(f"{TARGET_PATH}/ModelVisuals")
        if visual_root.IsValid():
            for child in Usd.PrimRange(visual_root):
                if child.GetAttribute("lab:tintable").Get():
                    attr = UsdGeom.Gprim(child).GetDisplayColorAttr()
                    if attr.IsValid():
                        attr.Set([color])

    def _refresh_mission_ui(self):
        summary = (
            f"Temperature: {self._temperature:.1f} °C | "
            f"Threshold: {self._temperature_threshold:.1f} °C"
        )
        if self._temperature_control_model is not None:
            self._temperature_control_model.set_value(summary)
        if self._humidity_readout_label is not None:
            change = "--" if self._ui_humidity_delta is None else f"{self._ui_humidity_delta:+.1f} pp"
            usb = "--" if self._hardware_humidity is None else f"{self._hardware_humidity:.1f}%RH"
            usb_change = "--" if self._hardware_humidity_delta is None else f"{self._hardware_humidity_delta:+.1f} pp"
            stale = self._hardware_sample_timestamp is None or time.monotonic() - self._hardware_sample_timestamp > 3.0 or bool(self._hardware_sensor_error)
            self._humidity_readout_label.text = f"Humidity button: {self._humidity:.1f}%RH | change {change}\nUSB: {usb} | change {usb_change}" + (" (no current reading)" if stale else "")
        if not self._mission_labels:
            carb.log_warn("[Lab 2 UI] mission status fields are unavailable; control-panel value updated")
            return
        initial = self._box_initial_xy
        current = self._box_current_xy
        moved = self._distance_xy(current, initial)
        target = self._stage.GetPrimAtPath(TARGET_PATH) if self._stage else None
        armed_attr = target.GetAttribute("lab:temperatureSensorTarget") if target and target.IsValid() else None
        armed = bool(
            self._lab2_prepared
            and armed_attr and armed_attr.IsValid() and armed_attr.Get()
        )
        values = {
            "temperature": f"Temperature: {self._temperature:.1f} °C",
            "threshold": f"Threshold: {self._temperature_threshold:.1f} °C",
            "sensor": f"Sensor Mode: {self._sensor_mode}",
            "target": "Target: box_02",
            "state": f"AGV State: {self._mission_state}",
            "push": f"Push Count: {self._push_count} / 2",
            "initial": "Box Initial: --" if initial is None else f"Box Initial: ({initial[0]:.2f}, {initial[1]:.2f})",
            "current": "Box Current: --" if current is None else f"Box Current: ({current[0]:.2f}, {current[1]:.2f})",
            "moved": f"Moved: {moved:.2f} m",
            "mission": "Mission: COMPLETED" if self._mission_state == "COMPLETED" else ("Mission: ACTIVE" if self._mission_active else ("Mission: READY (ARMED)" if armed else "Mission: NOT ARMED")),
        }
        for key, value in values.items():
            model = self._mission_models.get(key)
            if model is not None:
                model.set_value(value)

    def _abort_mission(self, reason):
        self._mission_active = False
        self._mission_person_blocked_elapsed = 0.0
        self._person_avoidance_until = 0.0
        self._command_robot_velocity(0.0, 0.0)
        carb.log_error(f"[Lab 2 State] mission aborted: {reason}")
        self._set_status(f"Humidity mission aborted: {reason}")
        self._narrate("error", f"습도 미션을 중단하고 정지 명령을 보냈습니다. 원인: {reason}", alert="mission")
        self._refresh_mission_ui()

    def _zero_robot_velocities(self):
        robot = self._robot_motion_prim()
        if robot is None:
            return
        rigid = UsdPhysics.RigidBodyAPI(robot)
        rigid.CreateVelocityAttr().Set(Gf.Vec3f(0.0))
        rigid.CreateAngularVelocityAttr().Set(Gf.Vec3f(0.0))

    def _reset_temperature_mission(self):
        self._navigation.cancel()
        self._robot_messages.clear()
        self._narrate("done", "습도 체험 기록을 정리하고 객체·속도·센서 기준을 초기화합니다.")
        self._agent_motion_epoch += 1
        self._agent_action_active = False
        if self._external_drive_task is not None and not self._external_drive_task.done():
            self._external_drive_task.cancel()
        self._external_drive_task = None
        if self._temperature_mission_task is not None and not self._temperature_mission_task.done():
            self._temperature_mission_task.cancel()
        self._temperature_mission_task = None
        self._person_safety_mode = "CLEAR"
        self._person_avoidance_until = 0.0
        self._mission_person_blocked_elapsed = 0.0
        self._pressed_keys.clear()
        self._external_drive_key = None
        self._external_drive_lease_until = 0.0
        self._timeline.stop()
        for path in (TARGET_PATH, ROBOT_PATH):
            prim = self._stage.GetPrimAtPath(path) if self._stage else None
            if prim and prim.IsValid():
                self._reset_single_prim(prim)
        self._zero_robot_velocities()
        self._robot_motion_prim_path = None
        self._mobile_base_drive_paths = None
        self._set_target_alarm_color(False)
        self._initialize_mission_from_stage(armed=True)
        self._set_status("Temperature mission reset: temperature, color, poses, velocities and state cleared.")

    def _launch_pusher(self):
        stage = self._stage
        pusher = stage.GetPrimAtPath(PUSHER_PATH)
        if not pusher.IsValid():
            self._set_status("먼저 Demo Scene을 생성하세요.")
            return

        # After impact, keep the Pusher where physics leaves it.  Only the next
        # explicit Launch/Relaunch action returns it to the start position.
        if self._pusher_has_launched:
            self._reset_single_prim(pusher)
        self._apply_mobility(pusher, "movable")
        rigid = UsdPhysics.RigidBodyAPI(pusher)
        rigid.CreateVelocityAttr().Set(Gf.Vec3f(4.0, 0.0, 0.0))
        rigid.CreateAngularVelocityAttr().Set(Gf.Vec3f(0.0))
        self._pusher_has_launched = True
        self._timeline.play()
        self._set_status(
            "Pusher 발사: +X 방향 4 m/s\n"
            "충돌 후 자동 복귀하지 않습니다. 다음 Launch에서 복귀 후 재발사합니다."
        )

    def _stop_robot(self):
        self._navigation.cancel()
        self._agent_motion_epoch += 1
        self._agent_action_active = False
        mission_was_active = self._mission_active
        self._mission_active = False
        self._mission_person_blocked_elapsed = 0.0
        self._person_avoidance_until = 0.0
        if self._temperature_mission_task is not None and not self._temperature_mission_task.done():
            self._temperature_mission_task.cancel()
        self._temperature_mission_task = None
        if self._external_drive_task is not None and not self._external_drive_task.done():
            self._external_drive_task.cancel()
        self._external_drive_task = None
        self._pressed_keys.clear()
        self._external_drive_key = None
        self._external_drive_lease_until = 0.0
        self._command_robot_velocity(0.0, 0.0)
        self._zero_robot_velocities()
        if mission_was_active:
            self._transition_mission("CANCELLED")
            self._set_status(
                "온도 미션과 AGV를 정지했습니다. 수동 키·버튼 제어가 다시 활성화되었습니다."
            )
        else:
            self._set_status("이동로봇 정지")

    def _reset_robot(self):
        if self._current_busy_operation() and not (self._mission_active or self._agent_action_active):
            self._set_status("로봇 초기화 거절: 진행 중인 작업이 끝난 뒤 다시 시도하세요.")
            return
        self._stop_robot()
        self._navigation.cancel()
        self._agent_motion_epoch += 1
        self._agent_action_active = False
        if self._external_drive_task is not None and not self._external_drive_task.done():
            self._external_drive_task.cancel()
        self._external_drive_task = None
        self._person_safety_mode = "CLEAR"
        self._person_avoidance_until = 0.0
        self._pressed_keys.clear()
        self._external_drive_key = None
        self._external_drive_lease_until = 0.0
        self._timeline.pause()
        robot = self._stage.GetPrimAtPath(ROBOT_PATH) if self._stage else None
        if robot is None or not robot.IsValid():
            self._set_status("로봇 초기화 실패: AGV가 없습니다. 먼저 AGV 추가 버튼을 누르세요.")
            return
        self._reset_single_prim(robot)
        self._zero_robot_velocities()
        self._robot_motion_prim_path = None
        self._mobile_base_drive_paths = None
        self._robot_yaw_deg = 0.0
        self._set_status("이동로봇을 초기 위치로 복원했습니다.")

    def _reset_objects(self):
        self._agent_motion_epoch += 1
        self._agent_action_active = False
        root = self._stage.GetPrimAtPath(LAB_ROOT)
        if not root.IsValid():
            self._set_status("먼저 Demo Scene을 생성하세요.")
            return
        self._timeline.stop()
        for prim in Usd.PrimRange(root):
            if prim.GetAttribute("lab:initialPosition").IsValid():
                self._reset_single_prim(prim)
        self._pusher_has_launched = False
        self._zero_robot_velocities()
        self._robot_motion_prim_path = None
        self._mobile_base_drive_paths = None
        self._set_target_alarm_color(False)
        armed_attr = self._stage.GetPrimAtPath(TARGET_PATH).GetAttribute("lab:temperatureSensorTarget")
        armed = bool(armed_attr.Get()) if armed_attr.IsValid() else False
        self._initialize_mission_from_stage(armed=armed)
        self._set_status("객체 위치와 속도를 초기화했습니다. Play 또는 P를 누르세요.")

    def _reset_single_prim(self, prim):
        xform = UsdGeom.Xformable(prim)
        translate_op = self._find_xform_op(xform, UsdGeom.XformOp.TypeTranslate)
        orient_op = self._find_xform_op(xform, UsdGeom.XformOp.TypeOrient)
        initial = prim.GetAttribute("lab:initialPosition").Get()
        initial_orientation = prim.GetAttribute("lab:initialOrientation").Get()
        if translate_op is not None and initial is not None:
            translate_op.Set(initial)
        if orient_op is not None and initial_orientation is not None:
            orient_op.Set(initial_orientation)
        rigid = UsdPhysics.RigidBodyAPI(prim)
        if rigid:
            rigid.CreateVelocityAttr().Set(Gf.Vec3f(0.0))
            rigid.CreateAngularVelocityAttr().Set(Gf.Vec3f(0.0))

    def _safe_edit(self, fn):
        was_playing = self._timeline.is_playing()
        self._timeline.pause()
        fn()
        if was_playing:
            self._timeline.play()

    def _report_selected(self, prim, action):
        mobility = prim.GetAttribute("lab:mobility").Get() or "unknown"
        mass = prim.GetAttribute("lab:massKg").Get() or 0.0
        xform = UsdGeom.Xformable(prim)
        scale_op = self._find_xform_op(xform, UsdGeom.XformOp.TypeScale)
        scale = scale_op.Get() if scale_op else Gf.Vec3f(1.0)
        self._set_status(
            f"{action}\n\n"
            f"Selected: {prim.GetName()}\n"
            f"Mobility: {mobility}\n"
            f"Mass: {float(mass):g} kg\n"
            f"Scale: ({scale[0]:.2f}, {scale[1]:.2f}, {scale[2]:.2f})"
        )

    def _json_path(self):
        output = self._extension_root() / "data"
        output.mkdir(parents=True, exist_ok=True)
        return output / "lab_objects.json"

    def _extension_root(self):
        return Path(
            omni.kit.app.get_app().get_extension_manager().get_extension_path(self._ext_id)
        )

    def _save_json(self):
        root = self._stage.GetPrimAtPath(OBJECT_ROOT)
        if not root.IsValid():
            self._set_status("저장할 Demo Scene이 없습니다.")
            return
        objects = []
        for prim in root.GetChildren():
            xform = UsdGeom.Xformable(prim)
            translate = self._find_xform_op(xform, UsdGeom.XformOp.TypeTranslate)
            scale = self._find_xform_op(xform, UsdGeom.XformOp.TypeScale)
            objects.append(
                {
                    "path": str(prim.GetPath()),
                    "mobility": prim.GetAttribute("lab:mobility").Get(),
                    "mass_kg": float(prim.GetAttribute("lab:massKg").Get() or 0.0),
                    "position": list(translate.Get()) if translate else [0.0, 0.0, 0.0],
                    "scale": list(scale.Get()) if scale else [1.0, 1.0, 1.0],
                }
            )
        path = self._json_path()
        path.write_text(json.dumps({"objects": objects}, indent=2), encoding="utf-8")
        self._set_status(f"JSON 저장 완료\n{path}")

    def _reload_json(self):
        path = self._json_path()
        if not path.exists():
            self._set_status("저장된 JSON이 없습니다. 먼저 S를 눌러 저장하세요.")
            return
        data = json.loads(path.read_text(encoding="utf-8"))
        self._timeline.pause()
        for item in data.get("objects", []):
            prim = self._stage.GetPrimAtPath(item["path"])
            if not prim.IsValid():
                continue
            self._apply_mobility(prim, item["mobility"])
            self._set_mass_attr(prim, item["mass_kg"])
            xform = UsdGeom.Xformable(prim)
            translate = self._find_xform_op(xform, UsdGeom.XformOp.TypeTranslate)
            scale = self._find_xform_op(xform, UsdGeom.XformOp.TypeScale)
            if translate:
                translate.Set(Gf.Vec3d(*item["position"]))
            if scale:
                scale.Set(Gf.Vec3f(*item["scale"]))
        self._set_status(f"JSON 불러오기 완료\n{path}")

    # ------------------------------------------------------------------
    # Lab 3: conversational control (local LLM via Ollama)
    # ------------------------------------------------------------------
    #
    # Design: the LLM (OllamaAgent, in ollama_agent.py) never touches the USD
    # stage. It only sees a JSON world_state snapshot and a list of tool
    # schemas, and asks to call tools by name. Every tool call is dispatched
    # through _agent_tool_executor below, which is the single place that (a)
    # enforces mutual exclusion with the temperature mission and with manual
    # driving, (b) clamps every numeric argument into a safe range, and (c)
    # actually reuses the same helpers the manual UI/keyboard controls use
    # (_apply_mobility, _set_mass_attr, _apply_color_index, _drive_robot_to,
    # ...) so a natural-language command can never do anything the buttons
    # and keys could not already do.

    def _on_agent_send(self):
        carb.log_info("[Ollama Prompt] Run Prompt callback received")
        self._set_agent_reply("SEND RECEIVED")
        if self._agent_field is None or self._agent_input_model is None:
            self._set_agent_reply("INPUT FIELD UNAVAILABLE")
            return
        model = self._agent_input_model
        text = model.get_value_as_string().strip()
        if not text:
            self._set_agent_reply("INPUT IS EMPTY")
            return
        if self._agent_busy or (
            self._agent_turn_task is not None and not self._agent_turn_task.done()
        ):
            self._set_agent_reply("이전 요청을 아직 처리 중입니다. 잠시 기다려주세요.")
            return
        model.set_value("")
        self._agent_input_active = False
        self._submit_agent_prompt(text)

    def _submit_agent_prompt(self, text):
        """Submit browser text without a native omni.ui input model."""
        text = str(text or "").strip()[:2000]
        if not text:
            self._set_status("EXTERNAL CONTROL: Ollama 명령을 입력하세요.")
            return False
        if self._agent_busy or (
            self._agent_turn_task is not None and not self._agent_turn_task.done()
        ):
            self._set_agent_reply("이전 요청을 아직 처리 중입니다. 잠시 기다려주세요.")
            return False
        self._narrate("plan", f"자연어 요청을 받았습니다: “{text}”. 등록 도구로 처리할 수 있는지 모델에 요청합니다.")
        self._agent_input_active = False
        self._agent_turn_task = asyncio.ensure_future(self._run_agent_turn_async(text))
        return True

    def _focus_agent_input(self):
        """Give the prompt field keyboard ownership over global robot keys."""
        if self._agent_field is None:
            return
        self._pressed_keys.clear()
        self._external_drive_key = None
        self._external_drive_lease_until = 0.0
        self._command_robot_velocity(0.0, 0.0)
        self._agent_input_active = True
        try:
            focus_keyboard = getattr(self._agent_field, "focus_keyboard", None)
            if callable(focus_keyboard):
                focus_keyboard()
            else:
                self._agent_field.focused = True
        except Exception as error:
            carb.log_warn(f"[Ollama Prompt] explicit focus unavailable: {error}")
        self._set_status("PROMPT INPUT ACTIVE: type a command, then press Enter or Send.")

    def _leave_agent_input(self):
        """Release prompt focus and restore global manual driving keys."""
        self._agent_input_active = False
        try:
            if self._agent_field is not None:
                self._agent_field.focused = False
        except Exception as error:
            carb.log_warn(f"[Ollama Prompt] focus release unavailable: {error}")
        self._set_status("MANUAL KEYS ACTIVE: arrow-key control restored.")

    async def _run_agent_turn_async(self, text):
        if self._agent_busy:
            self._set_agent_reply("이전 요청을 아직 처리 중입니다. 잠시 기다려주세요.")
            return
        self._agent_busy = True
        self._set_agent_reply(f"OLLAMA REQUEST [{self._agent.model}]: {text}")
        carb.log_info(
            f"[Ollama LLM Request] host={self._agent.host}, "
            f"model={self._agent.model}, prompt={text!r}"
        )
        try:
            agent = self._agent
            if agent is None or self._is_shutting_down:
                return
            world_state = self._agent_world_snapshot()
            reply = await agent.run_turn(text, world_state, self._agent_tool_executor)
        except asyncio.CancelledError:
            self._command_robot_velocity(0.0, 0.0)
            raise
        except Exception as error:  # noqa: BLE001 - keep the UI alive even if the agent crashes
            carb.log_error(f"[Interactive Physics Lab] Agent turn failed: {error}")
            reply = f"오류가 발생했습니다: {type(error).__name__}: {error}"
        finally:
            self._agent_busy = False
        if not self._is_shutting_down and self._agent is not None:
            self._set_agent_reply(f"OLLAMA RESPONSE: {reply}")
            # Final replies (including parser refusals) belong in the shared message history.
            failed = any(word in str(reply) for word in ("미완료", "실행 거절", "실행하지", "오류가 발생"))
            self._robot_messages.emit("error" if failed else "info", str(reply), alert="agent" if failed else None)
            carb.log_info(f"[Ollama LLM Response] {reply}")

    def _set_agent_reply(self, text):
        carb.log_info(f"[Interactive Physics Lab][Agent] {text}")
        self._last_status_text = str(text)
        if self._agent_reply_model is not None:
            self._agent_reply_model.set_value(str(text))

    def _agent_world_snapshot(self):
        """JSON-serializable snapshot of everything the LLM needs to reason about."""
        stage = self._stage
        objects = []
        if stage is not None:
            root = stage.GetPrimAtPath(OBJECT_ROOT)
            if root.IsValid():
                for child in root.GetChildren():
                    objects.append({
                        "object_id": child.GetName(),
                        "object_type": child.GetAttribute("lab:objectType").Get() or "unknown",
                        "mobility": child.GetAttribute("lab:mobility").Get(),
                        "mass_kg": round(float(child.GetAttribute("lab:massKg").Get() or 0.0), 2),
                        "position_xy": [round(v, 2) for v in self._prim_xy(child)],
                    })

        robot_installed = self._agv_is_installed()
        robot_info = None
        if robot_installed:
            robot_xy = self._robot_xy()
            robot_info = {
                "position_xy": [round(v, 2) for v in robot_xy] if robot_xy else None,
                "heading_deg": round(math.degrees(self._robot_heading()), 1),
                "arm_pose": self._last_arm_pose,
                "gripper": "open" if self._gripper_open else "closed",
                "camera_path": self._robot_camera_path,
                "camera_view_active": self._is_robot_camera_view_active(),
            }

        selection = []
        if stage is not None:
            selection = list(omni.usd.get_context().get_selection().get_selected_prim_paths())

        return {
            "objects": objects,
            "selected_prim_paths": selection,
            "robot_installed": robot_installed,
            "robot": robot_info,
            "temperature_mission": {
                "temperature_c": round(self._temperature, 1),
                "threshold_c": self._temperature_threshold,
                "state": self._mission_state,
                "mission_active": self._mission_active,
                "push_count": self._push_count,
            },
        }

    _AGENT_NON_PHYSICAL_TOOLS = {"get_status", "set_view"}

    async def _agent_tool_executor(self, name, args):
        self._narrate("plan", f"등록 도구 요청을 받았습니다: {name}. 대상과 입력값을 확인합니다.")
        try:
            result = await self._execute_agent_tool(name, args)
        except Exception as error:
            self._narrate("error", f"{name} 실행 중 오류가 발생했습니다: {error}", alert="agent")
            raise
        if result.get("ok"):
            self._robot_messages.resolve("agent")
            self._narrate("done", str(result.get("message") or f"{name} 실행 결과를 확인했습니다."))
        else:
            self._narrate("error", str(result.get("error") or f"{name} 실행에 실패했습니다."), alert="agent")
        return result

    async def _execute_agent_tool(self, name, args):
        """Single dispatch/safety gate for every tool call the LLM makes."""
        carb.log_info(f"[Ollama Tool Call] name={name}, args={args}")
        self._set_agent_reply(f"OLLAMA TOOL CALL: {name} {args}")
        if self._is_shutting_down or self._agent is None:
            return {"ok": False, "error": "Extension이 종료 중입니다."}
        if self._stage is None:
            return {"ok": False, "error": "USD Stage가 없습니다. 먼저 Lab을 실행하세요."}
        if not isinstance(args, dict):
            return {"ok": False, "error": "도구 인자는 JSON 객체여야 합니다."}

        if name not in self._AGENT_NON_PHYSICAL_TOOLS:
            if self._mission_active:
                return {
                    "ok": False,
                    "error": "자동 온도 미션이 진행 중입니다. 끝나거나 Reset Temperature "
                             "Mission 후 다시 시도하세요.",
                }
            if self._agent_action_active and name != "stop_robot":
                return {
                    "ok": False,
                    "error": "이전 대화형 동작이 아직 실행 중입니다. 잠시 후 다시 시도하세요.",
                }

        if name == "drive_robot":
            direction = str(args.get("direction", "")).lower()
            if direction not in ("forward", "backward"):
                return {"ok": False, "error": "direction은 forward/backward 중 하나여야 합니다."}
            return await self._agent_drive_distance(direction, args.get("distance_m", 1.0))
        if name == "turn_robot":
            direction = str(args.get("direction", "")).lower()
            if direction not in ("left", "right"):
                return {"ok": False, "error": "direction은 left/right 중 하나여야 합니다."}
            return await self._agent_turn_degrees(direction, args.get("degrees", 45.0))

        handlers = {
            "stop_robot": self._agent_tool_stop,
            "select_object": self._agent_tool_select,
            "set_mobility": self._agent_tool_set_mobility,
            "set_mass": self._agent_tool_set_mass,
            "set_color": self._agent_tool_set_color,
            "set_model": self._agent_tool_set_model,
            "scale_object": self._agent_tool_scale,
            "set_arm_pose": self._agent_tool_set_arm_pose,
            "set_gripper": self._agent_tool_set_gripper,
            "launch_pusher": self._agent_tool_launch_pusher,
            "reset_objects": self._agent_tool_reset_objects,
            "reset_robot": self._agent_tool_reset_robot,
            "reset_temperature_mission": self._agent_tool_reset_mission,
            "set_view": self._agent_tool_set_view,
            "get_status": self._agent_tool_get_status,
        }
        handler = handlers.get(name)
        if handler is None:
            return {"ok": False, "error": f"알 수 없는 도구: {name}"}
        return handler(args)

    def _agent_find_object(self, object_id):
        if not object_id:
            return None, {"ok": False, "error": "object_id가 필요합니다."}
        object_id = str(object_id).strip()
        if not object_id or "/" in object_id or "\\" in object_id or object_id in {".", ".."}:
            return None, {"ok": False, "error": "object_id 형식이 올바르지 않습니다."}
        root = self._stage.GetPrimAtPath(OBJECT_ROOT)
        prim = next((child for child in root.GetChildren() if child.GetName() == object_id), None) if root.IsValid() else None
        if prim is None or not prim.IsValid():
            return None, {"ok": False, "error": f"'{object_id}' 객체를 찾을 수 없습니다."}
        self._narrate("sensor", f"대상 객체 {prim.GetName()}을 찾았습니다. 현재 질량은 {float(prim.GetAttribute('lab:massKg').Get() or 0):g}kg입니다.")
        return prim, None

    @staticmethod
    def _agent_finite_float(value, field_name, minimum, maximum, absolute=False):
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None, f"{field_name} 값이 올바르지 않습니다."
        if not math.isfinite(number):
            return None, f"{field_name}에는 유한한 숫자만 사용할 수 있습니다."
        if absolute:
            number = abs(number)
        return max(minimum, min(maximum, number)), None

    # -- movement tools (multi-frame; run in their own asyncio loop) --------

    async def _experience_approach_object(self, object_id, clearance_m=1.25):
        prim, error = self._agent_find_object(object_id)
        if error:
            return error
        return await self._experience_object_contact_sequence(prim, float(clearance_m), 0)

    async def _experience_bump_object(self, object_id, repetitions=1):
        prim, error = self._agent_find_object(object_id)
        if error:
            return error
        object_type = prim.GetAttribute("lab:objectType")
        if str(prim.GetPath()) == WORKER_PATH or (
            object_type.IsValid() and object_type.Get() == "character"
        ):
            return {"ok": False, "error": "Worker 충돌은 안전 규칙상 실행할 수 없습니다."}
        return await self._experience_object_contact_sequence(
            prim, 1.25, max(1, min(5, int(repetitions)))
        )

    async def _experience_object_contact_sequence(self, prim, clearance, repetitions):
        """Approach a labeled object and optionally bump/back away repeatedly."""
        if self._robot_motion_prim() is None:
            return {"ok": False, "error": "AGV 설치에 실패했습니다."}
        app = omni.kit.app.get_app()
        self._timeline.play()
        motion_epoch = self._agent_motion_epoch
        object_start = self._prim_xy(prim)
        scale_op = self._find_xform_op(UsdGeom.Xformable(prim), UsdGeom.XformOp.TypeScale)
        object_scale = scale_op.Get() if scale_op else Gf.Vec3f(0.25, 0.25, 0.25)
        cycles = repetitions if repetitions > 0 else 1
        completed = 0
        self._agent_action_active = True
        try:
            for cycle in range(cycles):
                if motion_epoch != self._agent_motion_epoch or self._is_shutting_down:
                    return {"ok": False, "error": "자동체험 이동이 취소되었습니다."}
                target_xy = self._prim_xy(prim)
                robot_xy = self._robot_xy()
                if target_xy is None or robot_xy is None:
                    return {"ok": False, "error": "객체 또는 AGV 위치를 읽을 수 없습니다."}
                dx, dy = robot_xy[0] - target_xy[0], robot_xy[1] - target_xy[1]
                length = max(math.hypot(dx, dy), 1.0e-6)
                ux, uy = dx / length, dy / length
                support_radius = abs(ux) * abs(float(object_scale[0])) + abs(uy) * abs(float(object_scale[1]))
                effective_clearance = max(float(clearance), support_radius + 0.95)
                approach_xy = (
                    target_xy[0] + ux * effective_clearance,
                    target_xy[1] + uy * effective_clearance,
                )
                self._experience_step(f"{prim.GetName()} 접근 · {cycle + 1}/{cycles}회")
                deadline = time.monotonic() + self.AGENT_ACTION_TIMEOUT_SEC * 4.0
                while not self._drive_robot_to(approach_xy, self.AUTO_LINEAR_SPEED, 0.12):
                    if self._avoidance_state == "FAILED":
                        return {"ok": False, "error": self._avoidance_failure}
                    if time.monotonic() > deadline:
                        return {"ok": False, "error": f"{prim.GetName()} 접근 시간 초과"}
                    if motion_epoch != self._agent_motion_epoch:
                        return {"ok": False, "error": "자동체험 이동이 취소되었습니다."}
                    await app.next_update_async()
                turn_deadline = time.monotonic() + 8.0
                while not self._turn_robot_toward(self._prim_xy(prim)):
                    if time.monotonic() > turn_deadline:
                        return {"ok": False, "error": f"{prim.GetName()} 방향 정렬 시간 초과"}
                    await app.next_update_async()
                if repetitions <= 0:
                    return {"ok": True, "message": f"{prim.GetName()} 안전거리 {effective_clearance:.2f}m 접근 완료"}
                self._experience_step(f"{prim.GetName()} 충돌 · {cycle + 1}/{cycles}회")
                impact_until = time.monotonic() + 0.80
                while time.monotonic() < impact_until:
                    self._command_robot_velocity(self.AUTO_PUSH_SPEED, 0.0)
                    await app.next_update_async()
                self._command_robot_velocity(0.0, 0.0)
                retreat_start = self._robot_xy()
                self._experience_step(f"{prim.GetName()} 충돌 후 후퇴 · {cycle + 1}/{cycles}회")
                retreat_deadline = time.monotonic() + 7.0
                while self._distance_xy(self._robot_xy(), retreat_start) < 0.90:
                    self._command_robot_velocity(-self.AUTO_LINEAR_SPEED, 0.0)
                    if time.monotonic() > retreat_deadline:
                        return {"ok": False, "error": f"{prim.GetName()} 후퇴 시간 초과"}
                    await app.next_update_async()
                self._command_robot_velocity(0.0, 0.0)
                completed += 1
                self._reset_obstacle_avoidance()
        finally:
            self._agent_action_active = False
            self._command_robot_velocity(0.0, 0.0)
        moved = self._distance_xy(self._prim_xy(prim), object_start)
        return {"ok": True, "message": f"{prim.GetName()} {completed}회 충돌·후퇴 완료, 객체 이동={moved:.3f}m"}

    async def _agent_drive_distance(self, direction, distance_m):
        if self._robot_motion_prim() is None:
            return {"ok": False, "error": "로봇이 설치되지 않았습니다. Lab 1-C를 먼저 실행하세요."}
        distance_m, error = self._agent_finite_float(
            distance_m, "distance_m", 0.1, 5.0, absolute=True
        )
        if error:
            return {"ok": False, "error": error}

        forward_xy = self._robot_forward_xy()
        start_xy = self._robot_xy()
        if forward_xy is None or start_xy is None:
            return {"ok": False, "error": "로봇 위치를 읽을 수 없습니다."}
        sign = 1.0 if direction == "forward" else -1.0
        target_xy = (
            start_xy[0] + forward_xy[0] * distance_m * sign,
            start_xy[1] + forward_xy[1] * distance_m * sign,
        )
        self._narrate("plan", f"현재 위치 ({start_xy[0]:.2f}, {start_xy[1]:.2f})m에서 {distance_m:g}m 이동 목표 ({target_xy[0]:.2f}, {target_xy[1]:.2f})m를 계산했습니다.")
        speed = self.AUTO_LINEAR_SPEED * sign

        self._timeline.play()
        self._pressed_keys.difference_update({"UP", "DOWN", "LEFT", "RIGHT"})
        self._external_drive_key = None
        self._external_drive_lease_until = 0.0
        app = omni.kit.app.get_app()
        deadline = time.monotonic() + self.AGENT_ACTION_TIMEOUT_SEC
        observed_retry_count = 0
        result = None
        motion_epoch = self._agent_motion_epoch
        self._agent_action_active = True
        try:
            while True:
                if motion_epoch != self._agent_motion_epoch or self._is_shutting_down:
                    result = {"ok": False, "error": "사용자 정지 또는 Reset으로 이동이 취소되었습니다."}
                    break
                if self._drive_robot_to(target_xy, speed, self.AGENT_MOVE_TOLERANCE):
                    break
                if self._avoidance_state == "FAILED":
                    result = {"ok": False, "error": self._avoidance_failure}
                    break
                # Each obstacle retry contains a measured 0.8 m reverse leg,
                # a lateral bypass, and a fresh turn toward the goal.  Give
                # that explicitly requested recovery cycle its own time slice
                # instead of letting the ordinary 20 s move deadline cancel it.
                if self._avoidance_retry_count > observed_retry_count:
                    # On the first detected blockage reserve enough time for
                    # all three recovery cycles. Later retry transitions are
                    # already covered by this budget and must not extend it
                    # without bound.
                    if observed_retry_count == 0:
                        deadline += (
                            self.AGENT_ACTION_TIMEOUT_SEC
                            * self.OBSTACLE_MAX_RETRIES
                        )
                    observed_retry_count = self._avoidance_retry_count
                if self._mission_active:
                    result = {"ok": False, "error": "온도 미션이 시작되어 이동을 중단했습니다."}
                    break
                if time.monotonic() > deadline:
                    result = {
                        "ok": False,
                        "error": "제한 시간 안에 목표 지점에 도달하지 못했습니다. 장애물에 막혔을 수 있습니다.",
                    }
                    break
                await app.next_update_async()
        finally:
            self._agent_action_active = False
            self._command_robot_velocity(0.0, 0.0)

        end_xy = self._robot_xy()
        moved = self._distance_xy(end_xy, start_xy) if end_xy is not None else 0.0
        remaining = self._distance_xy(end_xy, target_xy) if end_xy is not None else distance_m
        if result is None and remaining > self.AGENT_MOVE_TOLERANCE:
            result = {"ok": False, "error": "목표 도달을 확인하지 못했습니다. 장애물과 로봇 상태를 확인하세요."}
        if result is not None:
            result['error'] = str(result.get('error', '이동 미완료')) + f" 실제 이동 {moved:.2f}m / 요청 {distance_m:.2f}m, 목표까지 {remaining:.2f}m 남음."
            self._reset_obstacle_avoidance()
            return result
        self._refresh_mission_ui()
        verb = "전진" if direction == "forward" else "후진"
        return {"ok": True, "message": f"{verb}으로 약 {moved:.2f} m 이동했습니다."}

    async def _agent_turn_degrees(self, direction, degrees):
        if self._robot_motion_prim() is None:
            return {"ok": False, "error": "로봇이 설치되지 않았습니다. Lab 1-C를 먼저 실행하세요."}
        degrees, error = self._agent_finite_float(
            degrees, "degrees", 1.0, 180.0, absolute=True
        )
        if error:
            return {"ok": False, "error": error}
        signed_deg = degrees if direction == "left" else -degrees
        desired_yaw = self._robot_heading() + math.radians(signed_deg)

        self._timeline.play()
        self._pressed_keys.difference_update({"UP", "DOWN", "LEFT", "RIGHT"})
        self._external_drive_key = None
        self._external_drive_lease_until = 0.0
        app = omni.kit.app.get_app()
        deadline = time.monotonic() + self.AGENT_ACTION_TIMEOUT_SEC
        result = None
        motion_epoch = self._agent_motion_epoch
        self._agent_action_active = True
        try:
            while True:
                if motion_epoch != self._agent_motion_epoch or self._is_shutting_down:
                    result = {"ok": False, "error": "사용자 정지 또는 Reset으로 회전이 취소되었습니다."}
                    break
                if self._turn_robot_to_heading(desired_yaw):
                    break
                if self._mission_active:
                    result = {"ok": False, "error": "온도 미션이 시작되어 회전을 중단했습니다."}
                    break
                if time.monotonic() > deadline:
                    result = {"ok": False, "error": "제한 시간 안에 회전을 마치지 못했습니다."}
                    break
                await app.next_update_async()
        finally:
            self._agent_action_active = False
            self._command_robot_velocity(0.0, 0.0)

        if result is not None:
            return result
        side = "왼쪽" if direction == "left" else "오른쪽"
        return {"ok": True, "message": f"{side}으로 약 {degrees:.0f}도 회전했습니다."}

    # -- instantaneous tools (plain synchronous handlers) --------------------

    def _agent_tool_stop(self, args):
        self._stop_robot()
        return {"ok": True, "message": "정지함."}

    def _agent_tool_select(self, args):
        prim, error = self._agent_find_object(args.get("object_id"))
        if error:
            return error
        self._select_path(str(prim.GetPath()))
        return {"ok": True, "message": f"{prim.GetName()} 선택함."}

    def _agent_tool_set_mobility(self, args):
        prim, error = self._agent_find_object(args.get("object_id"))
        if error:
            return error
        mobility = str(args.get("mobility", "")).lower()
        if mobility not in ("fixed", "movable", "kinematic"):
            return {"ok": False, "error": "mobility는 fixed/movable/kinematic 중 하나여야 합니다."}
        self._safe_edit(lambda: self._apply_mobility(prim, mobility))
        return {"ok": True, "message": f"{prim.GetName()} mobility → {mobility}"}

    def _agent_tool_set_mass(self, args):
        prim, error = self._agent_find_object(args.get("object_id"))
        if error:
            return error
        mass, numeric_error = self._agent_finite_float(args.get("mass_kg"), "mass_kg", 0.1, 1000.0)
        if numeric_error:
            return {"ok": False, "error": numeric_error}
        self._safe_edit(lambda: self._set_mass_attr(prim, mass))
        self._narrate("done", f"{prim.GetName()}의 질량을 {mass:g}kg으로 변경했습니다. 변경된 물리 반응을 확인해 보세요.")
        return {"ok": True, "message": f"{prim.GetName()} mass → {mass:g} kg"}

    def _agent_tool_set_color(self, args):
        prim, error = self._agent_find_object(args.get("object_id"))
        if error:
            return error
        color_name = str(args.get("color", "")).strip().lower()
        index = next(
            (i for i, (level_name, _) in enumerate(self.COLOR_LEVELS) if level_name.lower() == color_name),
            None,
        )
        if index is None:
            return {"ok": False, "error": "color는 blue/red/green/yellow/orange 중 하나여야 합니다."}
        self._safe_edit(lambda: self._apply_color_index(prim, index))
        return {"ok": True, "message": f"{prim.GetName()} color → {self.COLOR_LEVELS[index][0]}"}

    def _agent_tool_set_model(self, args):
        prim, error = self._agent_find_object(args.get("object_id"))
        if error:
            return error
        model_key = str(args.get("model", "")).strip().lower()
        index = next(
            (i for i, (_, key) in enumerate(self.MODEL_LEVELS) if key == model_key),
            None,
        )
        if index is None:
            return {"ok": False, "error": "model은 box/worker/robot/forklift 중 하나여야 합니다."}
        self._safe_edit(lambda: self._apply_model_index(prim, index))
        self._select_path(str(prim.GetPath()))
        return {"ok": True, "message": f"{prim.GetName()} model → {self.MODEL_LEVELS[index][0]}"}

    def _agent_tool_scale(self, args):
        prim, error = self._agent_find_object(args.get("object_id"))
        if error:
            return error
        factor, numeric_error = self._agent_finite_float(args.get("factor"), "factor", 0.2, 3.0)
        if numeric_error:
            return {"ok": False, "error": numeric_error}
        self._safe_edit(lambda: self._apply_scale_factor(prim, factor))
        return {"ok": True, "message": f"{prim.GetName()} 크기 × {factor:g}"}

    def _agent_tool_set_arm_pose(self, args):
        pose_name = str(args.get("pose", "")).strip().lower()
        if pose_name not in ("home", "observe"):
            return {"ok": False, "error": "pose는 home/observe 중 하나여야 합니다."}
        ok = self._set_franka_pose(pose_name, report=False)
        if not ok:
            return {"ok": False, "error": "Franka 팔 조인트를 찾지 못했습니다. Lab 1-C를 먼저 실행하세요."}
        return {"ok": True, "message": f"팔 자세 → {pose_name}"}

    def _agent_tool_set_gripper(self, args):
        state = str(args.get("state", "")).strip().lower()
        if state not in ("open", "close", "closed"):
            return {"ok": False, "error": "state는 open/close 중 하나여야 합니다."}
        found = self._set_gripper_state(state == "open", report=False)
        if not found:
            return {"ok": False, "error": "그리퍼 조인트를 찾지 못했습니다. Lab 1-C를 먼저 실행하세요."}
        return {"ok": True, "message": f"그리퍼 → {'열림' if state == 'open' else '닫힘'}"}

    def _agent_tool_launch_pusher(self, args):
        if not self._stage.GetPrimAtPath(PUSHER_PATH).IsValid():
            return {"ok": False, "error": "Pusher가 없습니다. Lab 1-A 또는 Lab 1-B를 먼저 실행하세요."}
        self._launch_pusher()
        return {"ok": True, "message": "Pusher 발사함."}

    def _agent_tool_reset_objects(self, args):
        if not self._stage.GetPrimAtPath(LAB_ROOT).IsValid():
            return {"ok": False, "error": "초기화할 실습 장면이 없습니다."}
        self._reset_objects()
        return {"ok": True, "message": "모든 객체를 초기화함."}

    def _agent_tool_reset_robot(self, args):
        if not self._agv_is_installed():
            return {"ok": False, "error": "초기화할 로봇이 설치되어 있지 않습니다."}
        self._reset_robot()
        return {"ok": True, "message": "로봇을 초기화함."}

    def _agent_tool_reset_mission(self, args):
        self._reset_temperature_mission()
        return {"ok": True, "message": "온도 미션을 초기화함."}

    def _agent_tool_set_view(self, args):
        view = str(args.get("view", "")).strip().lower()
        if view == "robot":
            if not self._show_robot_camera_view():
                return {"ok": False, "error": "로봇 카메라 시점으로 전환하지 못했습니다."}
            return {"ok": True, "message": "로봇 전방 카메라 시점으로 전환함."}
        if view == "main":
            if not self._show_main_view():
                return {"ok": False, "error": "기본 외부 시점으로 전환하지 못했습니다."}
            return {"ok": True, "message": "기본 외부 시점으로 전환함."}
        return {"ok": False, "error": "view는 robot/main 중 하나여야 합니다."}

    def _agent_tool_get_status(self, args):
        return {"ok": True, "world_state": self._agent_world_snapshot()}

    def _set_status(self, text):
        self._last_status_text = str(text)
        if hasattr(self, "_robot_messages") and not str(text).startswith(("PERSON SAFETY", "EXTERNAL DRIVE")):
            failed = any(w in str(text).lower() for w in ("error", "failed", "aborted", "실패", "오류"))
            self._narrate("error" if failed else "sensor", str(text), key="status", interval=1, alert="operation" if failed else None)
        carb.log_info(f"[Interactive Physics Lab] {text}")
        if self._status_label is not None:
            self._status_label.text = text

    def _reveal_agv(self):
        """Reveal the model without displaying its invisible collider."""
        stage = self._stage
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            robot = stage.GetPrimAtPath(ROBOT_PATH)
            for prim in Usd.PrimRange(robot):
                if str(prim.GetPath()) == f"{ROBOT_PATH}/CollisionShape":
                    continue
                imageable = UsdGeom.Imageable(prim)
                if imageable:
                    imageable.MakeVisible()
                if prim.IsA(UsdGeom.Gprim):
                    UsdGeom.Gprim(prim).CreateDisplayOpacityAttr([1.0])

    async def _author_agv_async(self, stage, app):
        spawn=(-4.0, -3.0, ROBOT_BASE_HEIGHT)
        floor_path=''
        if self._room_kind == 'usd':
            spawn,floor_path=await find_room_spawn(stage, LAB_ROOT+'/Room', app, self._set_status)
        robot = self._prepare_agv_install_root(stage)
        robot.CreateAttribute("lab:agvInstallComplete", Sdf.ValueTypeNames.Bool).Set(False)
        self._author_clean_transform(robot, spawn, (0.7, 0.7, 0.7))
        robot.CreateAttribute('lab:spawnFloorZ', Sdf.ValueTypeNames.Double).Set(spawn[2]-ROBOT_BASE_HEIGHT)
        robot.CreateAttribute('lab:spawnFloorPath', Sdf.ValueTypeNames.String).Set(floor_path)
        robot.CreateAttribute("lab:agvSizeVersion", Sdf.ValueTypeNames.Int).Set(1)
        robot.CreateAttribute("lab:isRobot", Sdf.ValueTypeNames.Bool).Set(True)
        robot.CreateAttribute("lab:robotModel", Sdf.ValueTypeNames.String).Set(
            "Educational PhysX AGV"
        )
        robot.CreateAttribute("lab:initialPosition", Sdf.ValueTypeNames.Double3).Set(
            Gf.Vec3d(*spawn)
        )
        robot.CreateAttribute("lab:initialScale", Sdf.ValueTypeNames.Float3).Set(
            Gf.Vec3f(0.7, 0.7, 0.7)
        )
        robot.CreateAttribute("lab:initialOrientation", Sdf.ValueTypeNames.Quatf).Set(
            Gf.Quatf(1.0, Gf.Vec3f(0.0))
        )
        self._set_status("ROBOT INSTALL 1/5: creating standalone AGV rigid body...")

        collision = UsdGeom.Cube.Define(stage, f"{ROBOT_PATH}/CollisionShape")
        collision.CreateSizeAttr(2.0)
        collision.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
        self._author_clean_transform(collision.GetPrim(), (0.0, 0.0, 0.0), (0.82, 0.55, 0.28))
        UsdPhysics.CollisionAPI.Apply(collision.GetPrim())
        rigid = UsdPhysics.RigidBodyAPI.Apply(robot)
        rigid.CreateRigidBodyEnabledAttr(True).Set(True)
        rigid.CreateKinematicEnabledAttr(False).Set(False)
        self._set_mass_attr(robot, 120.0)
        await self._wait_frames(app)

        visuals = UsdGeom.Xform.Define(stage, f"{ROBOT_PATH}/ModelVisuals").GetPrim()
        self._create_visual_cube(f"{visuals.GetPath()}/Body", (0.0, 0.0, 0.08), (0.78, 0.50, 0.22), (0.85, 0.08, 0.06))
        self._create_visual_cube(f"{visuals.GetPath()}/Top", (-0.10, 0.0, 0.38), (0.42, 0.38, 0.10), (0.75, 0.78, 0.82))
        self._create_visual_cube(f"{visuals.GetPath()}/FrontBumper", (0.84, 0.0, 0.0), (0.06, 0.52, 0.18), (0.95, 0.55, 0.08))
        for name, pos in (("WheelFL", (0.42, 0.57, -0.05)), ("WheelFR", (0.42, -0.57, -0.05)), ("WheelRL", (-0.42, 0.57, -0.05)), ("WheelRR", (-0.42, -0.57, -0.05))):
            wheel = UsdGeom.Cylinder.Define(stage, f"{visuals.GetPath()}/{name}")
            wheel.CreateRadiusAttr(0.18)
            wheel.CreateHeightAttr(0.12)
            wheel.CreateAxisAttr(UsdGeom.Tokens.y)
            wheel.CreateDisplayColorAttr([Gf.Vec3f(0.05, 0.05, 0.05)])
            self._author_clean_transform(wheel.GetPrim(), pos, (1.0, 1.0, 1.0))
        self._set_status("ROBOT INSTALL 2/5: collision and visual model separated.")
        await self._wait_frames(app)

        self._robot_motion_prim_path = str(robot.GetPath())
        self._mobile_base_drive_paths = None
        camera_prim = self._install_robot_camera(robot)
        self._set_status(
            f"ROBOT INSTALL 3/5: rigid root={robot.GetName()}, camera={camera_prim.GetName()}."
        )
        await self._wait_frames(app)

        self._command_robot_velocity(0.0, 0.0)
        self._set_status("ROBOT INSTALL 4/5: PhysX linear/angular velocity connected.")
        await self._wait_frames(app)

        self._robot_yaw_deg = 0.0
        if self._stage != stage or not robot.IsValid():
            raise RuntimeError("AGV 설치가 끝나기 전에 장면이 교체됐습니다.")
        robot.GetAttribute("lab:agvInstallComplete").Set(True)
        if not self._agv_is_installed():
            robot.GetAttribute("lab:agvInstallComplete").Set(False)
            raise RuntimeError("차량 차체·바퀴·카메라 생성 검증에 실패했습니다.")
        self._reveal_agv()
        self._select_path(ROBOT_PATH)
        self._initialize_mission_from_stage(armed=False)
        self._frame_prim_with_teaching_camera(robot)
        carb.log_info(
            f"[Lab 1-C] PHYSX AGV READY: root={robot.GetPath()}, mass=120kg, camera={camera_prim.GetPath()}"
        )
        self._set_status(
            "ROBOT INSTALL 5/5: PHYSX AGV READY.\n"
            f"출발 위치: ({spawn[0]:.2f}, {spawn[1]:.2f}, {spawn[2]:.2f}) · 카메라 높이 약 1.5m\n"
            "Play: ↑/←/↓/→ | AGV Camera.\n"
            + ("새 공간의 출발점에 AGV를 생성했습니다." if self._room_kind == 'usd' else "Next: Lab 2: Temperature Mission.")
        )

    def _prepare_agv_install_root(self, stage):
        """Replace this robot's local opinions and mask stale referenced parts."""
        if stage.GetEditTarget().GetLayer() != stage.GetSessionLayer():
            raise RuntimeError("AGV 복구는 현재 장면의 세션 레이어에서 실행해야 합니다.")
        stage.RemovePrim(ROBOT_PATH)
        known = ("", "CollisionShape", "ModelVisuals", "ModelVisuals/Body",
                 "ModelVisuals/Top", "ModelVisuals/FrontBumper",
                 "ModelVisuals/WheelFL", "ModelVisuals/WheelFR",
                 "ModelVisuals/WheelRL", "ModelVisuals/WheelRR", "LabFrontCamera")
        paths = {ROBOT_PATH + ("/" + name if name else "") for name in known}
        layer = stage.GetSessionLayer()
        # Define/reactivate parents and children together. A weak inactive or
        # instanced parent can otherwise make OverridePrim(child) return null.
        # Use Sdf only inside the change block; query USD after recomposition.
        with Sdf.ChangeBlock():
            for name in known:
                path = ROBOT_PATH + ("/" + name if name else "")
                spec = Sdf.CreatePrimInLayer(layer, path)
                spec.specifier = Sdf.SpecifierDef
                spec.typeName = ("Xform" if name in ("", "ModelVisuals") else
                                 "Camera" if name == "LabFrontCamera" else
                                 "Cylinder" if "/Wheel" in name else "Cube")
                spec.active = True
                spec.instanceable = False
                spec.referenceList.explicitItems = []
                spec.payloadList.explicitItems = []
                spec.SetInfo("apiSchemas", Sdf.TokenListOp.CreateExplicit([]))
                binding = Sdf.RelationshipSpec(spec, "material:binding", False)
                binding.targetPathList.explicitItems = []
        robot = stage.GetPrimAtPath(ROBOT_PATH)
        # An older referenced model can contain extra meshes/colliders. Mask
        # only those robot descendants; leave the room and other objects alone.
        old_paths = [str(p.GetPath()) for p in Usd.PrimRange(robot)]
        with Sdf.ChangeBlock():
            for path in old_paths:
                if path not in paths:
                    Sdf.CreatePrimInLayer(layer, path).active = False
        UsdGeom.Imageable(robot).CreatePurposeAttr(UsdGeom.Tokens.default_)
        return robot
