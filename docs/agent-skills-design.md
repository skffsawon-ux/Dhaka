# Skill Identity & Directory Design

## Skill Types

There are two fundamentally different kinds of skills:

### Code Skills
A `.py` file in a skills directory. Contains a class inheriting from `Skill` with `name`, `execute()`, and `cancel()`. Runs in-process inside skills_action_server. Written by developers.

### Physical Skills
A subdirectory containing `metadata.json`. Involve the robot's arm/body. Have a lifecycle:

1. **Recording** (recorder_node) — human demonstrates by puppeteering the arm. Creates directory + `metadata.json` + episode H5 files under `data/`.
2. **Training** (training_node → cloud) — episode data uploaded, neural network trained, checkpoint downloaded back.
3. **Execution** (skills_action_server → behavior_server) — runs the trained policy or replays recorded motion.

Physical skill subtypes:
- **Learned** (`"type": "learned"`) — ACT neural network policy inference
- **Replay** (`"type": "replay"`) — plays back a recorded H5 file's actions

### Directory structure of a physical skill
```
pick-socks/
├── metadata.json                    ← skill definition (name, type, execution config)
└── data/
    ├── dataset_metadata.json        ← recording metadata (episode count, timestamps)
    ├── episode_0.h5
    └── episode_1.h5
```

### Execution call chain
```
brain_client (LLM) → /execute_skill → skills_action_server
                                           │
                                 ┌─────────┴─────────┐
                           Code skill?          Physical skill?
                           Run execute()        Forward to behavior_server
                           in-process           via /behavior/execute
                                                     │
                                           ┌─────────┴─────────┐
                                       Learned?            Replay?
                                       Run ACT policy      Play H5 actions
```

---

## Skill ID Format

Skill IDs follow the pattern: `<user>/<skill_name>` matching `[a-z0-9_-]+/[a-z0-9_-]+`.

- **Code skill ID**: `<user>/<filename_without_py>` (e.g. `innate/navigate_to_position`)
- **Physical skill ID**: `<user>/<directory_name>` (e.g. `local/pick-socks`)

### Character rules
- Allowed characters in both `<user>` and `<skill_name>`: `[a-z0-9_-]`
- All inputs must be stripped of leading/trailing whitespace
- When creating a skill:
  - Either `name` (display name) or `id` must be provided
  - If `name` is not provided → `name = skill_name` portion of the ID
  - If `id` is not provided → `skill_name` derived from name: spaces → dashes, all other special chars removed, uppercase → lowercase
  - `id` must start with `innate/` or `local/`
- The `name` field in `metadata.json` is the human-readable display name (free-form text, not constrained)
- The directory name on disk **is** the `<skill_name>` portion of the ID

### Special users

| User | Disk location | Description |
|------|--------------|-------------|
| `local` | `~/skills/<skill_name>/` | User-created skills (recording, custom code) |
| `innate` | `$INNATE_OS_ROOT/skills/<skill_name>/` | Built-in skills shipped with innate-os |

No other users are valid yet. Future: cloud-synced skills from other users.

### ID ↔ Path resolution

```
innate/<skill_name>  →  $INNATE_OS_ROOT/skills/<skill_name>/
local/<skill_name>   →  ~/skills/<skill_name>/
```

For code skills, the "directory" is the parent directory containing the `.py` file:
```
innate/navigate_to_position  →  $INNATE_OS_ROOT/skills/navigate_to_position.py
local/my-custom-skill        →  ~/skills/my-custom-skill.py
```

### Examples

| ID | Display Name | Path |
|----|-------------|------|
| `local/pick-socks` | Pick Socks | `~/skills/pick-socks/` |
| `innate/wave` | wave | `$INNATE_OS_ROOT/skills/wave/` |
| `innate/navigate_to_position` | navigate_to_position | `$INNATE_OS_ROOT/skills/navigate_to_position.py` |
| `local/arm-circle` | Arm Circle | `~/skills/arm-circle/` |

---

## Interface Decisions

### Actions

| Interface | Type | Route | Param | Format | Notes |
|-----------|------|-------|-------|--------|-------|
| `/execute_skill` | `ExecuteSkill` | brain_client → skills_action_server | `skill_id` | **ID** (`user/name`) | Was `skill_type` (bare name). skills_action_server resolves to path internally |
| `/behavior/execute` | `ExecuteBehavior` | skills_action_server → behavior_server | `skill_dir` | **Absolute path** | Was `behavior_name`. Internal interface — skills_action_server already knows the path, just pass it through. Fixes the ~/skills vs ~/innate-os/skills bug |

### Services — Skill Discovery & Reload

| Interface | Type | Server | Param | Format | Notes |
|-----------|------|--------|-------|--------|-------|
| `/brain/get_available_skills` | `GetAvailableSkills` | skills_action_server | *(returns list)* | Response includes **id**, **name**, **abs path** per skill | Replaces the old `GetAvailablePrimitives` (already removed from code, only in old logs) |
| `/brain/reload_primitives` | `Trigger` | skills_action_server | *(none)* | No change | Reloads all skills from disk |
| `/brain/reload_skills` | `ReloadSkillsAgents` | skills_action_server | `skills[]` | **IDs** | Was bare names |
| `/brain/reload_skills_agents` | `ReloadSkillsAgents` | brain_client | `skills[]` | **IDs** | Was bare names. Forwards to `/brain/reload_skills` |
| `/brain/reload` | `Trigger` | brain_client | *(none)* | No change | Full reload of everything |

### Services — Recording (recorder_node)

| Interface | Type | Param | Format | Notes |
|-----------|------|-------|--------|-------|
| `brain/recorder/new_physical_primitive` | `ManipulationTask` | `skill_id`, `name` | **ID** + optional display name | Was `task_name` + `task_directory`. ID must be `local/` or `innate/`. If only name given, derive ID as `local/<sanitized_name>`. If only ID given, name defaults to skill_name portion. Recorder resolves ID → abs path and creates the directory |
| `brain/recorder/get_task_metadata` | `GetTaskMetadata` | `skill_id` | **ID** | Was `task_directory` (abs path). Recorder resolves internally |
| `brain/recorder/update_task_metadata` | `UpdateTaskMetadata` | `skill_id` | **ID** | Was `task_directory` |
| `brain/recorder/load_episode` | `LoadEpisode` | `skill_id`, `episode_id` | **ID** + int | Was `task_directory` + int |
| `brain/recorder/get_task_metadata_list` | `GetTaskMetadataList` | *(none)* | No change | Returns all. Response should include **id** per task |
| `brain/recorder/new_episode` | `Trigger` | *(none)* | No change | Uses active task |
| `brain/recorder/save_episode` | `Trigger` | *(none)* | No change | |
| `brain/recorder/cancel_episode` | `Trigger` | *(none)* | No change | |
| `brain/recorder/stop_episode` | `Trigger` | *(none)* | No change | |
| `brain/recorder/end_task` | `Trigger` | *(none)* | No change | |
| `brain/recorder/play_replay` | `Trigger` | *(none)* | No change | |
| `brain/recorder/pause_replay` | `Trigger` | *(none)* | No change | |
| `brain/recorder/stop_replay` | `Trigger` | *(none)* | No change | |

### Services — Cloud Training (innate_training node)

| Interface | Type | Param | Format | Notes |
|-----------|------|-------|--------|-------|
| `~/submit_skill` | `SubmitSkill` | `skill_dir` | **Absolute path** | `name` param removed — read display name from `metadata.json` instead |
| `~/create_run` | `CreateRun` | `skill_dir` | **Absolute path** | No change |
| `~/download_results` | `DownloadResults` | `skill_dir`, `run_id` | **Absolute path** + int | No change |

### Topics

| Topic | Type | Publisher | Fields | Notes |
|-------|------|----------|--------|-------|
| `/brain/recorder/status` | `RecorderStatus` | recorder_node | `skill_id`, `episode_number`, `status` | Was `current_task_name`. Change to **ID** |
| `/brain/recorder/replay_status` | `ReplayStatus` | recorder_node | `skill_id`, `episode_id`, etc. | Was `task_name`. Change to **ID** |
| `~/job_statuses` | `TrainingJobList` | training_node | `training_skill_id`, `skill_name`, `skill_dir` | `training_skill_id` is the cloud server's UUID, separate from the local skill ID. `skill_dir` is the abs path. No change needed |

### Internal interfaces (not on ROS)

| Interface | Where | Param | Notes |
|-----------|-------|-------|-------|
| Agent `get_skills()` | agent Python classes | Returns **IDs** | Was bare names. Agents declare which skills they can use by ID |
| `register_primitives_and_directive` | brain_client → websocket | Each primitive includes **id**, **name** | LLM sees both; uses ID in tool calls |
| Hot reload watcher | brain_client | Maps file changes to **IDs** | Derives `innate/<stem>` or `local/<stem>` based on which directory the file is in |

### Unused / To Remove

| Interface | Status |
|-----------|--------|
| `GetAvailablePrimitives.srv` | Already removed from code (only in old zenoh logs) |
| `GetAvailableBehaviors.srv` | Definition exists but no node uses it — **delete** |
| `/policy/execute` action | Legacy hardcoded inference node — **not launched in production** |

---

## Bugs Fixed by This Redesign

1. **behavior_server only searched `$INNATE_OS_ROOT/skills/`** → now receives absolute path from skills_action_server, no path guessing
2. **recorder_node hardcoded `~/innate-os/skills`** → resolves from ID using same logic as all other nodes
3. **metadata.json `name` ≠ directory name** → ID is always based on directory name; `name` is display-only, never used for path resolution
4. **recorder.yaml `data_directory: "~/skills"` vs behavior_server `~/innate-os/skills/`** → both resolve from ID; `local/` → `~/skills/`, `innate/` → `$INNATE_OS_ROOT/skills/`
