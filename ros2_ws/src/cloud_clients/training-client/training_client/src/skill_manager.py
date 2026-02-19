"""
Core skill+run lifecycle manager.

Exposes generator functions that yield :class:`ProgressUpdate` at each step.
Consumers (CLI, ROS node) iterate these generators to drive the workflow and
present progress however they like.
"""
from __future__ import annotations

import json
import logging
import time
from pathlib import Path
from typing import Any, Generator

from .client import OrchestratorClient
from .compression import cleanup_compressed
from .downloader import download_results
from .types import (
    ClientConfig,
    RunInfo,
    SkillInfo,
    ProgressStage,
    ProgressUpdate,
)
from .uploader import upload_data_files

logger = logging.getLogger(__name__)

# Files/dirs to skip when enumerating source files
_IGNORE_PATTERNS = {".git", "__pycache__", "*.zst", "*.zst.tmp"}

SKILL_JSON = "server-skill.json"


def read_skill_id(skill_dir: str | Path) -> str | None:
    """Read the skill_id from server-skill.json inside *skill_dir*, or None."""
    p = Path(skill_dir) / SKILL_JSON
    if not p.is_file():
        return None
    try:
        data = json.loads(p.read_text())
        return data.get("id")
    except (json.JSONDecodeError, OSError) as e:
        logger.warning("Failed to read %s: %s", p, e)
        return None


def write_skill_id(skill_dir: str | Path, skill_id: str) -> None:
    """Write the skill_id to server-skill.json inside *skill_dir*."""
    p = Path(skill_dir) / SKILL_JSON
    p.write_text(json.dumps({"id": skill_id}) + "\n")
    logger.info("Wrote skill_id %s to %s", skill_id, p)


class SkillManager:
    """
    High-level API for the training client.

    Every public method is a **generator** that yields ``ProgressUpdate``
    objects.  The caller iterates to drive the workflow forward.
    """

    def __init__(self, config: ClientConfig) -> None:
        self.config = config
        self.client = OrchestratorClient(config)

    # ── Full lifecycle ──────────────────────────────────────────────

    def submit(
        self,
        source_dir: str | Path,
        name: str | None = None,
    ) -> Generator[ProgressUpdate, None, SkillInfo]:
        """
        Create a skill (or reuse one from server-skill.json).

        Does **not** upload files — use ``upload_files`` for that.

        Yields progress updates throughout.  The final return value is
        the ``SkillInfo``.

        Parameters
        ----------
        source_dir
            Skill directory.  ``server-skill.json`` will be written here.
        name
            Skill name.  Defaults to the directory name.
        """
        source_dir = Path(source_dir).resolve()
        if not source_dir.is_dir():
            raise FileNotFoundError(f"Source directory does not exist: {source_dir}")

        skill_name = name or source_dir.name

        # ── Create or reuse skill ───────────────────────────────────
        existing_id = read_skill_id(source_dir)
        if existing_id:
            yield ProgressUpdate(
                stage=ProgressStage.CREATING,
                message=f"Reusing existing skill {existing_id} from server-skill.json",
                skill_id=existing_id,
            )
            skill = self.client.get_skill(existing_id)
        else:
            yield ProgressUpdate(
                stage=ProgressStage.CREATING,
                message="Creating skill on server…",
            )

            skill = self.client.create_skill(name=skill_name)
            write_skill_id(source_dir, skill.skill_id)

            yield ProgressUpdate(
                stage=ProgressStage.CREATING,
                message=f"Skill created: {skill.skill_id}",
                skill_id=skill.skill_id,
            )

        return skill

    # ── Re-upload files for an existing skill ───────────────────────

    def upload_files(
        self,
        skill_id: str,
        source_dir: str | Path,
    ) -> Generator[ProgressUpdate, None, None]:
        """
        Upload data files for an existing skill.

        This can be used to upload more data files to a skill that
        already exists (e.g. after a failed upload).
        """
        source_dir = Path(source_dir).resolve()
        if not source_dir.is_dir():
            raise FileNotFoundError(f"Source directory does not exist: {source_dir}")

        filenames = _enumerate_files(source_dir)
        if not filenames:
            raise ValueError(f"No files found in {source_dir}")

        zst_filenames = [f + ".zst" for f in filenames]

        yield ProgressUpdate(
            stage=ProgressStage.UPLOADING,
            message=f"Uploading {len(filenames)} file(s) to skill {skill_id}",
            skill_id=skill_id,
        )

        url_resp = self.client.request_file_urls(skill_id, zst_filenames)

        yield from upload_data_files(
            client=self.client,
            config=self.config,
            source_dir=source_dir,
            filenames=filenames,
            upload_urls=url_resp.get("upload_urls", {}),
            download_urls=url_resp.get("download_urls", {}),
        )

        yield ProgressUpdate(
            stage=ProgressStage.DONE,
            message=f"All files uploaded for skill {skill_id}",
            skill_id=skill_id,
        )

    # ── Poll run status ─────────────────────────────────────────────

    def watch(
        self,
        skill_id: str,
        run_id: int,
        *,
        interval: float | None = None,
    ) -> Generator[ProgressUpdate, None, RunInfo]:
        """
        Poll run status every *interval* seconds until terminal state.

        Yields a :class:`ProgressUpdate` on every poll.
        Returns the final :class:`RunInfo`.
        """
        interval = interval or self.config.poll_interval_seconds
        prev_status = None
        prev_daemon_state = None

        while True:
            run = self.client.get_run(skill_id, run_id)

            changed = (
                run.status != prev_status
                or run.daemon_state != prev_daemon_state
            )
            if changed or prev_status is None:
                state_str = run.status
                if run.daemon_state:
                    state_str += f" ({run.daemon_state})"
                if run.error_message:
                    state_str += f" — {run.error_message}"

                yield ProgressUpdate(
                    stage=ProgressStage.WAITING,
                    message=f"Run {skill_id}/{run_id}: {state_str}",
                    skill_id=skill_id,
                    run_id=run_id,
                )

                prev_status = run.status
                prev_daemon_state = run.daemon_state

            if run.is_terminal:
                return run

            time.sleep(interval)

    # ── Status snapshot ─────────────────────────────────────────────

    def run_status(self, skill_id: str, run_id: int) -> RunInfo:
        """Get current status of a run (single poll)."""
        return self.client.get_run(skill_id, run_id)

    def skill_info(self, skill_id: str) -> SkillInfo:
        """Get skill details."""
        return self.client.get_skill(skill_id)

    def list_skills(self) -> list[SkillInfo]:
        """List all skills for the authenticated user."""
        return self.client.list_skills()

    def list_runs(self, skill_id: str) -> list[RunInfo]:
        """List all runs for a skill."""
        return self.client.list_runs(skill_id)

    # ── Download results ────────────────────────────────────────────

    def download(
        self,
        skill_id: str,
        run_id: int,
        dest_dir: str | Path | None = None,
    ) -> Generator[ProgressUpdate, None, None]:
        """
        Download all result files for a completed run.

        If *dest_dir* is not given, downloads to the source_dir from
        the run's training_params.

        After download, marks the run as ``downloaded``.
        """
        run = self.client.get_run(skill_id, run_id)

        if dest_dir:
            dest = Path(dest_dir).resolve()
        elif run.source_dir:
            dest = Path(run.source_dir)
        else:
            raise ValueError(
                f"No dest_dir provided and run {skill_id}/{run_id} has no source_dir in params"
            )

        # Download into a run-specific subdirectory
        dest = dest / str(run_id)

        yield from download_results(
            client=self.client,
            config=self.config,
            skill_id=skill_id,
            run_id=run_id,
            dest_dir=dest,
        )

        # Mark as downloaded
        self.client.update_run(skill_id, run_id, status="downloaded")

        yield ProgressUpdate(
            stage=ProgressStage.DONE,
            message=f"Run {skill_id}/{run_id} results downloaded to {dest}",
            skill_id=skill_id,
            run_id=run_id,
        )

    # ── Cleanup ─────────────────────────────────────────────────────

    def cleanup(self, source_dir: str | Path) -> None:
        """Remove local .zst files from a source directory."""
        source_dir = Path(source_dir).resolve()
        if source_dir.is_dir():
            filenames = _enumerate_files(source_dir)
            cleanup_compressed(source_dir, filenames)


# ── Helpers ─────────────────────────────────────────────────────────


def _enumerate_files(source_dir: Path) -> list[str]:
    """
    List uploadable files in *source_dir*: root-level files and
    everything under ``data/``.  Skips hidden dirs, __pycache__,
    and .zst artifacts.
    """
    files: list[str] = []

    def _should_skip(rel: Path) -> bool:
        if any(p.startswith(".") or p == "__pycache__" for p in rel.parts):
            return True
        if rel.suffix == ".zst" or rel.name.endswith(".zst.tmp"):
            return True
        if rel.name == SKILL_JSON:
            return True
        return False

    # Root-level files only (no recursion)
    for path in sorted(source_dir.iterdir()):
        if not path.is_file():
            continue
        rel = path.relative_to(source_dir)
        if _should_skip(rel):
            continue
        files.append(str(rel))

    # Everything under data/
    data_dir = source_dir / "data"
    if data_dir.is_dir():
        for path in sorted(data_dir.rglob("*")):
            if not path.is_file():
                continue
            rel = path.relative_to(source_dir)
            if _should_skip(rel):
                continue
            files.append(str(rel))

    return files
