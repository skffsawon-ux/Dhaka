"""Background workers: polling, upload, download.

All functions in this module run on daemon threads and communicate with
the rest of the node exclusively through :class:`JobStore` and the
``rclpy`` logger.
"""

from __future__ import annotations

import json
import logging
import threading
import time
from pathlib import Path

from innate_cloud_msgs.msg import TransferProgress

from training_client.src.skill_manager import SkillManager
from training_client.src.types import ProgressStage, ProgressUpdate

from .job_store import JobStore

logger = logging.getLogger(__name__)

_SKILL_SEARCH_DIRS: list[Path] = [
    Path.home() / "skills",
    Path.home() / "innate-os" / "skills",
]


def id_to_skill_dir(skill_id: str) -> str | None:
    """Locate the on-disk directory for *skill_id*.

    Searches ``~/skills/*/metadata.json`` and
    ``~/innate-os/skills/*/metadata.json`` for a JSON object whose
    ``"id"`` field matches *skill_id*.  Returns the directory path as a
    string, or ``None`` if not found.
    """
    for base in _SKILL_SEARCH_DIRS:
        if not base.is_dir():
            continue
        for meta_path in base.glob("*/metadata.json"):
            try:
                data = json.loads(meta_path.read_text())
                if data.get("id") == skill_id:
                    return str(meta_path.parent)
            except (json.JSONDecodeError, OSError):
                continue
    return None


class Poller:
    """Periodically fetches all jobs, polls active runs, and starts uploads."""

    def __init__(
        self,
        manager: SkillManager,
        store: JobStore,
        poll_interval: float,
    ) -> None:
        self._mgr = manager
        self._store = store
        self._interval = poll_interval
        self._shutting_down = False

    def start(self) -> None:
        threading.Thread(target=self._loop, daemon=True, name="training_poll").start()

    def stop(self) -> None:
        self._shutting_down = True

    # ── Main loop ───────────────────────────────────────────────────

    def _loop(self) -> None:
        try:
            self._fetch_all()
        except Exception as e:
            logger.error("Initial fetch failed: %s", e)

        while not self._shutting_down:
            try:
                self._poll_active()
            except Exception as e:
                logger.warning("Poll cycle error: %s", e)
            time.sleep(self._interval)

    def _fetch_all(self) -> None:
        logger.info("Fetching all skills and runs…")
        try:
            skills = self._mgr.list_skills()
        except Exception as e:
            logger.error("list_skills failed: %s", e)
            return

        for skill in skills:
            self._store.put_skill(skill)

        for skill in skills:
            try:
                for run in self._mgr.list_runs(skill.skill_id):
                    self._store.put_job(run)
                    if run.status == "done":
                        dest = id_to_skill_dir(run.skill_id)
                        if dest:
                            maybe_auto_download(
                                self._mgr,
                                self._store,
                                run.skill_id,
                                run.run_id,
                                dest,
                            )
            except Exception as e:
                logger.warning("list_runs(%s) failed: %s", skill.skill_id, e)

    def _poll_active(self) -> None:
        for (skill_id, run_id), old in self._store.active_jobs():
            if self._shutting_down:
                return
            try:
                new = self._mgr.run_status(skill_id, run_id)
                self._store.put_job(new)
                if new.status == "done" and old.status != "done":
                    logger.info("Run %s/%s done → auto-download", skill_id, run_id)
                    dest = id_to_skill_dir(skill_id)
                    if dest:
                        maybe_auto_download(
                            self._mgr,
                            self._store,
                            skill_id,
                            run_id,
                            dest,
                        )
                    else:
                        logger.warning(
                            "No skill dir found for %s — skipping auto-download",
                            skill_id,
                        )
            except Exception as e:
                logger.warning("Poll %s/%s failed: %s", skill_id, run_id, e)


# ── Upload worker ───────────────────────────────────────────────────


def do_upload(
    manager: SkillManager,
    store: JobStore,
    skill_id: str,
    skill_dir: str,
) -> None:
    """Compress + upload data files for *skill_id*."""
    try:
        for update in manager.upload_files(skill_id, skill_dir):
            store.update_transfer(TransferProgress.UPLOAD, skill_id, -1, update)
        logger.info("Upload complete for skill %s", skill_id)
    except Exception as e:
        logger.error("Upload failed for %s: %s", skill_id, e)
        store.update_transfer(
            TransferProgress.UPLOAD,
            skill_id,
            -1,
            ProgressUpdate(
                stage=ProgressStage.ERROR,
                message=f"Upload failed: {e}",
                skill_id=skill_id,
                error=str(e),
            ),
        )
        return

    store.update_transfer(
        TransferProgress.UPLOAD,
        skill_id,
        -1,
        ProgressUpdate(
            stage=ProgressStage.DONE,
            message="Upload complete",
            skill_id=skill_id,
        ),
    )


# ── Download worker ─────────────────────────────────────────────────


def do_download(
    manager: SkillManager,
    store: JobStore,
    skill_id: str,
    run_id: int,
    dest_dir: str,
) -> None:
    """Download & decompress result files for a run."""
    try:
        for update in manager.download(skill_id, run_id, dest_dir=dest_dir):
            store.update_transfer(TransferProgress.DOWNLOAD, skill_id, run_id, update)
        # Clear finished transfer entry.
        store.update_transfer(
            TransferProgress.DOWNLOAD,
            skill_id,
            run_id,
            ProgressUpdate(
                stage=ProgressStage.DONE,
                message="Download complete",
                skill_id=skill_id,
                run_id=run_id,
            ),
        )
        # Refresh cached state (download marks the run as "downloaded").
        try:
            store.put_job(manager.run_status(skill_id, run_id))
        except Exception:
            pass
        logger.info("Download complete for %s/%s → %s", skill_id, run_id, dest_dir)
    except Exception as e:
        logger.error("Download failed for %s/%s: %s", skill_id, run_id, e)
        store.update_transfer(
            TransferProgress.DOWNLOAD,
            skill_id,
            run_id,
            ProgressUpdate(
                stage=ProgressStage.ERROR,
                message=f"Download failed: {e}",
                skill_id=skill_id,
                run_id=run_id,
                error=str(e),
            ),
        )


# ── Auto-download helper ───────────────────────────────────────────


def maybe_auto_download(
    manager: SkillManager,
    store: JobStore,
    skill_id: str,
    run_id: int,
    download_dir: str,
) -> None:
    """Spawn a download thread if one hasn't been started already."""
    if not store.mark_download_started(skill_id, run_id):
        return

    threading.Thread(
        target=do_download,
        args=(manager, store, skill_id, run_id, download_dir),
        daemon=True,
        name=f"dl-{skill_id[:8]}-{run_id}",
    ).start()
