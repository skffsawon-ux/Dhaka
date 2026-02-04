#!/usr/bin/env python3
"""
Skill that rotates the robot 360 degrees while scanning for objects using Gemini.
"""
import base64
import math
import json
from pathlib import Path
from datetime import datetime
import google.generativeai as genai
from brain_client.skill_types import Skill, SkillResult, RobotState, RobotStateType


def _load_env_file(env_path: Path) -> dict:
    """Load environment variables from a file."""
    env_vars = {}
    if env_path.exists():
        with open(env_path) as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith("#") and "=" in line:
                    key, value = line.split("=", 1)
                    env_vars[key.strip()] = value.strip()
    return env_vars


class ScanForObjects(Skill):
    """
    Rotates the robot in place while capturing images and detecting objects
    using Google Gemini vision API.
    """

    # Camera image is continuously updated during execution
    image = RobotState(RobotStateType.LAST_MAIN_CAMERA_IMAGE_B64)

    def __init__(self, logger):
        super().__init__(logger)
        # Load config from .env.scan next to this skill
        env_path = Path(__file__).parent / ".env.scan"
        env_vars = _load_env_file(env_path)
        
        self.api_key = env_vars.get("GEMINI_API_KEY", "")
        if self.api_key and self.api_key != "your_gemini_api_key_here":
            genai.configure(api_key=self.api_key)
            self.model = genai.GenerativeModel("gemini-2.0-flash")
            self.logger.info("[ScanForObjects] Gemini configured")
        else:
            self.model = None
            self.logger.warn(f"[ScanForObjects] GEMINI_API_KEY not set in {env_path}")
        
        # Scan parameters
        self.num_snapshots = 8    # Number of images to capture during rotation
        
        # Debug output directory
        self.debug_dir = Path(__file__).parent / "scan_debug"
        self._snapshot_counter = 0

    @property
    def name(self):
        return "scan_for_objects"

    def guidelines(self):
        return (
            "Use to scan surroundings for objects. The robot rotates 360 degrees "
            "while taking pictures and using computer vision to detect objects. "
            "Optionally specify target_object to search for a specific item. "
            "Returns list of detected objects and their approximate directions."
        )

    def execute(self, target_object: str = None):
        """
        Rotates 360 degrees while scanning for objects.

        Args:
            target_object: Optional specific object to search for (e.g., "person", "cup", "chair")

        Returns:
            tuple: (result_message, result_status)
        """
        if not self.model:
            self.logger.error("[ScanForObjects] GEMINI_API_KEY not set")
            return "Gemini API key not configured", SkillResult.FAILURE

        self.logger.info(
            f"\033[96m[BrainClient] Starting 360° object scan"
            f"{f' for {target_object}' if target_object else ''}\033[0m"
        )
        self._send_feedback(f"Starting scan{f' for {target_object}' if target_object else ''}...")

        # Calculate rotation per step in radians
        rotation_per_snapshot = (2 * math.pi) / self.num_snapshots

        all_detections = []
        target_found = False
        target_directions = []

        try:
            for i in range(self.num_snapshots):
                # Calculate approximate direction (0° = front, increases counter-clockwise)
                direction_deg = (i * 360 / self.num_snapshots) % 360
                direction_name = self._direction_name(direction_deg)
                
                self.logger.info(f"[ScanForObjects] Snapshot {i+1}/{self.num_snapshots} at {direction_name}")
                
                # Check if we have an image
                if not self.image:
                    self.logger.warn(f"[ScanForObjects] No image available at snapshot {i+1}")
                else:
                    # Send to Gemini for detection
                    detections = self._detect_objects(self.image, target_object)
                    
                    if detections:
                        for det in detections:
                            det['direction'] = direction_name
                            det['direction_deg'] = direction_deg
                            all_detections.append(det)
                            
                            # Check if target found
                            if target_object and target_object.lower() in det['class'].lower():
                                target_found = True
                                target_directions.append(direction_name)
                                self._send_feedback(f"Found {target_object} at {direction_name}!")
                
                # Rotate to next position (skip rotation after last snapshot)
                if i < self.num_snapshots - 1:
                    # Use mobility interface for precise blocking rotation
                    self.mobility.rotate(rotation_per_snapshot)

        except Exception as e:
            self.logger.error(f"[ScanForObjects] Error during scan: {e}")
            return f"Scan failed: {str(e)}", SkillResult.FAILURE

        # Summarize results
        if target_object:
            if target_found:
                directions_str = ", ".join(set(target_directions))
                result = f"Found {target_object} at: {directions_str}"
                self._send_feedback(result)
                return result, SkillResult.SUCCESS
            else:
                result = f"{target_object} not found during scan"
                self._send_feedback(result)
                return result, SkillResult.SUCCESS  # Not finding is still a successful scan
        else:
            # Report all unique objects found
            unique_objects = {}
            for det in all_detections:
                obj_class = det['class']
                if obj_class not in unique_objects:
                    unique_objects[obj_class] = []
                unique_objects[obj_class].append(det['direction'])
            
            if unique_objects:
                summary_parts = []
                for obj, directions in unique_objects.items():
                    unique_dirs = list(set(directions))
                    summary_parts.append(f"{obj} ({', '.join(unique_dirs)})")
                result = f"Objects found: {'; '.join(summary_parts)}"
            else:
                result = "No objects detected during scan"
            
            self._send_feedback(result)
            return result, SkillResult.SUCCESS

    def _detect_objects(self, image_b64: str, target_object: str = None) -> list:
        """Send image to Gemini and return detected objects."""
        self._snapshot_counter += 1
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Debug: save image
        self.debug_dir.mkdir(exist_ok=True)
        try:
            image_bytes = base64.b64decode(image_b64)
            image_path = self.debug_dir / f"snapshot_{self._snapshot_counter:02d}_{timestamp}.jpg"
            with open(image_path, "wb") as f:
                f.write(image_bytes)
            self.logger.info(f"[ScanForObjects] Saved image to {image_path}")
        except Exception as e:
            self.logger.warn(f"[ScanForObjects] Failed to save image: {e}")
        
        if not self.model:
            return []
        
        try:
            # Build prompt for structured output
            if target_object:
                prompt = f"""Look at this image and find any "{target_object}" objects.
Respond with ONLY a JSON array of detected objects. Each object should have:
- "class": the object type
- "confidence": your confidence 0.0-1.0
- "position": "left", "center", or "right" side of image

If no {target_object} is found, return an empty array: []
Example: [{{"class": "cup", "confidence": 0.9, "position": "center"}}]"""
            else:
                prompt = """List all distinct objects you can see in this image.
Respond with ONLY a JSON array. Each object should have:
- "class": the object type (be specific, e.g. "office chair" not just "furniture")
- "confidence": your confidence 0.0-1.0  
- "position": "left", "center", or "right" side of image

Example: [{"class": "laptop", "confidence": 0.95, "position": "center"}, {"class": "coffee mug", "confidence": 0.8, "position": "right"}]"""
            
            # Create image part for Gemini
            image_part = {
                "mime_type": "image/jpeg",
                "data": image_b64
            }
            
            # Call Gemini
            response = self.model.generate_content(
                [prompt, image_part],
                generation_config=genai.GenerationConfig(
                    response_mime_type="application/json",
                )
            )
            
            # Parse response
            response_text = response.text.strip()
            try:
                detections = json.loads(response_text)
                if not isinstance(detections, list):
                    detections = []
            except json.JSONDecodeError:
                self.logger.warn(f"[ScanForObjects] Failed to parse JSON: {response_text}")
                detections = []
            
            return detections

        except Exception as e:
            self.logger.error(f"[ScanForObjects] Detection error: {e}")
            return []

    def _direction_name(self, degrees: float) -> str:
        """Convert degrees to cardinal direction name."""
        # Normalize to 0-360
        degrees = degrees % 360
        
        if degrees < 22.5 or degrees >= 337.5:
            return "front"
        elif degrees < 67.5:
            return "front-left"
        elif degrees < 112.5:
            return "left"
        elif degrees < 157.5:
            return "back-left"
        elif degrees < 202.5:
            return "back"
        elif degrees < 247.5:
            return "back-right"
        elif degrees < 292.5:
            return "right"
        else:
            return "front-right"

    def cancel(self):
        """Stop the rotation."""
        self.logger.info("[ScanForObjects] Cancellation requested, stopping rotation")
        if self.mobility:
            self.mobility.send_cmd_vel(0.0, 0.0)
