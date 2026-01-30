"""
Intent Parser - Natural language to DesignSpec.
Detects domain (gripper/mechanism/enclosure), scale, material, params,
dimensions, force, environment, target object.
"""

import re
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple


@dataclass
class DesignSpec:
    """Structured design specification from natural language."""
    domain: str  # "gripper" | "mechanism" | "enclosure"
    scale: str   # "small" | "medium" | "large"
    material: Optional[str] = None
    params: Dict = field(default_factory=dict)
    target_object: Optional[str] = None
    raw_intent: str = ""

    def to_dict(self) -> Dict:
        return {
            "domain": self.domain,
            "scale": self.scale,
            "material": self.material,
            "params": self.params,
            "target_object": self.target_object,
            "raw_intent": self.raw_intent,
        }


class IntentParser:
    """Parse natural language intent into DesignSpec."""

    DOMAIN_KEYWORDS = {
        "gripper": [
            "gripper", "grip", "grasp", "pinch", "finger", "fingers",
            "grab", "clamp", "hand", "soft gripper", "robotic hand",
        ],
        "mechanism": [
            "hinge", "linkage", "cam", "slider", "mechanism", "joint",
            "lever", "crank", "pivot", "actuator mechanism", "four bar",
            "gear", "universal joint",
        ],
        "enclosure": [
            "enclosure", "box", "mount", "housing", "bracket", "case",
            "cover", "shell", "frame", "holder", "rack mount",
        ],
    }

    SCALE_KEYWORDS = {
        "small": ["small", "tiny", "mini", "compact", "micro", "little"],
        "large": ["large", "big", "heavy", "full-size", "macro"],
    }

    MATERIAL_KEYWORDS = {
        "silicone": ["silicone", "soft", "compliant", "dragonskin", "ecoflex"],
        "tpu": ["tpu", "flexible", "plastic"],
        "rigid": ["rigid", "plastic", "abs", "metal", "aluminum"],
    }

    # Material hints: soft/rigid/flexible/compliant refine material
    MATERIAL_HINTS = ["soft", "rigid", "flexible", "compliant"]

    TARGET_OBJECTS = [
        "egg", "eggs", "fruit", "apple", "tomato", "ball", "cylinder",
        "bottle", "cup", "tool", "object", "parts", "food", "electronics",
    ]

    # Explicit finger count
    NUM_FINGERS_DIGIT = re.compile(r"\b([2-5])\s*finger(s)?\b", re.IGNORECASE)
    WORD_TO_NUM = {"two": 2, "three": 3, "four": 4, "five": 5}
    NUM_FINGERS_WORD = re.compile(
        r"\b(two|three|four|five)\s*finger(s)?\b", re.IGNORECASE
    )

    # Dimensions: "5cm long", "10mm thick", "2 inch"
    DIM_PATTERN = re.compile(
        r"\b(\d+(?:\.\d+)?)\s*(mm|cm|m|inch|in)\s*(long|wide|thick|high|length|width|height|diameter)\b",
        re.IGNORECASE,
    )
    DIM_UNITS_MM = {"mm": 1, "cm": 10, "m": 1000, "inch": 25.4, "in": 25.4}

    # Force: gentle, strong, delicate, heavy duty
    FORCE_KEYWORDS = {
        "gentle": ["gentle", "light", "delicate", "soft touch"],
        "medium": ["medium", "moderate", "normal"],
        "strong": ["strong", "heavy duty", "heavy-duty", "high force", "firm"],
    }

    # Environment
    ENV_KEYWORDS = {
        "dry": ["dry", "clean", "office"],
        "wet": ["wet", "water", "washable"],
        "oily": ["oily", "greasy", "lubricated"],
        "dusty": ["dusty", "dirty", "outdoor"],
        "clean_room": ["clean room", "cleanroom", "sterile"],
    }

    def parse(self, text: str) -> DesignSpec:
        text = (text or "").strip().lower()
        raw = text

        domain = self._detect_domain(text)
        scale = self._detect_scale(text)
        material = self._detect_material(text)
        target_object = self._detect_target_object(text)
        params = self._extract_params(text, domain)

        # Dimensions, force, environment (all domains)
        dims = self._extract_dimensions(text)
        if dims:
            params["dimensions_mm"] = dims
        force = self._extract_force_requirement(text)
        if force:
            params["force_requirement"] = force
        env = self._extract_environment(text)
        if env:
            params["environment"] = env
        # Material hints refine material
        for hint in self.MATERIAL_HINTS:
            if hint in text and "material" not in params:
                params["material_hint"] = hint

        return DesignSpec(
            domain=domain,
            scale=scale,
            material=material,
            params=params,
            target_object=target_object,
            raw_intent=raw,
        )

    def _detect_domain(self, text: str) -> str:
        for domain, keywords in self.DOMAIN_KEYWORDS.items():
            for kw in keywords:
                if kw in text:
                    return domain
        return "gripper"

    def _detect_scale(self, text: str) -> str:
        for scale, keywords in self.SCALE_KEYWORDS.items():
            for kw in keywords:
                if kw in text:
                    return scale
        return "medium"

    def _detect_material(self, text: str) -> Optional[str]:
        for material, keywords in self.MATERIAL_KEYWORDS.items():
            for kw in keywords:
                if kw in text:
                    return material
        return None

    def _detect_target_object(self, text: str) -> Optional[str]:
        for obj in self.TARGET_OBJECTS:
            if obj in text:
                return obj
        m = re.search(r"\bfor\s+(\w+)", text)
        if m:
            return m.group(1).lower()
        return None

    def _extract_num_fingers(self, text: str) -> Optional[int]:
        m = self.NUM_FINGERS_DIGIT.search(text)
        if m:
            return int(m.group(1))
        m = self.NUM_FINGERS_WORD.search(text)
        if m:
            return self.WORD_TO_NUM.get(m.group(1).lower())
        return None

    def _extract_dimensions(self, text: str) -> Optional[Dict[str, float]]:
        """Extract explicit dimensions in mm. E.g. '5cm long' -> {'length_mm': 50}."""
        out: Dict[str, float] = {}
        for m in self.DIM_PATTERN.finditer(text):
            val = float(m.group(1)) * self.DIM_UNITS_MM.get(
                m.group(2).lower(), 1
            )
            key = m.group(3).lower()
            if key in ("long", "length"):
                out["length_mm"] = val
            elif key in ("wide", "width"):
                out["width_mm"] = val
            elif key in ("thick", "high", "height"):
                out["height_mm"] = val
            elif key == "diameter":
                out["diameter_mm"] = val
        return out if out else None

    def _extract_force_requirement(self, text: str) -> Optional[str]:
        for level, keywords in self.FORCE_KEYWORDS.items():
            for kw in keywords:
                if kw in text:
                    return level
        return None

    def _extract_environment(self, text: str) -> Optional[str]:
        for env, keywords in self.ENV_KEYWORDS.items():
            for kw in keywords:
                if kw in text:
                    return env
        return None

    def _extract_params(self, text: str, domain: str) -> Dict:
        params: Dict = {}
        if domain == "gripper":
            # Gesture detection (order: specific phrases first)
            if any(x in text for x in ["tripod", "three point", "pen grip"]):
                params["gesture"] = "tripod"
                params["num_fingers"] = 3
            elif any(x in text for x in ["ball grip", "spherical", "round object"]):
                params["gesture"] = "spherical"
                params["num_fingers"] = 5
            elif any(x in text for x in ["key grip", "lateral pinch", "card holder", "lateral"]):
                params["gesture"] = "lateral"
                params["num_fingers"] = 2
            elif any(x in text for x in ["hook grip", "carry bags", "hanging"]):
                params["gesture"] = "hook"
                params["num_fingers"] = 3
            elif any(x in text for x in ["power grip", "cylindrical grip", "wrap around", "power", "wrap"]):
                params["gesture"] = "power"
                params["num_fingers"] = 4
            elif "pinch" in text:
                params["gesture"] = "pinch"
                params["num_fingers"] = 2
            else:
                params["gesture"] = "pinch"
                params["num_fingers"] = 2
            explicit = self._extract_num_fingers(text)
            if explicit is not None:
                params["num_fingers"] = explicit
        elif domain == "mechanism":
            if "four bar" in text or "4 bar" in text or "linkage" in text:
                params["type"] = "four_bar_linkage"
            elif "cam" in text or "follower" in text:
                params["type"] = "cam_follower"
            elif "slider" in text:
                params["type"] = "slider"
            elif "gear" in text:
                params["type"] = "gear_pair"
            elif "universal" in text:
                params["type"] = "universal_joint"
            elif "hinge" in text:
                params["type"] = "hinge"
            else:
                params["type"] = "hinge"
        elif domain == "enclosure":
            if "rack mount" in text or "19" in text:
                params["type"] = "rack_mount"
            elif "bracket" in text:
                params["type"] = "bracket"
            elif "housing" in text:
                params["type"] = "housing"
            elif "mount" in text:
                params["type"] = "mount"
            elif "box" in text:
                params["type"] = "box"
            else:
                params["type"] = "box"
            if "waterproof" in text or "water proof" in text:
                params["waterproof"] = True
            if "ventilated" in text or "ventilation" in text:
                params["ventilated"] = True
            for pat in ["m3", "m4", "1/4-20", "1/4 20"]:
                if pat in text:
                    params["mounting_pattern"] = "M3" if pat == "m3" else "M4" if pat == "m4" else "1/4-20"
                    break
            if "snap" in text or "lid" in text:
                params["snap_fit_lid"] = True
        return params
