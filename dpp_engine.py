"""
D++ Universal Rule Engine (simulation-first)
Save as: dpp_engine.py

Run:
    python dpp_engine.py farm.dpp

This version is a safer prototype, not a certified controller for physical equipment.
It simulates sensor readings and device state. It does NOT drive GPIO or real hardware.
"""

import re
import sys
import time
from typing import Any, Dict, List, Optional, Tuple


# Change this sequence to test different scenarios.
# None means "no packet arrived during this evaluation interval".
SIMULATED_STREAM: List[Optional[float]] = [
    # A: one invalid packet interrupts the activation timer
    35.0, 28.0, -99.0,
    # Valid low readings continue long enough to activate the pump
    27.0, 28.0, 29.0, 29.0,
    # D: cutoff flicker: 52 starts cutoff, 48 cancels it, then >50 persists
    52.0, 48.0, 55.0, 56.0, 57.0,
    # B: invalid readings persist long enough to trigger sensor fault
    -99.0, -98.0, -97.0, -96.0,
    # E: two valid readings recover the sensor
    40.0, 41.0,
    # C: no packets for at least 4 seconds triggers timeout
    None, None, None, None, None,
]


COMPARATORS = {
    "IS GREATER THAN": lambda value, threshold: value > threshold,
    "IS LESS THAN": lambda value, threshold: value < threshold,
    "IS EQUAL TO": lambda value, threshold: value == threshold,
    "IS GREATER THAN OR EQUAL TO": lambda value, threshold: value >= threshold,
    "IS LESS THAN OR EQUAL TO": lambda value, threshold: value <= threshold,
}


class DPlusPlusCompileError(ValueError):
    """Raised when the D++ file contains an unsupported or invalid rule."""


class UniversalDPlusPlusEngine:
    def __init__(self, filepath: str):
        self.filepath = filepath
        self.registry: Dict[str, Dict[str, str]] = {}
        self.settings: Dict[str, str] = {}
        self.rule: Dict[str, Any] = {}

        self.device_state = "OFF"
        self.invalid_since: Optional[float] = None
        self.last_valid_value: Optional[float] = None
        self.last_packet_time: Optional[float] = None
        self.sensor_fault = False
        self.recovery_readings = 0

        self.activation_since: Optional[float] = None
        self.cutoff_since: Optional[float] = None

    @staticmethod
    def _number(value: str) -> float:
        try:
            return float(value)
        except ValueError as exc:
            raise DPlusPlusCompileError(f"Expected a numeric value, got {value!r}") from exc

    def compile(self) -> bool:
        try:
            with open(self.filepath, "r", encoding="utf-8") as file:
                source = file.read()
        except OSError as exc:
            print(f"❌ Could not read D++ file '{self.filepath}': {exc}")
            return False

        # Remove comments, but preserve line-oriented declarations.
        lines = []
        for raw_line in source.splitlines():
            line = raw_line.strip()
            if not line or line.startswith("//") or line.startswith("#"):
                continue
            lines.append(line)
        clean_source = "\n".join(lines)

        print("🧬 [D++ ENGINE] Compiling rules...")

        # RESOURCE <type> <name> IDENTIFIED BY <address>
        # RESOURCE <type> <name> LOCATED AT <address>
        resource_re = re.compile(
            r"""^RESOURCE\s+(\w+)\s+(\w+)\s+(IDENTIFIED\s+BY|LOCATED\s+AT)\s+(.+?)\s*$""",
            re.IGNORECASE,
        )
        for line in lines:
            match = resource_re.match(line)
            if match:
                resource_type, resource_name, _, address = match.groups()
                self.registry[resource_name] = {
                    "type": resource_type.lower(),
                    "address": address.strip().strip("\"'"),
                }
                continue

            set_match = re.match(r"^SET\s+(\w+)\s*=\s*(.+)$", line, re.IGNORECASE)
            if set_match:
                key, value = set_match.groups()
                self.settings[key.lower()] = value.strip()

        # Core rule grammar. Comparator alternatives are ordered longest first.
        comparator_pattern = (
            r"IS GREATER THAN OR EQUAL TO|IS LESS THAN OR EQUAL TO|"
            r"IS GREATER THAN|IS LESS THAN|IS EQUAL TO"
        )
        metric_pattern = r"([A-Za-z_][\w.]*)"
        rule_pattern = re.compile(
            rf"WHEN\s+{metric_pattern}\s+({comparator_pattern})\s+"
            r"(-?\d+(?:\.\d+)?)\s+FOR\s+(\d+(?:\.\d+)?)\s+seconds\s+"
            r"TRIGGER\s+([A-Za-z_]\w*)\s+(ON|OFF)",
            re.IGNORECASE | re.DOTALL,
        )
        rule_match = rule_pattern.search(clean_source)
        if not rule_match:
            print("❌ Compilation failure: expected a valid WHEN...FOR...TRIGGER rule.")
            return False

        metric, op, threshold, duration, device, target_state = rule_match.groups()
        self.rule = {
            "metric": metric,
            "op": op.upper(),
            "thresh": float(threshold),
            "duration": float(duration),
            "device": device,
            "target_state": target_state.upper(),
            "off_op": None,
            "off_thresh": None,
            "val_min": None,
            "val_max": None,
            "retry_limit": 0.0,
            "timeout": None,
            "stabilize": 0.0,
        }

        until_match = re.search(
            rf"UNTIL\s+{metric_pattern}\s+({comparator_pattern})\s+"
            r"(-?\d+(?:\.\d+)?)",
            clean_source,
            re.IGNORECASE,
        )
        if until_match:
            _, off_op, off_threshold = until_match.groups()
            self.rule["off_op"] = off_op.upper()
            self.rule["off_thresh"] = float(off_threshold)

        validate_match = re.search(
            rf"VALIDATE\s+{metric_pattern}\s+BETWEEN\s+"
            r"(-?\d+(?:\.\d+)?)\s+AND\s+(-?\d+(?:\.\d+)?)",
            clean_source,
            re.IGNORECASE,
        )
        if validate_match:
            _, low, high = validate_match.groups()
            self.rule["val_min"] = float(low)
            self.rule["val_max"] = float(high)
            if float(low) > float(high):
                print("❌ Compilation failure: VALIDATE lower bound exceeds upper bound.")
                return False

        retry_match = re.search(
            r"RETRY\s+LIMIT\s*=\s*(\d+(?:\.\d+)?)\s+SECONDS?",
            clean_source,
            re.IGNORECASE,
        )
        if retry_match:
            self.rule["retry_limit"] = float(retry_match.group(1))

        timeout_match = re.search(
            rf"TIMEOUT\s+{metric_pattern}\s+AFTER\s+(\d+(?:\.\d+)?)\s+seconds",
            clean_source,
            re.IGNORECASE,
        )
        if timeout_match:
            self.rule["timeout"] = float(timeout_match.group(2))

        stabilize_match = re.search(
            r"STABILIZE\s+FOR\s+(\d+(?:\.\d+)?)\s+seconds",
            clean_source,
            re.IGNORECASE,
        )
        if stabilize_match:
            self.rule["stabilize"] = float(stabilize_match.group(1))

        interval_text = self.settings.get("evaluation_interval", "1 second")
        interval_match = re.fullmatch(
            r"(\d+(?:\.\d+)?)\s*(seconds?|s)", interval_text, re.IGNORECASE
        )
        if not interval_match:
            print(f"❌ Unsupported evaluation_interval: {interval_text!r}")
            return False
        self.evaluation_interval = float(interval_match.group(1))
        if self.evaluation_interval <= 0:
            print("❌ evaluation_interval must be greater than zero.")
            return False

        if self.rule["device"] not in self.registry:
            print(
                f"⚠️ Device '{self.rule['device']}' was not declared as a RESOURCE; "
                "using simulated hardware mode."
            )

        print(
            "✔ Compilation successful: "
            f"metric={self.rule['metric']}, device={self.rule['device']}, "
            f"activation={self.rule['duration']}s, "
            f"stabilization={self.rule['stabilize']}s"
        )
        return True

    def _set_device(self, new_state: str, reason: str) -> None:
        if new_state == self.device_state:
            return
        self.device_state = new_state
        print(
            f"   🔧 [D++ DEVICE ACTION] {reason} -> "
            f"Set '{self.rule['device']}' to {self.device_state}"
        )

    def _enter_sensor_fault(self, reason: str) -> None:
        if not self.sensor_fault:
            self.sensor_fault = True
            self.recovery_readings = 0
            print(f"   🚨 [SENSOR FAULT] {reason}")
            # Simulation safety policy: stop irrigation while sensor health is unknown.
            self._set_device("OFF", "Safe-state policy: sensor data is unreliable")
        else:
            print(f"   ⚠️ [SENSOR FAULT CONTINUES] {reason}")

    def _check_cutoff(self, value: float, now: float) -> None:
        if self.device_state != "ON" or not self.rule["off_op"]:
            self.cutoff_since = None
            return

        comparator = COMPARATORS[self.rule["off_op"]]
        cutoff_met = comparator(value, self.rule["off_thresh"])

        if not cutoff_met:
            if self.cutoff_since is not None:
                print("   ↩️ Cutoff condition cleared; stabilization timer reset.")
            self.cutoff_since = None
            return

        if self.cutoff_since is None:
            self.cutoff_since = now
            print(
                f"   ⏳ Cutoff condition detected; waiting "
                f"{self.rule['stabilize']:g}s for stabilization."
            )

        elapsed = now - self.cutoff_since
        if elapsed + 1e-6 >= self.rule["stabilize"]:
            self._set_device("OFF", "Cutoff condition remained true for required duration")
            self.cutoff_since = None
            self.activation_since = None

    def execute(self, simulated_stream: Optional[List[Optional[float]]] = None) -> None:
        stream = SIMULATED_STREAM if simulated_stream is None else simulated_stream
        if not stream:
            print("No simulated readings configured.")
            return

        self.device_state = "OFF"
        self.invalid_since = None
        self.last_valid_value = None
        self.last_packet_time = None
        self.sensor_fault = False
        self.recovery_readings = 0
        self.activation_since = None
        self.cutoff_since = None

        print("\n--- D++ SIMULATION RUNTIME ---")
        print("Note: this run simulates hardware; it does not control a physical pump.")
        start = time.monotonic()
        last_evaluation = start
        packet_seen = False

        for index, value in enumerate(stream):
            # Keep evaluation ticks at the configured real-time interval.
            if index > 0:
                target = start + index * self.evaluation_interval
                delay = target - time.monotonic()
                if delay > 0:
                    time.sleep(delay)

            now = time.monotonic()
            elapsed_clock = now - start
            print(
                f"\n[Clock: {elapsed_clock:.1f}s] "
                f"Evaluating '{self.rule['metric']}'"
                + (f": {value}" if value is not None else ": NO PACKET")
            )

            # Timeout is based on the last packet arrival, not the loop interval.
            if (
                self.rule["timeout"] is not None
                and packet_seen
                and self.last_packet_time is not None
                and now - self.last_packet_time + 1e-6 >= self.rule["timeout"]
            ):
                self._enter_sensor_fault(
                    f"No sensor packet for {now - self.last_packet_time:.1f}s "
                    f"(timeout {self.rule['timeout']:g}s)."
                )

            # None means no packet arrived. Do not refresh the last-packet timestamp.
            if value is None:
                print("   ⚠️ No packet received; last-packet timer continues.")
                # A condition cannot count as continuously verified without a reading.
                if self.activation_since is not None:
                    print("   ↩️ Missing packet interrupted activation timer; timer reset.")
                if self.cutoff_since is not None:
                    print("   ↩️ Missing packet interrupted cutoff stabilization; timer reset.")
                self.activation_since = None
                self.cutoff_since = None
                last_evaluation = now
                continue

            packet_seen = True
            self.last_packet_time = now

            # Validate incoming data before it can influence rules.
            low = self.rule["val_min"]
            high = self.rule["val_max"]
            valid = low is None or high is None or low <= value <= high

            if not valid:
                print(
                    f"   ⚠️ [TELEMETRY REJECTED] Out-of-bounds packet dropped ({value})."
                )
                if self.invalid_since is None:
                    self.invalid_since = now
                if (
                    self.rule["retry_limit"] > 0
                    and now - self.invalid_since + 1e-6 >= self.rule["retry_limit"]
                ):
                    self._enter_sensor_fault(
                        f"Invalid readings persisted for "
                        f"{now - self.invalid_since:.1f}s."
                    )
                # Invalid data must not count toward a continuous control condition.
                if self.activation_since is not None:
                    print("   ↩️ Invalid reading interrupted activation timer; timer reset.")
                if self.cutoff_since is not None:
                    print("   ↩️ Invalid reading interrupted cutoff stabilization; timer reset.")
                self.activation_since = None
                self.cutoff_since = None
                last_evaluation = now
                continue

            # A valid reading breaks the consecutive-invalid interval.
            self.invalid_since = None
            self.last_valid_value = value

            if self.sensor_fault:
                self.recovery_readings += 1
                print(
                    f"   🩺 Valid recovery reading "
                    f"{self.recovery_readings}/2."
                )
                if self.recovery_readings < 2:
                    print("   ⛔ Control remains blocked pending recovery confirmation.")
                    last_evaluation = now
                    continue
                self.sensor_fault = False
                self.recovery_readings = 0
                self.activation_since = None
                self.cutoff_since = None
                print("   ✅ [SENSOR RECOVERED] Two consecutive valid readings received.")

            # Rule timing uses monotonic elapsed time rather than sample counts.
            if self.device_state == "OFF":
                activation_condition = COMPARATORS[self.rule["op"]](
                    value, self.rule["thresh"]
                )
                if activation_condition:
                    if self.activation_since is None:
                        self.activation_since = now
                        print("   ⏳ Activation condition started.")
                    sustained = now - self.activation_since
                    if sustained + 1e-6 >= self.rule["duration"]:
                        self._set_device(
                            self.rule["target_state"],
                            f"Activation condition sustained for {sustained:.1f}s",
                        )
                        self.activation_since = None
                        self.cutoff_since = None
                else:
                    if self.activation_since is not None:
                        print("   ↩️ Activation condition cleared; timer reset.")
                    self.activation_since = None

            if not self.sensor_fault:
                self._check_cutoff(value, now)

            print(
                f"   Status State: {self.rule['device']} = {self.device_state}"
                + (" [SENSOR FAULT]" if self.sensor_fault else "")
            )
            last_evaluation = now

        print("\n--- SIMULATION FINISHED ---")
        print(f"Final simulated device state: {self.rule['device']} = {self.device_state}")
        if self.sensor_fault:
            print("Final sensor status: FAULT (manual inspection recommended).")
        else:
            print("Final sensor status: no active fault.")
        print("Reminder: simulated state is not proof of physical device operation.")


if __name__ == "__main__":
    target = sys.argv[1] if len(sys.argv) > 1 else "farm.dpp"
    engine = UniversalDPlusPlusEngine(target)
    if engine.compile():
        engine.execute()
