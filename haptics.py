#!/usr/bin/env python3
"""
Logitech MX Master 4 – Haptic Sequence Player
==============================================
Communicates with the mouse over raw HID using the HID++ 2.0 protocol to
trigger the built-in haptic motor (feature 0x19B0).

Usage
-----
  python haptics.py --list                       # List detected Logitech HID devices
  python haptics.py                              # Interactive demo (all waveforms)
  python haptics.py --waveform HAPPY_ALERT       # Play a single named waveform
  python haptics.py --waveform 5                 # Play waveform by numeric ID
  python haptics.py --sequence heartbeat         # Play a predefined sequence
  python haptics.py --level 75                   # Set haptic feedback intensity (0-100)
  python haptics.py --sequence heartbeat --level 100  # Combined

Requirements
------------
  pip install hidapi

Supported waveforms (as discovered from Solaar PR #3024 / hidpp20_constants.py):
  SHARP_STATE_CHANGE  = 0x00      HAPPY_ALERT     = 0x05
  DAMP_STATE_CHANGE   = 0x01      ANGRY_ALERT     = 0x06
  SHARP_COLLISION     = 0x02      COMPLETED       = 0x07
  DAMP_COLLISION      = 0x03      SQUARE          = 0x08
  SUBTLE_COLLISION    = 0x04      WAVE            = 0x09
                                  FIREWORK        = 0x0A
                                  MAD             = 0x0B
                                  KNOCK           = 0x0C
                                  JINGLE          = 0x0D
                                  RINGING         = 0x0E
"""

import argparse
import sys
import time
from enum import IntEnum
from typing import Optional

try:
    import hid
except ImportError:
    print(
        "Error: 'hidapi' is not installed.\n"
        "Install it with:  pip install hidapi",
        file=sys.stderr,
    )
    sys.exit(1)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

LOGITECH_VID = 0x046D  # Logitech USB vendor ID

# HID++ report IDs
SHORT_REPORT  = 0x10   # 7-byte  short HID++ report
LONG_REPORT   = 0x11   # 20-byte long  HID++ report

# HID++ 2.0 feature IDs
FEATURE_ROOT  = 0x0000  # IRoot – always at feature index 0
FEATURE_HAPTIC = 0x19B0  # Haptic feedback

# Typical device indices for Logitech receivers
DEVICE_INDEX_BOLT      = 0x01   # Most common Bolt receiver channel
DEVICE_INDEX_BLUETOOTH = 0xFF   # Direct Bluetooth connection

# HID++ 2.0 function IDs for the HAPTIC feature (byte 3 high nibble)
HAPTIC_FN_GET_CAPABILITIES = 0x00   # Function 0  → cmd byte 0x00
HAPTIC_FN_GET_STATE        = 0x10   # Function 1  → cmd byte 0x10
HAPTIC_FN_SET_STATE        = 0x20   # Function 2  → cmd byte 0x20
HAPTIC_FN_PLAY_WAVEFORM    = 0x40   # Function 4  → cmd byte 0x40

# Software ID appended to function byte (identifies our app)
SW_ID = 0x01


# ---------------------------------------------------------------------------
# Haptic waveform definitions (from Solaar / Logitech firmware)
# ---------------------------------------------------------------------------

class HapticWaveForm(IntEnum):
    SHARP_STATE_CHANGE  = 0x00
    DAMP_STATE_CHANGE   = 0x01
    SHARP_COLLISION     = 0x02
    DAMP_COLLISION      = 0x03
    SUBTLE_COLLISION    = 0x04
    HAPPY_ALERT         = 0x05
    ANGRY_ALERT         = 0x06
    COMPLETED           = 0x07
    SQUARE              = 0x08
    WAVE                = 0x09
    FIREWORK            = 0x0A
    MAD                 = 0x0B
    KNOCK               = 0x0C
    JINGLE              = 0x0D
    RINGING             = 0x0E


# Map of names → waveform IDs (case-insensitive lookup)
WAVEFORM_BY_NAME: dict[str, HapticWaveForm] = {
    w.name.upper(): w for w in HapticWaveForm
}


# ---------------------------------------------------------------------------
# Predefined haptic sequences  (list of (waveform, delay_seconds) tuples)
# ---------------------------------------------------------------------------

PREDEFINED_SEQUENCES: dict[str, list[tuple[HapticWaveForm, float]]] = {
    "demo": [(w, 0.4) for w in HapticWaveForm],

    "heartbeat": [
        (HapticWaveForm.KNOCK, 0.15),
        (HapticWaveForm.KNOCK, 0.45),
        (HapticWaveForm.KNOCK, 0.15),
        (HapticWaveForm.KNOCK, 0.80),
    ],

    "success": [
        (HapticWaveForm.HAPPY_ALERT, 0.2),
        (HapticWaveForm.COMPLETED,   0.0),
    ],

    "error": [
        (HapticWaveForm.ANGRY_ALERT, 0.15),
        (HapticWaveForm.ANGRY_ALERT, 0.15),
        (HapticWaveForm.ANGRY_ALERT, 0.0),
    ],

    "jingle_alert": [
        (HapticWaveForm.JINGLE,       0.0),
    ],

    "fireworks": [
        (HapticWaveForm.FIREWORK,     0.3),
        (HapticWaveForm.FIREWORK,     0.25),
        (HapticWaveForm.FIREWORK,     0.2),
        (HapticWaveForm.FIREWORK,     0.0),
    ],

    "triple_knock": [
        (HapticWaveForm.KNOCK, 0.18),
        (HapticWaveForm.KNOCK, 0.18),
        (HapticWaveForm.KNOCK, 0.0),
    ],

    "wave_sweep": [
        (HapticWaveForm.WAVE, 0.3),
        (HapticWaveForm.WAVE, 0.25),
        (HapticWaveForm.WAVE, 0.2),
        (HapticWaveForm.WAVE, 0.0),
    ],
}


# ---------------------------------------------------------------------------
# MXMaster4 class
# ---------------------------------------------------------------------------

class MXMaster4:
    """
    Low-level interface to the Logitech MX Master 4 haptic motor
    via HID++ 2.0 over raw HID.
    """

    def __init__(self, device: hid.device, device_index: int = DEVICE_INDEX_BOLT):
        self._dev = device
        self.device_index = device_index
        self._haptic_feature_index: Optional[int] = None

    # ------------------------------------------------------------------
    # HID++ messaging helpers
    # ------------------------------------------------------------------

    def _send_long(self, feature_index: int, function: int, params: bytes = b"") -> Optional[bytes]:
        """
        Send a 20-byte (long) HID++ 2.0 request and return the reply,
        or None on timeout / error.
        """
        cmd_byte = function | SW_ID
        payload = bytes([
            LONG_REPORT,
            self.device_index,
            feature_index,
            cmd_byte,
        ]) + params

        # Pad to 20 bytes (report ID + 19 payload bytes)
        payload = payload[:20].ljust(20, b"\x00")

        try:
            self._dev.write(list(payload))
            # Read up to ~500 ms for a matching reply
            for _ in range(50):
                reply = self._dev.read(20, timeout_ms=10)
                if not reply:
                    continue
                # Filter by report ID and feature index
                if reply[0] == LONG_REPORT and reply[2] == feature_index:
                    return bytes(reply)
        except Exception as exc:
            print(f"[WARN] HID write/read error: {exc}", file=sys.stderr)
        return None

    # ------------------------------------------------------------------
    # HID++ 2.0 feature resolution
    # ------------------------------------------------------------------

    def _get_feature_index(self, feature_id: int) -> Optional[int]:
        """
        Query the Root Feature (index 0) to resolve a Feature ID → Feature Index.
        Returns the 8-bit index on success, None otherwise.
        """
        msb = (feature_id >> 8) & 0xFF
        lsb = feature_id & 0xFF
        reply = self._send_long(
            feature_index=0x00,
            function=0x00,            # IRoot::getFeature
            params=bytes([msb, lsb]),
        )
        if reply and len(reply) >= 5 and reply[4] != 0:
            return reply[4]
        return None

    def get_haptic_feature_index(self) -> Optional[int]:
        """Resolve (and cache) the HAPTIC feature index."""
        if self._haptic_feature_index is None:
            self._haptic_feature_index = self._get_feature_index(FEATURE_HAPTIC)
        return self._haptic_feature_index

    # ------------------------------------------------------------------
    # Haptic control
    # ------------------------------------------------------------------

    def get_capabilities(self) -> Optional[bytes]:
        """
        Call HAPTIC function 0 (getCapabilities).
        Returns raw reply bytes.
        """
        idx = self.get_haptic_feature_index()
        if idx is None:
            return None
        return self._send_long(idx, HAPTIC_FN_GET_CAPABILITIES)

    def get_state(self) -> Optional[bytes]:
        """Call HAPTIC function 1 (getState)."""
        idx = self.get_haptic_feature_index()
        if idx is None:
            return None
        return self._send_long(idx, HAPTIC_FN_GET_STATE)

    def set_haptic_level(self, level: int) -> bool:
        """
        Set haptic feedback strength.
        level = 0 → disabled
        level = 1..100 → enabled at that percentage
        """
        idx = self.get_haptic_feature_index()
        if idx is None:
            print("[ERROR] HAPTIC feature not found on this device.", file=sys.stderr)
            return False

        level = max(0, min(100, level))
        if level == 0:
            params = bytes([0x00, 0x32])   # disable, 50 % stored
        else:
            params = bytes([0x01, level])  # enable, level%

        reply = self._send_long(idx, HAPTIC_FN_SET_STATE, params)
        return reply is not None

    def play_waveform(self, waveform: HapticWaveForm | int) -> bool:
        """
        Play a single haptic waveform (HAPTIC function 4 / cmd 0x40).
        """
        idx = self.get_haptic_feature_index()
        if idx is None:
            print("[ERROR] HAPTIC feature not found on this device.", file=sys.stderr)
            return False

        params = bytes([int(waveform)])
        reply = self._send_long(idx, HAPTIC_FN_PLAY_WAVEFORM, params)
        return reply is not None

    def play_sequence(
        self,
        sequence: list[tuple[HapticWaveForm | int, float]],
    ) -> None:
        """
        Play a list of (waveform, delay_seconds) steps.
        The delay is the pause *after* triggering that waveform.
        """
        for waveform, delay in sequence:
            ok = self.play_waveform(waveform)
            name = HapticWaveForm(waveform).name if isinstance(waveform, int) else waveform.name
            status = "✓" if ok else "✗"
            print(f"  [{status}] {name:<24} (id=0x{int(waveform):02X})")
            if delay > 0:
                time.sleep(delay)


# ---------------------------------------------------------------------------
# Device discovery
# ---------------------------------------------------------------------------

def find_mx_master4_device(verbose: bool = False) -> Optional[tuple[hid.device, int]]:
    """
    Enumerate all HID devices, looking for Logitech VID (0x046D) devices
    on the vendor-defined usage page (0xFF00).  Returns (device, device_index)
    or None if nothing suitable is found.
    """
    candidates = []
    for info in hid.enumerate():
        if info["vendor_id"] != LOGITECH_VID:
            continue
        up = info.get("usage_page", 0)
        if up not in (0xFF00, 0xFF43):
            continue
        candidates.append(info)
        if verbose:
            print(
                f"  Found: PID=0x{info['product_id']:04X}  "
                f"usage_page=0x{info.get('usage_page', 0):04X}  "
                f"usage=0x{info.get('usage', 0):04X}  "
                f"path={info['path']}  "
                f"product='{info.get('product_string', '')}'"
            )

    for info in candidates:
        d = hid.device()
        try:
            d.open_path(info["path"])
            d.set_nonblocking(True)
        except Exception:
            continue

        # Try Bluetooth first (0xFF), then Bolt receiver device indices
        for dev_idx in [0xFF, DEVICE_INDEX_BOLT, 0x02, 0x03]:
            mouse = MXMaster4(d, dev_idx)
            idx = mouse.get_haptic_feature_index()
            if idx is not None:
                print(
                    f"✓ Found MX Master 4: PID=0x{info['product_id']:04X}, "
                    f"device_index=0x{dev_idx:02X}, haptic_feature_index=0x{idx:02X}"
                )
                return d, dev_idx

        d.close()

    return None


def list_logitech_devices() -> None:
    """Print all Logitech HID devices that could be the mouse."""
    print("Logitech HID devices found:")
    print("-" * 72)
    found = False
    for info in hid.enumerate():
        if info["vendor_id"] != LOGITECH_VID:
            continue
        found = True
        print(
            f"  PID=0x{info['product_id']:04X}  "
            f"usage_page=0x{info.get('usage_page', 0):04X}  "
            f"usage=0x{info.get('usage', 0):04X}\n"
            f"    product  : {info.get('product_string', '<none>')}\n"
            f"    path     : {info['path']}\n"
        )
    if not found:
        print("  (none)")


# ---------------------------------------------------------------------------
# Interactive demo (no arguments)
# ---------------------------------------------------------------------------

def run_interactive_demo(mouse: MXMaster4) -> None:
    print("\nRunning interactive waveform demo...")
    print("  Each waveform will play in order with a short pause.\n")
    waveforms = list(HapticWaveForm)
    for i, wf in enumerate(waveforms):
        is_last = i == len(waveforms) - 1
        ok = mouse.play_waveform(wf)
        status = "✓" if ok else "✗"
        print(f"  [{status}] {wf.name:<24} (id=0x{wf.value:02X})")
        if not is_last:
            time.sleep(0.5)
    print("\nDemo complete.")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        prog="haptics.py",
        description="Play haptic sequences on Logitech MX Master 4",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=f"""\
Available waveforms:
  {chr(10).join(f'  {w.name:<26} (id=0x{w.value:02X})' for w in HapticWaveForm)}

Available sequences:
  {', '.join(sorted(PREDEFINED_SEQUENCES.keys()))}
""",
    )

    parser.add_argument(
        "--list",
        action="store_true",
        help="List all detected Logitech HID devices and exit",
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Show verbose device discovery output",
    )
    parser.add_argument(
        "--waveform", "-w",
        metavar="NAME_OR_ID",
        help="Play a single haptic waveform (name or hex/decimal id)",
    )
    parser.add_argument(
        "--sequence", "-s",
        metavar="NAME",
        choices=list(PREDEFINED_SEQUENCES.keys()),
        help="Play a predefined sequence",
    )
    parser.add_argument(
        "--level", "-l",
        type=int,
        metavar="0-100",
        help="Set haptic feedback intensity (0=off, 1-100=enabled)",
    )
    parser.add_argument(
        "--demo",
        action="store_true",
        help="Run full waveform demo (plays every waveform once)",
    )

    args = parser.parse_args()

    # List mode – no device needed
    if args.list:
        list_logitech_devices()
        return

    # Connect to the mouse
    print("Searching for Logitech MX Master 4 with HAPTIC feature…")
    result = find_mx_master4_device(verbose=args.verbose)
    if result is None:
        print(
            "\n✗ No compatible device found.\n"
            "  Make sure the MX Master 4 is connected (Bolt receiver or Bluetooth).\n"
            "  On Windows you may need to run this script as Administrator, or install\n"
            "  a generic HID driver (e.g., Zadig) for the Bolt receiver interface.\n"
            "  Use --list to see what devices are visible.",
            file=sys.stderr,
        )
        sys.exit(1)

    dev, dev_idx = result
    mouse = MXMaster4(dev, dev_idx)

    try:
        # --- Set level first if requested ---
        if args.level is not None:
            level = max(0, min(100, args.level))
            print(f"\nSetting haptic level to {level}%…")
            ok = mouse.set_haptic_level(level)
            print(f"  {'✓ Done' if ok else '✗ Failed'}")
            if level == 0:
                print("  Haptics disabled.")
                return

        # --- Play a single waveform ---
        if args.waveform:
            raw = args.waveform.strip().upper()
            if raw in WAVEFORM_BY_NAME:
                wf = WAVEFORM_BY_NAME[raw]
            else:
                try:
                    wf = HapticWaveForm(int(raw, 0))
                except (ValueError, KeyError):
                    print(
                        f"✗ Unknown waveform '{args.waveform}'.\n"
                        f"  Valid names: {', '.join(WAVEFORM_BY_NAME.keys())}",
                        file=sys.stderr,
                    )
                    sys.exit(1)
            print(f"\nPlaying waveform {wf.name} (0x{wf.value:02X})…")
            ok = mouse.play_waveform(wf)
            print(f"  {'✓ Done' if ok else '✗ Failed (no reply – try --verbose)'}")

        # --- Play a named sequence ---
        elif args.sequence:
            seq = PREDEFINED_SEQUENCES[args.sequence]
            print(f"\nPlaying sequence '{args.sequence}' ({len(seq)} steps)…")
            mouse.play_sequence(seq)

        # --- Full demo (all waveforms) ---
        elif args.demo:
            run_interactive_demo(mouse)

        # --- Default: interactive demo ---
        else:
            print(
                "\nNo action specified – running interactive demo.\n"
                "Tip: use --help to see all options.\n"
            )
            run_interactive_demo(mouse)

    finally:
        dev.close()


if __name__ == "__main__":
    main()
