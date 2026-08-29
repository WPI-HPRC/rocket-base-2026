Import("env")
import os
import io
import stat
import platform
import ssl
import subprocess
import urllib.request
import zipfile
from pathlib import Path


# flatbuffers download requires SSL which isn't included with some PlatformIO python builds
def _make_ssl_context():
    try:
        import certifi

        return ssl.create_default_context(cafile=certifi.where())
    except ImportError:
        return ssl.create_default_context()


# ─── Configuration ────────────────────────────────────────────────────────────

FLATC_VERSION = "25.12.19"
SCHEMA_DIR    = Path(env.subst("$PROJECT_DIR/telemetry-2026"))
OUTPUT_DIR    = Path(env.subst("$PROJECT_DIR/lib/telemetry-generated"))
TOOLS_DIR     = Path(env.subst("$PROJECT_DIR/tools"))

# ─── Platform → release asset mapping ────────────────────────────────────────

def get_flatc_asset():
    """Return (archive_filename, binary_name_in_zip, local_binary_name)."""
    system = platform.system()
    machine = platform.machine().lower()

    if system == "Linux":
        # flatbuffers releases only ship x86-64 Linux binaries officially;
        # ARM (Raspberry Pi, etc.) must build from source or use apt.
        if "aarch64" in machine or "arm64" in machine:
            return None, None, "flatc"   # signal to fall back to apt/source
        return (
            "Linux.flatc.binary.g++-13.zip",
            "flatc",
            "flatc",
        )
    elif system == "Darwin":
        # Universal binary works on both Intel and Apple Silicon
        return (
            "Mac.flatc.binary.zip",
            "flatc",
            "flatc",
        )
    elif system == "Windows":
        return (
            "Windows.flatc.binary.zip",
            "flatc.exe",
            "flatc.exe",
        )
    else:
        print(f"[flatbuffers] Unsupported OS: {system}")
        env.Exit(1)

# ─── Download + cache ─────────────────────────────────────────────────────────

def download_flatc():
    """Download flatc for the current platform if not already cached."""
    TOOLS_DIR.mkdir(parents=True, exist_ok=True)

    archive_name, binary_in_zip, local_name = get_flatc_asset()
    local_binary = TOOLS_DIR / local_name

    # Cache hit — check the binary matches the desired version
    version_stamp = TOOLS_DIR / ".flatc_version"
    if local_binary.exists() and version_stamp.exists():
        if version_stamp.read_text().strip() == FLATC_VERSION:
            return str(local_binary)
        else:
            print(f"[flatbuffers] flatc version mismatch, re-downloading...")
            local_binary.unlink()

    # ARM Linux: no official binary, try system package
    if archive_name is None:
        import shutil
        flatc = shutil.which("flatc")
        if flatc:
            print(f"[flatbuffers] ARM Linux detected, using system flatc: {flatc}")
            return flatc
        print(
            "[flatbuffers] ERROR: No official flatc binary for ARM Linux.\n"
            "  Install with:  sudo apt install flatbuffers-compiler"
        )
        env.Exit(1)

    url = (
        f"https://github.com/google/flatbuffers/releases/download/"
        f"v{FLATC_VERSION}/{archive_name}"
    )

    print(f"[flatbuffers] Downloading flatc v{FLATC_VERSION} for {platform.system()}...")
    print(f"              {url}")

    try:
        with urllib.request.urlopen(url, context=_make_ssl_context()) as response:
            zip_bytes = io.BytesIO(response.read())
    except Exception as e:
        print(f"[flatbuffers] Download failed: {e}")
        env.Exit(1)

    with zipfile.ZipFile(zip_bytes) as zf:
        if binary_in_zip not in zf.namelist():
            print(f"[flatbuffers] ERROR: '{binary_in_zip}' not found in archive.")
            print(f"              Archive contains: {zf.namelist()}")
            env.Exit(1)
        data = zf.read(binary_in_zip)

    local_binary.write_bytes(data)

    # Mark executable on Unix
    if platform.system() != "Windows":
        local_binary.chmod(local_binary.stat().st_mode | stat.S_IEXEC | stat.S_IXGRP | stat.S_IXOTH)

    # Write version stamp so we only re-download on version bump
    version_stamp.write_text(FLATC_VERSION)

    print(f"[flatbuffers] flatc saved to {local_binary}")
    return str(local_binary)

# ─── Staleness check ──────────────────────────────────────────────────────────

def is_stale(schema_path: Path, output_dir: Path) -> bool:
    stem = schema_path.stem
    header = output_dir / f"{stem}_generated.h"
    if not header.exists():
        return True
    return schema_path.stat().st_mtime > header.stat().st_mtime

# ─── Code generation ──────────────────────────────────────────────────────────

def generate_flatbuffers():
    flatc = download_flatc()
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    fbs_files = list(SCHEMA_DIR.glob("**/*.fbs"))
    if not fbs_files:
        print(f"[flatbuffers] No .fbs files found in {SCHEMA_DIR}")
        return

    stale = [f for f in fbs_files if is_stale(f, OUTPUT_DIR)]
    if not stale:
        print("[flatbuffers] All generated headers are up to date.")
        return

    print(f"[flatbuffers] Generating {len(stale)} schema(s) → {OUTPUT_DIR}")
    for fbs in stale:
        cmd = [
            flatc,
            "--cpp", "--gen-mutable", "--reflect-names",
            "--cpp-std", "c++11",
            "-o", str(OUTPUT_DIR),
            str(fbs),
        ]
        print(f"  flatc: {fbs.name}")
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode != 0:
            print(f"[flatbuffers] FAILED on {fbs.name}:\n{result.stderr}")
            env.Exit(1)

    print("[flatbuffers] Code generation complete.")

# ─── Run immediately at script load time ──────────────────────────────────────
# This is the critical difference: generation happens while SCons is still
# constructing the build graph, so headers exist before source scanning begins.
generate_flatbuffers()

# Still register the alias for `pio run -t generate_fbs`
env.AlwaysBuild(env.Alias("generate_fbs", [], lambda s, t, e: generate_flatbuffers()))
