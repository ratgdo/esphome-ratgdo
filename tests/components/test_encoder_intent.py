"""Build and run the EncoderIntent host test.

The class is header-only and free of ESPHome dependencies, so its expiry rules
can be exercised on the host with a plain C++ compiler.
"""

import os
from pathlib import Path
import shutil
import subprocess
import tempfile

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
SOURCE = REPO_ROOT / "tests" / "cpp" / "test_encoder_intent.cpp"

COMPILERS = ("g++", "clang++")


def _compiler() -> str | None:
    for name in COMPILERS:
        if shutil.which(name):
            return name
    return None


def _in_ci() -> bool:
    return bool(os.environ.get("CI"))


def test_encoder_intent() -> None:
    compiler = _compiler()
    if compiler is None:
        # Skipping here would turn the only coverage of the expiry rules into a
        # green run, so CI has to fail instead.
        msg = f"no C++ compiler available (tried {', '.join(COMPILERS)})"
        if _in_ci():
            pytest.fail(msg)
        pytest.skip(msg)

    # Build inside the repo tree: pytest's temp dir can live on a noexec mount,
    # and skipping on that would hide a regression just as effectively.
    with tempfile.TemporaryDirectory(dir=REPO_ROOT) as build_dir:
        binary = Path(build_dir) / "test_encoder_intent"
        subprocess.run(
            [
                compiler,
                "-std=c++17",
                "-Wall",
                "-Werror",
                f"-I{REPO_ROOT}",
                str(SOURCE),
                "-o",
                str(binary),
            ],
            check=True,
        )
        result = subprocess.run(
            [str(binary)], capture_output=True, text=True, check=True
        )
    assert result.stdout.strip() == "ok"
