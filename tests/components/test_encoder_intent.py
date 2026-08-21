"""Build and run the EncoderIntent host test.

The class is header-only and free of ESPHome dependencies, so its expiry rules
can be exercised on the host with a plain C++ compiler.
"""

from pathlib import Path
import shutil
import subprocess

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
SOURCE = REPO_ROOT / "tests" / "cpp" / "test_encoder_intent.cpp"

COMPILERS = ("g++", "clang++")


def _compiler() -> str | None:
    for name in COMPILERS:
        if shutil.which(name):
            return name
    return None


@pytest.mark.skipif(_compiler() is None, reason="no C++ compiler available")
def test_encoder_intent(tmp_path: Path) -> None:
    binary = tmp_path / "test_encoder_intent"
    subprocess.run(
        [
            _compiler(),
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
    try:
        result = subprocess.run(
            [str(binary)], capture_output=True, text=True, check=True
        )
    except PermissionError:  # pytest's temp dir lives on a noexec mount
        pytest.skip("temporary directory is not executable")
    assert result.stdout.strip() == "ok"
