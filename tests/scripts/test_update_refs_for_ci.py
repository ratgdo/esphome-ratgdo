#!/usr/bin/env python3
"""Tests for update_refs_for_ci.py script."""

import json
import os
import sys
import tempfile
from pathlib import Path
from unittest import mock

import pytest

# Add scripts directory to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent / "scripts"))

import update_refs_for_ci


class TestGetPRInfo:
    """Test get_pr_info function."""

    def test_pull_request_from_fork(self, tmp_path):
        """Test PR from a fork."""
        event_file = tmp_path / "event.json"
        event_data = {
            "pull_request": {
                "head": {
                    "ref": "feature-branch",
                    "repo": {"full_name": "user/esphome-ratgdo"},
                }
            }
        }
        event_file.write_text(json.dumps(event_data))

        with mock.patch.dict(
            os.environ,
            {
                "GITHUB_REF": "refs/pull/123/merge",
                "GITHUB_EVENT_PATH": str(event_file),
            },
        ):
            branch, fork_repo = update_refs_for_ci.get_pr_info()
            assert branch == "feature-branch"
            assert fork_repo == "user/esphome-ratgdo"

    def test_pull_request_from_main_repo(self, tmp_path):
        """Test PR from the main repository."""
        event_file = tmp_path / "event.json"
        event_data = {
            "pull_request": {
                "head": {
                    "ref": "fix-something",
                    "repo": {"full_name": "ratgdo/esphome-ratgdo"},
                }
            }
        }
        event_file.write_text(json.dumps(event_data))

        with mock.patch.dict(
            os.environ,
            {
                "GITHUB_REF": "refs/pull/456/merge",
                "GITHUB_EVENT_PATH": str(event_file),
            },
        ):
            branch, fork_repo = update_refs_for_ci.get_pr_info()
            assert branch == "fix-something"
            assert fork_repo == "ratgdo/esphome-ratgdo"

    def test_push_to_branch(self):
        """Test push to a branch."""
        with mock.patch.dict(os.environ, {"GITHUB_REF": "refs/heads/develop"}):
            branch, fork_repo = update_refs_for_ci.get_pr_info()
            assert branch == "develop"
            assert fork_repo is None

    def test_no_github_ref(self):
        """Test when GITHUB_REF is not set."""
        with mock.patch.dict(os.environ, {}, clear=True):
            branch, fork_repo = update_refs_for_ci.get_pr_info()
            assert branch == "main"
            assert fork_repo is None


class TestUpdateRefs:
    """Test the main update functionality."""

    def test_update_external_components(self, tmp_path):
        """Test updating external_components ref."""
        yaml_content = """
external_components:
  - source:
      type: git
      url: https://github.com/ratgdo/esphome-ratgdo
      ref: main
    refresh: 1s
"""
        yaml_file = tmp_path / "test.yaml"
        yaml_file.write_text(yaml_content)

        os.chdir(tmp_path)
        with mock.patch.dict(os.environ, {"GITHUB_REF": "refs/heads/test-branch"}):
            # Run the script
            with mock.patch("sys.argv", ["update_refs_for_ci.py"]):
                exec(open(update_refs_for_ci.__file__).read())

        updated_content = yaml_file.read_text()
        assert "ref: test-branch" in updated_content
        assert "ref: main" not in updated_content

    def test_update_remote_package_with_ref(self, tmp_path):
        """Test updating remote_package that already has ref."""
        yaml_content = """
packages:
  remote_package:
    url: https://github.com/ratgdo/esphome-ratgdo
    ref: main
    files: [base.yaml]
"""
        yaml_file = tmp_path / "test.yaml"
        yaml_file.write_text(yaml_content)

        os.chdir(tmp_path)
        with mock.patch.dict(os.environ, {"GITHUB_REF": "refs/heads/feature-x"}):
            with mock.patch("sys.argv", ["update_refs_for_ci.py"]):
                exec(open(update_refs_for_ci.__file__).read())

        updated_content = yaml_file.read_text()
        assert "ref: feature-x" in updated_content
        assert "ref: main" not in updated_content

    def test_add_ref_to_remote_package(self, tmp_path):
        """Test adding ref to remote_package that doesn't have one."""
        yaml_content = """
packages:
  remote_package:
    url: https://github.com/ratgdo/esphome-ratgdo
    files: [base.yaml]
"""
        yaml_file = tmp_path / "test.yaml"
        yaml_file.write_text(yaml_content)

        os.chdir(tmp_path)
        with mock.patch.dict(os.environ, {"GITHUB_REF": "refs/heads/add-ref"}):
            with mock.patch("sys.argv", ["update_refs_for_ci.py"]):
                exec(open(update_refs_for_ci.__file__).read())

        updated_content = yaml_file.read_text()
        assert "ref: add-ref" in updated_content
        # Check proper indentation
        assert "    ref: add-ref\n    files:" in updated_content

    def test_update_dashboard_import(self, tmp_path):
        """Test updating dashboard_import URL."""
        yaml_content = """
dashboard_import:
  package_import_url: github://ratgdo/esphome-ratgdo/v2board.yaml@main
"""
        yaml_file = tmp_path / "test.yaml"
        yaml_file.write_text(yaml_content)

        os.chdir(tmp_path)
        with mock.patch.dict(os.environ, {"GITHUB_REF": "refs/heads/update-dash"}):
            with mock.patch("sys.argv", ["update_refs_for_ci.py"]):
                exec(open(update_refs_for_ci.__file__).read())

        updated_content = yaml_file.read_text()
        assert "@update-dash" in updated_content
        assert "@main" not in updated_content

    def test_fork_updates_urls(self, tmp_path):
        """Test that fork PRs update repository URLs."""
        yaml_content = """
external_components:
  - source:
      type: git
      url: https://github.com/ratgdo/esphome-ratgdo
      ref: main
dashboard_import:
  package_import_url: github://ratgdo/esphome-ratgdo/v2board.yaml@main
"""
        yaml_file = tmp_path / "test.yaml"
        yaml_file.write_text(yaml_content)

        event_file = tmp_path / "event.json"
        event_data = {
            "pull_request": {
                "head": {
                    "ref": "fork-feature",
                    "repo": {"full_name": "forkeduser/esphome-ratgdo"},
                }
            }
        }
        event_file.write_text(json.dumps(event_data))

        os.chdir(tmp_path)
        with mock.patch.dict(
            os.environ,
            {
                "GITHUB_REF": "refs/pull/789/merge",
                "GITHUB_EVENT_PATH": str(event_file),
            },
        ):
            with mock.patch("sys.argv", ["update_refs_for_ci.py"]):
                exec(open(update_refs_for_ci.__file__).read())

        updated_content = yaml_file.read_text()
        assert "url: https://github.com/forkeduser/esphome-ratgdo" in updated_content
        assert (
            "github://forkeduser/esphome-ratgdo/v2board.yaml@fork-feature"
            in updated_content
        )
        assert "ref: fork-feature" in updated_content
        assert "ratgdo/esphome-ratgdo" not in updated_content

    def test_skip_main_branch(self, tmp_path):
        """Test that main branch is skipped."""
        yaml_content = """
external_components:
  - source:
      type: git
      url: https://github.com/ratgdo/esphome-ratgdo
      ref: main
"""
        yaml_file = tmp_path / "test.yaml"
        yaml_file.write_text(yaml_content)

        os.chdir(tmp_path)
        with mock.patch.dict(os.environ, {"GITHUB_REF": "refs/heads/main"}):
            with mock.patch("sys.argv", ["update_refs_for_ci.py"]):
                with pytest.raises(SystemExit) as exc_info:
                    exec(open(update_refs_for_ci.__file__).read())
                assert exc_info.value.code == 0

        # Content should not be changed
        updated_content = yaml_file.read_text()
        assert updated_content == yaml_content

    def test_preserve_esphome_tags(self, tmp_path):
        """Test that ESPHome-specific tags are preserved."""
        yaml_content = """
external_components:
  - source:
      type: git
      url: https://github.com/ratgdo/esphome-ratgdo
      ref: main

button:
  - platform: template
    name: "Test"
    on_press:
      then:
        lambda: !lambda |-
          id($id_prefix).query_status();
"""
        yaml_file = tmp_path / "test.yaml"
        yaml_file.write_text(yaml_content)

        os.chdir(tmp_path)
        with mock.patch.dict(os.environ, {"GITHUB_REF": "refs/heads/preserve-tags"}):
            with mock.patch("sys.argv", ["update_refs_for_ci.py"]):
                exec(open(update_refs_for_ci.__file__).read())

        updated_content = yaml_file.read_text()
        assert "ref: preserve-tags" in updated_content
        assert "lambda: !lambda |-" in updated_content  # ESPHome tag preserved
        assert "id($id_prefix).query_status();" in updated_content

    def test_no_changes_no_write(self, tmp_path):
        """Test that files are not written if no changes are needed."""
        yaml_content = """
packages:
  some_other_package:
    url: https://github.com/other/repo
    ref: main
"""
        yaml_file = tmp_path / "test.yaml"
        yaml_file.write_text(yaml_content)
        original_mtime = yaml_file.stat().st_mtime

        os.chdir(tmp_path)
        with mock.patch.dict(os.environ, {"GITHUB_REF": "refs/heads/no-changes"}):
            with mock.patch("sys.argv", ["update_refs_for_ci.py"]):
                exec(open(update_refs_for_ci.__file__).read())

        # File should not be modified
        assert yaml_file.stat().st_mtime == original_mtime
        assert yaml_file.read_text() == yaml_content


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
