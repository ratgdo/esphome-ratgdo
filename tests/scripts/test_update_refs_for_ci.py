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

    def test_github_actions_workspace_path(self, tmp_path):
        """Test that GITHUB_ACTIONS uses GITHUB_WORKSPACE for project root."""
        yaml_content = """
external_components:
  - source:
      type: git
      url: https://github.com/ratgdo/esphome-ratgdo
      ref: main
    refresh: 1s
"""
        workspace_path = tmp_path / "workspace"
        workspace_path.mkdir()

        yaml_file = workspace_path / "test.yaml"
        yaml_file.write_text(yaml_content)

        os.chdir(tmp_path)
        with mock.patch.dict(
            os.environ,
            {
                "GITHUB_ACTIONS": "true",
                "GITHUB_WORKSPACE": str(workspace_path),
                "GITHUB_REF": "refs/heads/test-branch",
            },
        ):
            update_refs_for_ci.main()

        updated_content = yaml_file.read_text()
        assert f"path: {workspace_path}/components" in updated_content

    def test_update_external_components_to_local(self, tmp_path):
        """Test updating external_components to use local path."""
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

        # Mock the project root to be tmp_path
        os.chdir(tmp_path)
        with mock.patch.object(Path, "absolute", return_value=tmp_path):
            with mock.patch.dict(os.environ, {"GITHUB_REF": "refs/heads/test-branch"}):
                update_refs_for_ci.main()

        updated_content = yaml_file.read_text()
        assert "type: local" in updated_content
        assert f"path: {tmp_path}/components" in updated_content
        assert "type: git" not in updated_content
        assert "url: https://github.com/ratgdo/esphome-ratgdo" not in updated_content
        assert "ref: main" not in updated_content

    def test_update_remote_package_to_local_include(self, tmp_path):
        """Test updating remote_package to use local include."""
        yaml_content = """
packages:
  remote_package:
    url: https://github.com/ratgdo/esphome-ratgdo
    ref: main
    files: [base.yaml]
    refresh: 1s
"""
        yaml_file = tmp_path / "test.yaml"
        yaml_file.write_text(yaml_content)

        os.chdir(tmp_path)
        with mock.patch.object(Path, "absolute", return_value=tmp_path):
            with mock.patch.dict(os.environ, {"GITHUB_REF": "refs/heads/test-branch"}):
                update_refs_for_ci.main()

        updated_content = yaml_file.read_text()
        assert "packages:" in updated_content
        assert f"- !include {tmp_path}/base.yaml" in updated_content
        assert "remote_package:" not in updated_content
        assert "url: https://github.com/ratgdo/esphome-ratgdo" not in updated_content
        assert "ref: main" not in updated_content

    def test_remove_dashboard_import_package_url(self, tmp_path):
        """Test removing package_import_url from dashboard_import."""
        yaml_content = """
dashboard_import:
  package_import_url: github://ratgdo/esphome-ratgdo/v2board.yaml@main
"""
        yaml_file = tmp_path / "test.yaml"
        yaml_file.write_text(yaml_content)

        os.chdir(tmp_path)
        with mock.patch.object(Path, "absolute", return_value=tmp_path):
            with mock.patch.dict(os.environ, {"GITHUB_REF": "refs/heads/test-branch"}):
                update_refs_for_ci.main()

        updated_content = yaml_file.read_text()
        assert "dashboard_import:" not in updated_content
        assert "package_import_url:" not in updated_content

    def test_mixed_updates(self, tmp_path):
        """Test updating a file with multiple patterns."""
        yaml_content = """
external_components:
  - source:
      type: git
      url: https://github.com/ratgdo/esphome-ratgdo
      ref: main
    refresh: 1s

dashboard_import:
  package_import_url: github://ratgdo/esphome-ratgdo/v25board.yaml@main

packages:
  remote_package:
    url: https://github.com/ratgdo/esphome-ratgdo
    ref: main
    files: [base.yaml]
    refresh: 1s
"""
        yaml_file = tmp_path / "test.yaml"
        yaml_file.write_text(yaml_content)

        os.chdir(tmp_path)
        with mock.patch.object(Path, "absolute", return_value=tmp_path):
            with mock.patch.dict(os.environ, {"GITHUB_REF": "refs/heads/test-mixed"}):
                update_refs_for_ci.main()

        updated_content = yaml_file.read_text()
        # Check external_components
        assert "type: local" in updated_content
        assert f"path: {tmp_path}/components" in updated_content
        # Check dashboard_import is removed
        assert "dashboard_import:" not in updated_content
        assert "package_import_url:" not in updated_content
        # packages section should have remote_package replaced with local include list
        assert "packages:" in updated_content
        assert f"- !include {tmp_path}/base.yaml" in updated_content
        assert "remote_package:" not in updated_content
        # Ensure no GitHub URLs remain for external_components
        assert updated_content.count("github.com/ratgdo/esphome-ratgdo") == 0

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
        with mock.patch.object(Path, "absolute", return_value=tmp_path):
            with mock.patch.dict(
                os.environ, {"GITHUB_REF": "refs/heads/preserve-tags"}
            ):
                update_refs_for_ci.main()

        updated_content = yaml_file.read_text()
        assert "type: local" in updated_content
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
        with mock.patch.object(Path, "absolute", return_value=tmp_path):
            with mock.patch.dict(os.environ, {"GITHUB_REF": "refs/heads/no-changes"}):
                update_refs_for_ci.main()

        # File should not be modified
        assert yaml_file.stat().st_mtime == original_mtime
        assert yaml_file.read_text() == yaml_content


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
