#!/usr/bin/env python3
"""Tests for update_refs_for_ci.py script."""

import os
import sys
import tempfile
from pathlib import Path
from unittest import mock

import pytest

# Add scripts directory to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent / "scripts"))

import update_refs_for_ci


class TestUpdateRefs:
    """Test the main update functionality."""

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
            update_refs_for_ci.main()

        updated_content = yaml_file.read_text()
        assert "type: local" in updated_content
        assert f"path: {tmp_path}/components" in updated_content
        assert "type: git" not in updated_content
        assert "url: https://github.com/ratgdo/esphome-ratgdo" not in updated_content
        assert "ref: main" not in updated_content

    def test_update_with_comments(self, tmp_path):
        """Test updating when there are comments in the YAML."""
        yaml_content = """
external_components:
  - source:
      type: git
      url: https://github.com/ratgdo/esphome-ratgdo
      ref: main
    refresh: 1s
  # - source:
  #     type: local
  #     path: components
"""
        yaml_file = tmp_path / "test.yaml"
        yaml_file.write_text(yaml_content)

        os.chdir(tmp_path)
        with mock.patch.object(Path, "absolute", return_value=tmp_path):
            update_refs_for_ci.main()

        updated_content = yaml_file.read_text()
        # Check that git source is replaced with local
        assert "type: local" in updated_content
        assert f"path: {tmp_path}/components" in updated_content
        assert "type: git" not in updated_content
        assert "url: https://github.com/ratgdo/esphome-ratgdo" not in updated_content
        # Comments should remain
        assert "# - source:" in updated_content

    def test_update_remote_package_to_local(self, tmp_path):
        """Test updating remote_package to use local file URL."""
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
        with mock.patch.object(Path, "absolute", return_value=tmp_path):
            update_refs_for_ci.main()

        updated_content = yaml_file.read_text()
        assert f"url: file://{tmp_path}" in updated_content
        assert "url: https://github.com/ratgdo/esphome-ratgdo" not in updated_content
        assert "ref: main" not in updated_content

    def test_update_dashboard_import_to_local(self, tmp_path):
        """Test updating dashboard_import to use local file URL."""
        yaml_content = """
dashboard_import:
  package_import_url: github://ratgdo/esphome-ratgdo/v2board.yaml@main
"""
        yaml_file = tmp_path / "test.yaml"
        yaml_file.write_text(yaml_content)

        os.chdir(tmp_path)
        with mock.patch.object(Path, "absolute", return_value=tmp_path):
            update_refs_for_ci.main()

        updated_content = yaml_file.read_text()
        assert f"package_import_url: file://{tmp_path}/v2board.yaml" in updated_content
        assert "github://ratgdo/esphome-ratgdo" not in updated_content

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
            update_refs_for_ci.main()

        # File should not be modified
        assert yaml_file.stat().st_mtime == original_mtime
        assert yaml_file.read_text() == yaml_content

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
    files: [base.yaml]
    refresh: 1s
"""
        yaml_file = tmp_path / "test.yaml"
        yaml_file.write_text(yaml_content)

        os.chdir(tmp_path)
        with mock.patch.object(Path, "absolute", return_value=tmp_path):
            update_refs_for_ci.main()

        updated_content = yaml_file.read_text()
        # Check external_components
        assert "type: local" in updated_content
        assert f"path: {tmp_path}/components" in updated_content
        # Check dashboard_import
        assert f"package_import_url: file://{tmp_path}/v25board.yaml" in updated_content
        # Check remote_package
        assert f"url: file://{tmp_path}" in updated_content
        # Ensure no GitHub URLs remain
        assert "github.com/ratgdo/esphome-ratgdo" not in updated_content
        assert "github://ratgdo/esphome-ratgdo" not in updated_content

    def test_remote_package_without_ref(self, tmp_path):
        """Test updating remote_package that doesn't have a ref line."""
        yaml_content = """
packages:
  remote_package:
    url: https://github.com/ratgdo/esphome-ratgdo
    files: [base.yaml]
"""
        yaml_file = tmp_path / "test.yaml"
        yaml_file.write_text(yaml_content)

        os.chdir(tmp_path)
        with mock.patch.object(Path, "absolute", return_value=tmp_path):
            update_refs_for_ci.main()

        updated_content = yaml_file.read_text()
        assert f"url: file://{tmp_path}" in updated_content
        assert "files: [base.yaml]" in updated_content
        # Should not add a ref line
        assert "ref:" not in updated_content


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
