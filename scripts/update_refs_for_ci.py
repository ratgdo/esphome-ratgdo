#!/usr/bin/env python3
"""Update YAML files to use local paths instead of GitHub URLs for CI testing."""

import os
import re
import sys
from pathlib import Path


def main():
    """Main function."""
    # Get the absolute path to the project root
    project_root = Path(__file__).parent.parent.absolute()

    print(f"Project root: {project_root}")
    print("Updating YAML files to use local paths for CI testing...")

    # Process all YAML files
    for yaml_file in Path(".").glob("*.yaml"):
        with open(yaml_file, "r") as f:
            content = f.read()

        original = content

        # Update external_components to use local path
        if (
            "external_components:" in content
            and "type: git" in content
            and "ratgdo/esphome-ratgdo" in content
        ):
            # Replace the git source with local source
            content = re.sub(
                r"(\s*-\s*source:\s*\n\s*)type:\s*git\s*\n\s*url:\s*https://github\.com/ratgdo/esphome-ratgdo\s*\n\s*ref:\s*\w+",
                rf"\1type: local\n\1path: {project_root}/components",
                content,
            )

        # Update remote_package to use local path
        if "remote_package:" in content and "ratgdo/esphome-ratgdo" in content:
            # First update the URL
            content = re.sub(
                r"(remote_package:\s*\n\s*)url:\s*https://github\.com/ratgdo/esphome-ratgdo",
                rf"\1url: file://{project_root}",
                content,
            )
            # Then remove the ref line while preserving indentation
            # This matches the ref line and removes it completely
            content = re.sub(r"(\n\s+)ref:\s*\w+\n", r"\n", content)

        # Update dashboard_import to point to local file
        if "dashboard_import:" in content:
            # Extract filename from the github URL
            match = re.search(
                r"github://ratgdo/esphome-ratgdo/(\S+\.yaml)@\w+", content
            )
            if match:
                filename = match.group(1)
                # Replace with local file path
                content = re.sub(
                    r"(package_import_url:\s*)github://ratgdo/esphome-ratgdo/\S+\.yaml@\w+",
                    rf"\1file://{project_root}/{filename}",
                    content,
                )

        if content != original:
            with open(yaml_file, "w") as f:
                f.write(content)
            print(f"Updated {yaml_file}")

    print("Done!")
    return 0


if __name__ == "__main__":
    sys.exit(main())
