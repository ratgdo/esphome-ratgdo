#!/usr/bin/env python3
"""Update YAML files to use current Git branch instead of main."""

import json
import os
import sys
from pathlib import Path

import yaml

RATGDO_REPO = "ratgdo/esphome-ratgdo"


def get_current_branch():
    """Get current branch name from GitHub environment."""
    # For PRs, check GITHUB_REF first
    github_ref = os.environ.get("GITHUB_REF", "")
    if "/pull/" in github_ref:
        # It's a PR, get branch name from event file
        github_event_path = os.environ.get("GITHUB_EVENT_PATH")
        if github_event_path and os.path.exists(github_event_path):
            with open(github_event_path) as f:
                event_data = json.load(f)
                pr_data = event_data.get("pull_request", {})
                return pr_data.get("head", {}).get("ref", "main")

    # For pushes, extract branch from GITHUB_REF
    if github_ref.startswith("refs/heads/"):
        return github_ref.replace("refs/heads/", "")

    return "main"


branch = get_current_branch()

if branch == "main":
    print("On main branch, skipping ref updates")
    sys.exit(0)

print(f"Updating refs to: {branch}")

# Process all YAML files
for yaml_file in Path(".").glob("*.yaml"):
    with open(yaml_file, "r") as f:
        data = yaml.safe_load(f)

    if not data:
        continue

    changed = False

    # Update external_components
    if "external_components" in data:
        for comp in data.get("external_components", []):
            if not isinstance(comp, dict) or "source" not in comp:
                continue
            source = comp["source"]
            if RATGDO_REPO not in source.get("url", ""):
                continue
            if source.get("ref") == "main":
                source["ref"] = branch
                changed = True

    # Update remote_package
    if "packages" in data and isinstance(data["packages"], dict):
        pkg = data["packages"].get("remote_package", {})
        if RATGDO_REPO in pkg.get("url", ""):
            if pkg.get("ref") == "main":
                pkg["ref"] = branch
                changed = True
            elif "ref" not in pkg:
                pkg["ref"] = branch
                changed = True

    # Update dashboard_import
    if "dashboard_import" in data:
        url = data["dashboard_import"].get("package_import_url", "")
        if "@main" in url:
            data["dashboard_import"]["package_import_url"] = url.replace(
                "@main", f"@{branch}"
            )
            changed = True

    if changed:
        with open(yaml_file, "w") as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)
        print(f"Updated {yaml_file}")

print("Done!")
