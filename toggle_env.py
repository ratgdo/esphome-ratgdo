import glob
import re
import sys


def toggle_env(mode, ref="softserial-killer"):
    if mode not in ["local", "remote"]:
        print("Usage: python toggle_env.py [local|remote] [branch_name]")
        sys.exit(1)

    yaml_files = glob.glob("*.yaml") + glob.glob("static/*.yaml")

    for filepath in yaml_files:
        if filepath in {"secrets.yaml", "ratgdo.yaml"}:
            continue

        with open(filepath) as f:
            content = f.read()

        new_content = content
        # 1. Handle board files with `packages:` block
        if "packages:" in new_content:
            # Extract the base file name (e.g. base.yaml, base_secplusv1.yaml)
            base_file_match = re.search(
                r"file[s]?:\s*\[?(base[\w\.]*\.yaml)\]?", new_content
            )
            if base_file_match:
                base_file = base_file_match.group(1)

                # Create the standardized replacement blocks
                if mode == "local":
                    packages_block = f"""packages:
  # remote_package:
  #   url: https://github.com/ratgdo/esphome-ratgdo
  #   ref: {ref}
  #   files: [{base_file}]
  #   refresh: 1s
  remote_package: !include
    file: {base_file}"""
                else:
                    packages_block = f"""packages:
  remote_package:
    url: https://github.com/ratgdo/esphome-ratgdo
    ref: {ref}
    files: [{base_file}]
    refresh: 1s
  # remote_package: !include
  #   file: {base_file}"""

                # Replace everything from `packages:` to the next top-level key or EOF
                new_content = re.sub(
                    r"^packages:\n(?:^[ \t]+.*\n?|^\s*\n)*",
                    packages_block + "\n\n",
                    new_content,
                    flags=re.MULTILINE,
                )

        # 2. Handle base files with `external_components:` block
        if "external_components:" in new_content:
            if mode == "local":
                ext_block = f"""external_components:
  # - source:
  #     type: git
  #     url: https://github.com/ratgdo/esphome-ratgdo
  #     ref: {ref}
  #   refresh: 1s
  - source:
      type: local
      path: components"""
            else:
                ext_block = f"""external_components:
  - source:
      type: git
      url: https://github.com/ratgdo/esphome-ratgdo
      ref: {ref}
    refresh: 1s
  # - source:
  #     type: local
  #     path: components"""

            new_content = re.sub(
                r"^external_components:\n(?:^[ \t]+.*\n?|^\s*\n)*",
                ext_block + "\n\n",
                new_content,
                flags=re.MULTILINE,
            )
        # 3. Handle dashboard_import branch suffix update
        new_content = re.sub(
            r"package_import_url: github://ratgdo/esphome-ratgdo/([^@\n]+)(?:@[^\n]+)?",
            f"package_import_url: github://ratgdo/esphome-ratgdo/\\1@{ref}",
            new_content,
        )

        if content != new_content:
            with open(filepath, "w") as f:
                f.write(new_content)
            print(f"Updated {filepath} to {mode} mode (ref: {ref})")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python toggle_env.py [local|remote] [branch_name]")
        sys.exit(1)

    mode = sys.argv[1]
    ref = sys.argv[2] if len(sys.argv) > 2 else "main"
    toggle_env(mode, ref)
