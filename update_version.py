# update_version.py
import re

version_file = "version.h"

with open(version_file, "r") as f:
    content = f.read()

version_pattern = r"#define\s+APP_VERSION_PATCH\s+(\d+)"
match = re.search(version_pattern, content)
if match:
    patch_version = int(match.group(1)) + 1
    updated_content = re.sub(version_pattern, f"#define APP_VERSION_PATCH   {patch_version}", content)
    with open(version_file, "w") as f:
        f.write(updated_content)
    print(f"Version {patch_version} updated.")
else:
    print("Version pattern not found in the file.")