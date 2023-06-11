import sys
import subprocess

FILENAME_VERSION_H = 'include/version.h'

file_template = """
/* This is a generated file that gets the git hash of the HEAD commit. */

#define VERSION_HASH "{version_hash}"
"""

result = subprocess.run(['git', 'rev-parse', 'HEAD'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
if result.returncode != 0:
    print('ERROR: Getting latest git hash failed:', file=sys.stderr)
    print(result.stdout.decode('utf-8'), file=sys.stderr)
    sys.exit(1)

git_hash = result.stdout.decode('utf-8').strip()[:8]
print(f'Stamping git hash: {git_hash}')

with open(FILENAME_VERSION_H, 'w') as f:
    f.write(file_template.format(version_hash=git_hash))