#!/usr/bin/env python3
import os
import sys
import tarfile
import urllib.request
import shutil

VOCAB_DIR = "/opt/ORB_SLAM3/Vocabulary"
TXT_PATH = os.path.join(VOCAB_DIR, "ORBvoc.txt")
TXT_TAR_PATH = os.path.join(VOCAB_DIR, "ORBvoc.txt.tar.gz")
FALLBACK_URL = "https://github.com/UZ-SLAMLab/ORB_SLAM3/raw/master/Vocabulary/ORBvoc.txt.tar.gz"

def main():
    target_dir = sys.argv[1] if len(sys.argv) > 1 else VOCAB_DIR
    target_txt = os.path.join(target_dir, "ORBvoc.txt")
    target_archive = os.path.join(target_dir, "ORBvoc.txt.tar.gz")

    if os.path.isfile(target_txt) and os.path.getsize(target_txt) > 0:
        print(f"[INFO] Vocabulary already exists at: {target_txt}")
        sys.exit(0)

    os.makedirs(target_dir, exist_ok=True)

    if os.path.isfile(target_archive) and os.path.getsize(target_archive) > 0:
        print(f"[INFO] Found local archive: {target_archive}. Extracting...")
        archive_to_extract = target_archive
        delete_after = False
    else:
        print(f"[INFO] Downloading ORBvoc.txt.tar.gz from {FALLBACK_URL}...")
        archive_to_extract = target_archive
        req = urllib.request.Request(
            FALLBACK_URL,
            headers={"User-Agent": "Mozilla/5.0 (X11; Linux x86_64)"}
        )
        with urllib.request.urlopen(req) as resp, open(archive_to_extract, "wb") as out_file:
            shutil.copyfileobj(resp, out_file)
        print(f"[INFO] Download completed: {os.path.getsize(archive_to_extract)} bytes")
        delete_after = True

    print(f"[INFO] Extracting archive {archive_to_extract} to {target_dir}...")
    with tarfile.open(archive_to_extract, "r:gz") as tar:
        tar.extractall(path=target_dir)

    if delete_after and os.path.isfile(archive_to_extract):
        os.remove(archive_to_extract)
        print(f"[INFO] Removed temporary archive {archive_to_extract}")

    if os.path.isfile(target_txt) and os.path.getsize(target_txt) > 0:
        print(f"[SUCCESS] ORBvoc.txt verified successfully at {target_txt} ({os.path.getsize(target_txt)} bytes).")
        sys.exit(0)
    else:
        print(f"[ERROR] Extraction finished but {target_txt} was not found or is empty.", file=sys.stderr)
        sys.exit(1)

if __name__ == "__main__":
    main()
