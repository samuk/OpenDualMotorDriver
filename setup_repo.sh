#!/usr/bin/env bash
# Bash equivalent of setup_repo.ps1 for macOS / Linux users.
#
# Copies (never moves, never deletes) source files, schematics, gerbers, BOM,
# and renders from the parent "MR17 - Custom Dual Motor Driver" folder into the
# OpenDualMotorDriver repo layout. Re-running it is safe.

set -euo pipefail

repo_root="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source_root="${1:-$( dirname "$repo_root" )}"

if [ "$repo_root" = "$source_root" ]; then
  echo "Refusing to run with source == destination." >&2
  echo "Move OpenDualMotorDriver out of the source folder or pass an explicit source path." >&2
  exit 1
fi

echo "Repo root  : $repo_root"
echo "Source root: $source_root"
echo

ensure_dir() { mkdir -p "$1"; }

copy_if_exists() {
  local from="$1"
  local to="$2"
  if [ -e "$from" ]; then
    ensure_dir "$( dirname "$to" )"
    cp -f "$from" "$to"
    echo "  copied  $( basename "$from" )"
  else
    echo "  missing $from" >&2
  fi
}

copy_dir_contents() {
  local from="$1"
  local to="$2"
  shift 2
  if [ ! -d "$from" ]; then
    echo "  missing source folder $from" >&2
    return 0
  fi
  ensure_dir "$to"
  shopt -s nullglob
  for f in "$from"/*; do
    [ -d "$f" ] && continue
    case "$( basename "$f" )" in
      __pycache__) continue ;;
    esac
    cp -f "$f" "$to/"
    echo "  copied  $( basename "$f" )"
  done
  shopt -u nullglob
}

echo "[1/6] Firmware sources"
copy_dir_contents "$source_root/Codes/PicoDualMotorDriver" "$repo_root/Firmware/PicoDualMotorDriver"
copy_if_exists    "$source_root/Codes/API_REFERENCE.md"    "$repo_root/Firmware/API_REFERENCE.md"

echo
echo "[2/6] Desktop GUI sources"
gui_src="$source_root/Codes/tools"
gui_dst="$repo_root/Software/gui"
ensure_dir "$gui_dst"
if [ -d "$gui_src" ]; then
  shopt -s nullglob
  for f in "$gui_src"/*; do
    [ -d "$f" ] && continue
    name="$( basename "$f" )"
    if [ "$name" = "README.md" ]; then continue; fi
    cp -f "$f" "$gui_dst/$name"
    echo "  copied  $name"
  done
  shopt -u nullglob
else
  echo "  missing source folder $gui_src" >&2
fi

echo
echo "[3/6] Schematics"
copy_dir_contents "$source_root/Schematics" "$repo_root/Hardware/Schematics"

echo
echo "[4/6] Gerbers"
copy_dir_contents "$source_root/PCB Gerber Files" "$repo_root/Hardware/Gerbers"

echo
echo "[5/6] PCB renders + BOM"
copy_dir_contents "$source_root/Pictures/EDA" "$repo_root/Hardware/PCB-Renders"

bom_dst="$repo_root/Hardware/BOM"
ensure_dir "$bom_dst"
copy_if_exists "$source_root/BOM - MR17 Custom Dual Motor Driver.xlsx"     "$bom_dst/BOM.xlsx"
copy_if_exists "$source_root/BOM - CSV - MR17 Custom Dual Motor Driver.csv" "$bom_dst/BOM.csv"

echo
echo "[6/6] Photos and gallery images"
pictures_src="$source_root/Pictures"
images_dst="$repo_root/Images"
docs_images_dst="$repo_root/Docs/images"
ensure_dir "$images_dst"
ensure_dir "$docs_images_dst"

if [ -d "$pictures_src" ]; then
  shopt -s nullglob
  for f in "$pictures_src"/*.JPG "$pictures_src"/*.jpg "$pictures_src"/*.PNG "$pictures_src"/*.png \
            "$pictures_src"/*.JPEG "$pictures_src"/*.jpeg; do
    [ -e "$f" ] || continue
    cp -f "$f" "$images_dst/$( basename "$f" )"
    echo "  copied  $( basename "$f" )"
  done
  shopt -u nullglob
else
  echo "  missing source folder $pictures_src" >&2
fi

copy_if_exists "$pictures_src/EDA/PCB_3D_1.PNG" "$images_dst/PCB_3D_1.PNG"
copy_if_exists "$pictures_src/EDA/PCB_3D_2.PNG" "$images_dst/PCB_3D_2.PNG"

echo
echo "Done. The OpenDualMotorDriver folder is now self-contained and ready to commit."
