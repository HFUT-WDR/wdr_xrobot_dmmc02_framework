#!/bin/sh
set -eu
project_root=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
libxr_root="$project_root/Middlewares/Third_Party/LibXR"
patch_path="$project_root/patches/libxr-host-topic.patch"
# 保留依赖仓中的其他改动；已应用的补丁不重复应用。
if git -C "$libxr_root" apply --reverse --check "$patch_path" 2>/dev/null; then
  :
elif git -C "$libxr_root" apply --check "$patch_path"; then
  git -C "$libxr_root" apply "$patch_path"
else
  echo "LibXR补丁与当前依赖不匹配，请先检查依赖版本及本地改动。" >&2
  exit 1
fi
cd "$project_root"
xrobot setup --frozen
cd host
xrobot setup --frozen
xrobot gen -c User/xrobot.yaml
