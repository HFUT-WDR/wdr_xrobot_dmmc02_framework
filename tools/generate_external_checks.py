#!/usr/bin/env python3
"""生成模块构造和未启用 YAML 配置的编译检查，不执行烧录或运行。"""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import tempfile

import yaml

ROOT = Path(__file__).resolve().parents[1]
MODULES = [f"YunMo-yanyu/{name}" for name in
           ("CMD", "RMMotor", "DMMotor", "DR16", "Referee", "HostData")]
MODULES += [f"xrobot-org/{name}" for name in ("SharedTopic", "SharedTopicClient")]
MODULES += ["YunMo-yanyu/I6x", "YunMo-yanyu/EventBinder"]


def run(*args, cwd=ROOT):
    subprocess.run(args, cwd=cwd, check=True)


def generate(output):
    output.mkdir(parents=True, exist_ok=True)
    for module in MODULES:
        run("xrobot", "check-module", "--offline", module, "-o",
            str(output / (module.split("/")[1] + ".cpp")))

    # 这些示例绑定仅用于编译检查，不代表实际外接接线。
    bindings = json.loads((ROOT / "tests/external_modules/compile_bindings.json").read_text())
    template = yaml.safe_load((ROOT / "config/templates/external_devices.yaml.in").read_text())
    for instance in template["modules"]:
        for arg in instance["args"]:
            for key, value in arg.items():
                if key in bindings.get(instance["id"], {}):
                    arg[key] = bindings[instance["id"]][key]
                elif value is None:
                    raise ValueError(f"Unbound compile fixture: {instance['id']}.{key}")
    # 在临时基座中生成代码，避免改变主工程选中的配置。
    with tempfile.TemporaryDirectory(prefix="wdr-module-check-") as temp:
        stage = Path(temp)
        shutil.copyfile(ROOT / "xrobot.lock", stage / "xrobot.lock")
        (stage / "Modules").symlink_to(ROOT / "Modules", target_is_directory=True)
        shutil.copytree(ROOT / "User", stage / "User",
                        ignore=shutil.ignore_patterns("xrobot_main.hpp"))
        board = yaml.safe_load((ROOT / "User/xrobot.yaml").read_text())
        # PyYAML 不保留引号样式，而 XRobot 用该样式识别 C++ 字符串字面量。
        def preserve_topic_strings(value):
            if isinstance(value, dict):
                for key, item in value.items():
                    if isinstance(item, str) and key.endswith("_topic_name"):
                        value[key] = '"' + item.strip('"') + '"'
                    else:
                        preserve_topic_strings(item)
            elif isinstance(value, list):
                for item in value:
                    preserve_topic_strings(item)
        preserve_topic_strings(board)
        board["modules"].extend(template["modules"])
        (stage / "User/external_compile.yaml").write_text(
            yaml.safe_dump(board, sort_keys=False, allow_unicode=True))
        run("xrobot", "gen", "-c", "User/external_compile.yaml", "--no-line-directives", cwd=stage)
        shutil.copyfile(stage / "User/xrobot_main.hpp", output / "external_profile.hpp")
        # 在临时工程生成完整 i6x 配置，不改变原工程的默认配置。
        run("xrobot", "gen", "-c", "User/xrobot_i6x.yaml", "--no-line-directives", cwd=stage)
        shutil.copyfile(stage / "User/xrobot_main.hpp", output / "i6x_profile.hpp")
    (output / "i6x_profile.cpp").write_text(
        '// 仅编译验证 i6x 的 YAML 配置，不链接进默认固件。\n'
        '#include "i6x_profile.hpp"\n')
    (output / "external_profile.cpp").write_text(
        '// 生成的未启用配置，仅编译检查，不接入固件。\n'
        '#include "external_profile.hpp"\n')


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path,
                        default=ROOT / "build/external_modules")
    args = parser.parse_args()
    generate(args.output.resolve())
