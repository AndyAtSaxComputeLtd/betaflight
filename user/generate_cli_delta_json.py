#!/usr/bin/env python3

import argparse
import ast
import json
import re
import subprocess
import sys
from collections import defaultdict
from pathlib import Path


COMMANDS_PATH = "src/main/cli/cli.c"
SETTINGS_PATH = "src/main/cli/settings.c"
PARAMETER_NAMES_PATH = "src/main/fc/parameter_names.h"


def git_show(tag, path):
    result = subprocess.run(
        ["git", "show", f"{tag}:{path}"],
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise FileNotFoundError(result.stderr.strip() or f"missing {tag}:{path}")
    return result.stdout


def git_tag_exists(tag):
    result = subprocess.run(
        ["git", "tag", "--list", tag],
        check=False,
        capture_output=True,
        text=True,
    )
    return result.returncode == 0 and result.stdout.strip() == tag


def extract_initializer_block(source, anchor):
    start = source.find(anchor)
    if start == -1:
        raise ValueError(f"anchor not found: {anchor}")
    brace_start = source.find("{", start)
    if brace_start == -1:
        raise ValueError(f"initializer start not found for: {anchor}")
    depth = 0
    in_string = False
    escape = False
    for index in range(brace_start, len(source)):
        char = source[index]
        if in_string:
            if escape:
                escape = False
            elif char == "\\":
                escape = True
            elif char == '"':
                in_string = False
            continue
        if char == '"':
            in_string = True
            continue
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return source[brace_start + 1:index]
    raise ValueError(f"unterminated initializer for: {anchor}")


def split_top_level_csv(text):
    items = []
    start = 0
    paren_depth = 0
    brace_depth = 0
    bracket_depth = 0
    in_string = False
    escape = False
    for index, char in enumerate(text):
        if in_string:
            if escape:
                escape = False
            elif char == "\\":
                escape = True
            elif char == '"':
                in_string = False
            continue
        if char == '"':
            in_string = True
            continue
        if char == "(":
            paren_depth += 1
        elif char == ")":
            paren_depth -= 1
        elif char == "{":
            brace_depth += 1
        elif char == "}":
            brace_depth -= 1
        elif char == "[":
            bracket_depth += 1
        elif char == "]":
            bracket_depth -= 1
        elif char == "," and paren_depth == 0 and brace_depth == 0 and bracket_depth == 0:
            items.append(text[start:index].strip())
            start = index + 1
    tail = text[start:].strip()
    if tail:
        items.append(tail)
    return items


def decode_c_string(expr):
    expr = expr.strip()
    if expr == "NULL":
        return None
    parts = []
    for match in re.finditer(r'"((?:\\.|[^"\\])*)"', expr, flags=re.DOTALL):
        parts.append(ast.literal_eval(f'"{match.group(1)}"'))
    if not parts:
        return expr
    return "".join(parts)


def normalize_whitespace(value):
    return re.sub(r"\s+", " ", value).strip()


def parse_command_definitions(cli_source):
    block = extract_initializer_block(cli_source, "const clicmd_t cmdTable[] =")
    definitions = defaultdict(list)
    needle = "CLI_COMMAND_DEF("
    index = 0
    while True:
        start = block.find(needle, index)
        if start == -1:
            break
        pos = start + len(needle)
        depth = 1
        in_string = False
        escape = False
        while pos < len(block) and depth > 0:
            char = block[pos]
            if in_string:
                if escape:
                    escape = False
                elif char == "\\":
                    escape = True
                elif char == '"':
                    in_string = False
            else:
                if char == '"':
                    in_string = True
                elif char == "(":
                    depth += 1
                elif char == ")":
                    depth -= 1
            pos += 1
        call_body = block[start + len(needle):pos - 1]
        fields = split_top_level_csv(call_body)
        if len(fields) != 4:
            raise ValueError(f"unexpected CLI_COMMAND_DEF field count: {len(fields)}")
        name = decode_c_string(fields[0])
        definitions[name].append(
            {
                "description": decode_c_string(fields[1]),
                "args": decode_c_string(fields[2]),
                "handler": normalize_whitespace(fields[3]),
            }
        )
        index = pos
    return {name: sorted(variants, key=lambda item: json.dumps(item, sort_keys=True)) for name, variants in definitions.items()}


def split_brace_entries(block):
    entries = []
    depth = 0
    start = None
    in_string = False
    escape = False
    for index, char in enumerate(block):
        if in_string:
            if escape:
                escape = False
            elif char == "\\":
                escape = True
            elif char == '"':
                in_string = False
            continue
        if char == '"':
            in_string = True
            continue
        if char == "{":
            if depth == 0:
                start = index
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0 and start is not None:
                entries.append(block[start:index + 1])
                start = None
    return entries


def parse_parameter_name_map(source):
    mapping = {}
    for macro, value in re.findall(r'^#define\s+(PARAM_NAME_[A-Z0-9_]+)\s+"((?:\\.|[^"\\])*)"', source, flags=re.MULTILINE):
        mapping[macro] = ast.literal_eval(f'"{value}"')
    return mapping


def parse_setting_name(name_expr, parameter_names):
    name_expr = name_expr.strip()
    if name_expr.startswith('"'):
        return decode_c_string(name_expr)
    if name_expr in parameter_names:
        return parameter_names[name_expr]
    return name_expr


def parse_type_expression(type_expr):
    tokens = [token.strip() for token in type_expr.split("|")]
    var_type = next((token for token in tokens if token.startswith("VAR_")), None)
    section = next((token for token in tokens if token.endswith("_VALUE")), None)
    mode = next((token for token in tokens if token.startswith("MODE_")), "MODE_DIRECT")
    return {
        "expression": normalize_whitespace(type_expr),
        "tokens": tokens,
        "var_type": var_type,
        "section": section,
        "mode": mode,
    }


def parse_config(config_expr):
    text = normalize_whitespace(config_expr)
    lookup_match = re.search(r'\.config\.lookup\s*=\s*\{\s*([A-Z0-9_]+)\s*\}', text)
    if lookup_match:
        return {"kind": "lookup", "table": lookup_match.group(1), "raw": text}
    minmax_match = re.search(r'\.config\.minmax\s*=\s*\{\s*([^,]+)\s*,\s*([^}]+)\s*\}', text)
    if minmax_match:
        return {"kind": "minmax", "min": normalize_whitespace(minmax_match.group(1)), "max": normalize_whitespace(minmax_match.group(2)), "raw": text}
    minmax_unsigned_match = re.search(r'\.config\.minmaxUnsigned\s*=\s*\{\s*([^,]+)\s*,\s*([^}]+)\s*\}', text)
    if minmax_unsigned_match:
        return {"kind": "minmaxUnsigned", "min": normalize_whitespace(minmax_unsigned_match.group(1)), "max": normalize_whitespace(minmax_unsigned_match.group(2)), "raw": text}
    array_match = re.search(r'\.config\.array\.length\s*=\s*([^,}]+)', text)
    if array_match:
        return {"kind": "array", "length": normalize_whitespace(array_match.group(1)), "raw": text}
    bitset_match = re.search(r'\.config\.bitpos\s*=\s*([^,}]+)', text)
    if bitset_match:
        return {"kind": "bitset", "bitpos": normalize_whitespace(bitset_match.group(1)), "raw": text}
    string_match = re.search(r'\.config\.string\s*=\s*\{\s*([^}]*)\}', text)
    if string_match:
        values = [normalize_whitespace(part) for part in split_top_level_csv(string_match.group(1))]
        return {"kind": "string", "values": values, "raw": text}
    u32_match = re.search(r'\.config\.u32Max\s*=\s*([^,}]+)', text)
    if u32_match:
        return {"kind": "u32Max", "max": normalize_whitespace(u32_match.group(1)), "raw": text}
    d32_match = re.search(r'\.config\.d32Max\s*=\s*([^,}]+)', text)
    if d32_match:
        return {"kind": "d32Max", "max": normalize_whitespace(d32_match.group(1)), "raw": text}
    return {"kind": "raw", "raw": text}


def parse_setting_definitions(settings_source, parameter_names):
    block = extract_initializer_block(settings_source, "const clivalue_t valueTable[] =")
    definitions = defaultdict(list)
    for entry in split_brace_entries(block):
        fields = split_top_level_csv(entry[1:-1])
        if len(fields) < 5:
            continue
        name = parse_setting_name(fields[0], parameter_names)
        record = {
            "type": parse_type_expression(fields[1]),
            "config": parse_config(fields[2]),
            "pgn": normalize_whitespace(fields[3]),
            "offset": normalize_whitespace(",".join(fields[4:])) if len(fields) > 5 else normalize_whitespace(fields[4]),
        }
        definitions[name].append(record)
    return {name: sorted(variants, key=lambda item: json.dumps(item, sort_keys=True)) for name, variants in definitions.items()}


def load_version_snapshot(tag):
    cli_source = git_show(tag, COMMANDS_PATH)
    settings_source = git_show(tag, SETTINGS_PATH)
    parameter_name_source = git_show(tag, PARAMETER_NAMES_PATH)
    parameter_names = parse_parameter_name_map(parameter_name_source)
    commands = parse_command_definitions(cli_source)
    settings = parse_setting_definitions(settings_source, parameter_names)
    return {
        "tag": tag,
        "commands": commands,
        "settings": settings,
        "stats": {
            "command_names": len(commands),
            "command_variants": sum(len(variants) for variants in commands.values()),
            "setting_names": len(settings),
            "setting_variants": sum(len(variants) for variants in settings.values()),
        },
    }


def diff_named_definitions(baseline, target):
    baseline_names = set(baseline)
    target_names = set(target)
    added = {name: target[name] for name in sorted(target_names - baseline_names)}
    removed = {name: baseline[name] for name in sorted(baseline_names - target_names)}
    changed = {}
    for name in sorted(baseline_names & target_names):
        if baseline[name] != target[name]:
            changed[name] = {
                "baseline": baseline[name],
                "target": target[name],
            }
    return {
        "added": added,
        "removed": removed,
        "changed": changed,
        "counts": {
            "added": len(added),
            "removed": len(removed),
            "changed": len(changed),
        },
    }


def build_report(target_tag, baseline_tags):
    report = {
        "target": target_tag,
        "baselines": {},
    }
    if not git_tag_exists(target_tag):
        raise SystemExit(f"target tag does not exist: {target_tag}")
    target_snapshot = load_version_snapshot(target_tag)
    report["target_stats"] = target_snapshot["stats"]
    for baseline_tag in baseline_tags:
        if not git_tag_exists(baseline_tag):
            report["baselines"][baseline_tag] = {
                "tag_exists": False,
                "error": f"git tag '{baseline_tag}' not found",
            }
            continue
        baseline_snapshot = load_version_snapshot(baseline_tag)
        report["baselines"][baseline_tag] = {
            "tag_exists": True,
            "baseline_stats": baseline_snapshot["stats"],
            "commands": diff_named_definitions(baseline_snapshot["commands"], target_snapshot["commands"]),
            "parameters": diff_named_definitions(baseline_snapshot["settings"], target_snapshot["settings"]),
        }
    return report


def main(argv):
    parser = argparse.ArgumentParser(description="Generate a JSON map of Betaflight CLI command and parameter changes across tags.")
    parser.add_argument("--target", default="2025.12.5", help="Target tag to compare against.")
    parser.add_argument(
        "--baseline",
        action="append",
        dest="baselines",
        default=["4.3.3", "4.4.0", "4.3.5"],
        help="Baseline tag to compare against. Repeat to add multiple baselines.",
    )
    parser.add_argument(
        "--output",
        default="user/cli_command_parameter_changes_to_2025.12.5.json",
        help="Output path for the generated JSON report.",
    )
    args = parser.parse_args(argv)

    report = build_report(args.target, args.baselines)
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"wrote {output_path}")


if __name__ == "__main__":
    main(sys.argv[1:])