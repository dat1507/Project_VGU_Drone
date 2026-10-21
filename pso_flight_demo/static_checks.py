"""Dependency-free syntax/AST safety checks; does not write bytecode or run ROS."""

import ast
from pathlib import Path


def main():
    root = Path(__file__).resolve().parent
    files = sorted(root.rglob("*.py"))
    for path in files:
        source = path.read_text()
        tree = ast.parse(source, filename=str(path))
        compile(tree, str(path), "exec", dont_inherit=True)
        for node in ast.walk(tree):
            if isinstance(node, (ast.Import, ast.ImportFrom)):
                names = [a.name for a in node.names]
                names.append(getattr(node, "module", "") or "")
                if any("export_flight_path" in n for n in names):
                    raise AssertionError(f"Forbidden side-effectful import: {path}")
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
                if node.func.attr == "command" and node.args and isinstance(node.args[0], ast.Constant):
                    if node.args[0].value not in {21, 176}:
                        raise AssertionError(f"Unexpected control command: {path}")
        if any(line.rstrip() != line for line in source.splitlines()):
            raise AssertionError(f"Trailing whitespace: {path}")
    print(f"PASS: {len(files)} Python files parsed/compiled in memory; AST import/command guards; whitespace")
    print("This is not a type checker, flight test, or ROS/DDS integration test.")


if __name__ == "__main__":
    main()
