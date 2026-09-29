"""Run from the repository root: python -m unittest discover -s .agents/tests -v."""
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


class ConfigurationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="pnx-config-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
        shutil.copytree(ROOT / "configs", self.root / "configs")
        self.boards = sorted(p.parent.name for p in (self.root / "configs/boards").glob("*/board.cmake"))
        self.assertTrue(self.boards)

    def cmake(self, script, board, *args):
        return subprocess.run(
            ["cmake", f"-DPNX_BOARD={board}", *args, "-P", f"configs/cmake/{script}"],
            cwd=self.root, text=True, capture_output=True,
        )

    def test_generation_and_editor_without_generated_boards(self):
        self.assertFalse((self.root / "boards").exists())
        for board in self.boards:
            with self.subTest(board=board):
                output = self.root / "build" / board
                result = self.cmake("configure.cmake", board, f"-DOUT_DIR={output}")
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                for name in ("config.hpp", "robot_config.hpp", "bsp_bindings.hpp", "bsp_bindings.cpp", "cubemx.script"):
                    self.assertGreater((output / name).stat().st_size, 0)
                result = self.cmake("export_editor_context.cmake", board, f"-DPNX_EDITOR_OUTPUT={output}/editor.json")
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                context = json.loads((output / "editor.json").read_text())
                self.assertEqual(context["board"], board)
                for file in context["files"].values():
                    path = Path(file).resolve()
                    self.assertTrue(path.is_relative_to(self.root / "configs"), file)
                    self.assertTrue(path.is_file(), file)
        self.assertFalse((self.root / "boards").exists())

    def test_unknown_profile_rejected(self):
        for script, output in (("configure.cmake", "-DOUT_DIR=build/bad"),
                               ("export_editor_context.cmake", "-DPNX_EDITOR_OUTPUT=build/bad.json")):
            with self.subTest(script=script):
                self.assertNotEqual(self.cmake(script, "unknown_board", output).returncode, 0)
        self.assertFalse((self.root / "build/bad/config.hpp").exists())
        self.assertFalse((self.root / "build/bad.json").exists())

    def test_mismatched_identity_rejected(self):
        board = self.boards[0]
        path = self.root / "configs/boards" / board / "board.json"
        data = json.loads(path.read_text())
        data["name"] = "wrong_profile"
        path.write_text(json.dumps(data), encoding="utf-8")
        self.assertNotEqual(self.cmake("configure.cmake", board, "-DOUT_DIR=build/bad").returncode, 0)

    def test_fixed_binding_must_exist_in_ioc(self):
        board = self.boards[0]
        path = self.root / "configs/boards" / board / "board.json"
        data = json.loads(path.read_text())
        data["bindings"]["gpio_outputs"]["bmi088_acc_cs"]["pin"] = "invalid_pin"
        path.write_text(json.dumps(data), encoding="utf-8")
        result = self.cmake("configure.cmake", board, "-DOUT_DIR=build/bad")
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse((self.root / "build/bad/bsp_bindings.cpp").exists())


if __name__ == "__main__":
    unittest.main()
