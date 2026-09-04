#!/usr/bin/env python3
#
# Copyright 2026 O-Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import annotations

import importlib.machinery
import importlib.util
from pathlib import Path
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "frontend_http_node.py"


def _load_frontend_module():
    loader = importlib.machinery.SourceFileLoader("frontend_http_node", str(SCRIPT))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


frontend = _load_frontend_module()


class GaussianSplatArtifactRootsTest(unittest.TestCase):

    def test_simulation_gaussian_splat_artifacts_are_allowed(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            workspace = Path(temp_dir)
            roots = frontend.gaussian_splat_artifact_allowed_roots(workspace)
            simulation_artifact = (
                workspace
                / "missions"
                / "simulations"
                / "demo"
                / "20260904T120000Z"
                / "gaussian_splat"
                / "gaussian_splat_manifest.json"
            ).resolve()

            self.assertTrue(any(root == simulation_artifact or root in simulation_artifact.parents for root in roots))

    def test_unrelated_paths_are_not_allowed(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            workspace = Path(temp_dir) / "workspace"
            outside = Path("/var/lib/amr_sweeper/gaussian_splat_manifest.json").resolve()
            roots = frontend.gaussian_splat_artifact_allowed_roots(workspace)

            self.assertFalse(any(root == outside or root in outside.parents for root in roots))


if __name__ == "__main__":
    unittest.main()
