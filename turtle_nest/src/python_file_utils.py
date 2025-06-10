"""
 ------------------------------------------------------------------
 Copyright 2025 Janne Karttunen

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
 ------------------------------------------------------------------
"""


from black import format_str, Mode
import sys
import ast

#ast.parse requires Python 3.9+
if sys.version_info < (3, 9):
    sys.stderr.write(
        f"ERROR: Python >= 3.9 is required, but you have {sys.version.split()[0]}\n"
    )
    sys.exit(1)


def generate_new_setup_py(package_path, package_name, node_name):
    file_path = f'{package_path}/setup.py'
    new_console_script = f'{node_name} = {package_name}.{node_name}:main'
    formatted_code = _append_to_entry_points(file_path, new_console_script)
    return formatted_code


def _append_to_entry_points(file_path, new_console_script):
    with open(file_path, 'r') as f:
        tree = ast.parse(f.read(), filename=file_path)

    # Update the tree with a new console_script
    entry_points_updater = _EntryPointsUpdater(new_console_script)
    tree = entry_points_updater.visit(tree)
    ast.fix_missing_locations(tree)
    unformatted_code = ast.unparse(tree)

    if entry_points_updater.updates_made:
        # Go with the ROS 2 practices of not normalizing single quotes, and max line length 100
        return format_str(unformatted_code, mode=Mode(string_normalization=False, line_length=100))
    else:
        raise RuntimeError("Couldn't find entry_points or console_scripts in the setup.py!")


class _EntryPointsUpdater(ast.NodeTransformer):
    """Adds a new 'console_scripts' field in the entrypoint, for setup.py"""
    def __init__(self, new_console_script):
        super().__init__()
        self.new_console_script = new_console_script
        self.updates_made = False
    
    def visit_Call(self, node):
        """Traverse the AST to find the 'entry_points' argument """
        # Look for the `setup()` call
        if not (isinstance(node.func, ast.Name) and node.func.id == 'setup'):
            return node

        # Add new console_scripts field.
        for kw in node.keywords:
            if kw.arg == 'entry_points' and isinstance(kw.value, ast.Dict):
                # Find the 'console_scripts' entry
                for key, value in zip(kw.value.keys, kw.value.values):
                    if (isinstance(key, ast.Constant) and
                            isinstance(key.value, str) and
                            key.value == 'console_scripts'):
                        # Append the new entry
                        if isinstance(value, ast.List):
                            value.elts.append(ast.Constant(value=self.new_console_script))
                            self.updates_made = True
                        return node
                        
        return self.generic_visit(node)

