"""
MCP 验证器 (Validator)

自动验证生成的 MCP Server 代码，修复错误，并注册到 OpenClaw。
"""

import os
import sys
import json
import yaml
import asyncio
import subprocess
from pathlib import Path
from typing import Optional, List, Dict, Any


class MCPValidator:
    """
    MCP Server 验证器

    功能：
    1. 在隔离进程中运行生成的代码
    2. 捕获并分析错误
    3. 自动修复（最多3次重试）
    4. 注册到 OpenClaw 配置
    """

    MAX_RETRIES = 3
    DEFAULT_TIMEOUT = 30

    def __init__(self):
        self.retry_count = 0
        self.error_history: List[str] = []

    async def validate(
        self,
        server_file: str,
        max_retries: int = 3,
        timeout: int = 30
    ) -> bool:
        """
        验证 MCP Server 是否能正常运行。

        Args:
            server_file: MCP Server Python 文件路径
            max_retries: 最大重试次数
            timeout: 验证超时时间（秒）

        Returns:
            验证是否通过
        """
        server_file = Path(server_file)

        if not server_file.exists():
            print(f"      ❌ 文件不存在: {server_file}")
            return False

        for attempt in range(1, max_retries + 1):
            print(f"      尝试 {attempt}/{max_retries}...")

            success, errors = await self._run_validation(server_file, timeout)

            if success:
                print(f"      ✓ 验证通过")
                return True

            # 记录错误
            self.error_history.extend(errors)
            print(f"      ✗ 发现 {len(errors)} 个错误")
            for err in errors[:3]:  # 只显示前3个错误
                print(f"         - {err[:100]}...")

            if attempt < max_retries:
                print(f"      🔄 尝试自动修复...")
                fixed = await self._attempt_fix(server_file, errors)
                if not fixed:
                    print(f"      ⚠ 自动修复失败")

        print(f"      ❌ 验证失败，已达最大重试次数")
        return False

    async def _run_validation(
        self,
        server_file: Path,
        timeout: int
    ) -> tuple[bool, List[str]]:
        """
        在子进程中运行验证。

        Returns:
            (是否成功, 错误列表)
        """
        errors = []

        try:
            # 使用 Python 语法检查
            result = await asyncio.wait_for(
                self._run_syntax_check(server_file),
                timeout=timeout
            )

            if not result["success"]:
                errors.extend(result.get("errors", []))
                return False, errors

            # 尝试导入模块
            result = await asyncio.wait_for(
                self._run_import_check(server_file),
                timeout=timeout
            )

            if not result["success"]:
                errors.extend(result.get("errors", []))
                return False, errors

            return True, []

        except asyncio.TimeoutError:
            errors.append(f"验证超时（>{timeout}秒）")
            return False, errors

        except Exception as e:
            errors.append(f"验证异常: {e}")
            return False, errors

    async def _run_syntax_check(self, server_file: Path) -> Dict[str, Any]:
        """运行语法检查"""
        try:
            proc = await asyncio.create_subprocess_exec(
                sys.executable, "-m", "py_compile", str(server_file),
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE
            )

            stdout, stderr = await proc.communicate()

            if proc.returncode == 0:
                return {"success": True}
            else:
                return {
                    "success": False,
                    "errors": [stderr.decode('utf-8', errors='ignore')]
                }

        except Exception as e:
            return {"success": False, "errors": [str(e)]}

    async def _run_import_check(self, server_file: Path) -> Dict[str, Any]:
        """尝试导入模块检查依赖"""
        try:
            # 创建一个临时脚本来测试导入
            test_script = f"""
import sys
sys.path.insert(0, str({server_file.parent!r}))

try:
    import importlib.util
    spec = importlib.util.spec_from_file_location("test_module", {str(server_file)!r})
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    print("IMPORT_SUCCESS")
except Exception as e:
    print(f"IMPORT_ERROR: {{e}}")
    sys.exit(1)
"""

            proc = await asyncio.create_subprocess_exec(
                sys.executable, "-c", test_script,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE
            )

            stdout, stderr = await proc.communicate()
            output = stdout.decode('utf-8', errors='ignore')

            if "IMPORT_SUCCESS" in output:
                return {"success": True}
            else:
                errors = []
                stderr_text = stderr.decode('utf-8', errors='ignore')
                if stderr_text:
                    errors.append(stderr_text)
                if "IMPORT_ERROR" in output:
                    errors.append(output.split("IMPORT_ERROR:")[-1].strip())
                return {"success": False, "errors": errors}

        except Exception as e:
            return {"success": False, "errors": [str(e)]}

    async def _attempt_fix(self, server_file: Path, errors: List[str]) -> bool:
        """
        尝试自动修复代码错误。

        简单的修复规则：
        - 缺少 import -> 添加
        - 语法错误 -> 尝试修正
        """
        try:
            code = server_file.read_text(encoding='utf-8')

            # 常见的自动修复规则
            fixes_applied = []

            # 修复 1: 缺少 struct import
            if any("struct" in err.lower() for err in errors):
                if "import struct" not in code:
                    code = "import struct\n" + code
                    fixes_applied.append("添加 struct import")

            # 修复 2: 缺少 typing import
            if any("typing" in err.lower() or "Optional" in err or "Dict" in err for err in errors):
                if "from typing import" not in code and "import typing" not in code:
                    imports = []
                    if "Optional" in code:
                        imports.append("Optional")
                    if "Dict" in code:
                        imports.append("Dict")
                    if "Any" in code:
                        imports.append("Any")
                    if "List" in code:
                        imports.append("List")
                    if imports:
                        import_line = f"from typing import {', '.join(imports)}\n"
                        code = import_line + code
                        fixes_applied.append(f"添加 typing imports: {imports}")

            # 修复 3: 缺少 asyncio import
            if any("asyncio" in err.lower() for err in errors):
                if "import asyncio" not in code:
                    code = "import asyncio\n" + code
                    fixes_applied.append("添加 asyncio import")

            # 修复 4: 语法错误 - 缩进问题
            if any("IndentationError" in err for err in errors):
                # 尝试修复缩进（简单处理）
                lines = code.split('\n')
                fixed_lines = []
                prev_indent = 0

                for line in lines:
                    stripped = line.lstrip()
                    if stripped and not stripped.startswith('#'):
                        current_indent = len(line) - len(stripped)
                        if current_indent > prev_indent + 4:
                            # 缩进过多
                            line = ' ' * (prev_indent + 4) + stripped
                        prev_indent = current_indent
                    fixed_lines.append(line)

                code = '\n'.join(fixed_lines)
                fixes_applied.append("修复缩进")

            if fixes_applied:
                server_file.write_text(code, encoding='utf-8')
                print(f"         应用修复: {', '.join(fixes_applied)}")
                return True

            return False

        except Exception as e:
            print(f"         修复过程出错: {e}")
            return False

    async def register_to_openclaw(
        self,
        server_file: str,
        target_name: str
    ) -> bool:
        """
        将生成的 MCP Server 注册到 OpenClaw 配置。

        Args:
            server_file: MCP Server 文件路径
            target_name: MCP Server 名称

        Returns:
            是否成功更新配置
        """
        # 查找 OpenClaw 配置文件
        config_paths = [
            Path.home() / ".config" / "openclaw" / "config.yaml",
            Path.home() / ".openclaw" / "config.yaml",
            Path.cwd() / "config.yaml",
        ]

        config_file = None
        for path in config_paths:
            if path.exists():
                config_file = path
                break

        if not config_file:
            print(f"      ⚠ 未找到 OpenClaw 配置文件")
            return False

        try:
            # 读取现有配置
            with open(config_file, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f) or {}

            # 确保 mcpServers 部分存在
            if "mcpServers" not in config:
                config["mcpServers"] = {}

            # 添加新的 MCP Server 配置
            server_name = f"{target_name}-server"
            config["mcpServers"][server_name] = {
                "command": sys.executable,
                "args": [str(server_file)],
                "transportType": "stdio",
                "autoApprove": []
            }

            # 写回配置文件
            with open(config_file, 'w', encoding='utf-8') as f:
                yaml.dump(config, f, default_flow_style=False, allow_unicode=True)

            print(f"      ✓ 已添加到配置: {config_file}")
            return True

        except Exception as e:
            print(f"      ⚠ 更新配置失败: {e}")
            return False
