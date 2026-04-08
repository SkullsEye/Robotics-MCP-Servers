"""
协议分析器 (Analyzer)

解析 PDF 协议文档、C++/Python SDK 源码，提取结构化的硬件接口描述。
"""

import os
import re
import json
import asyncio
from pathlib import Path
from typing import Literal, Optional, Dict, Any, List
from dataclasses import dataclass

import pypdf
from pypdf import PdfReader


try:
    import litellm
    LITELLM_AVAILABLE = True
except ImportError:
    LITELLM_AVAILABLE = False


@dataclass
class ProtocolAction:
    """协议动作定义"""
    name: str
    params: Dict[str, str]
    description: str
    command_code: Optional[str] = None


@dataclass
class ProtocolState:
    """协议状态定义"""
    name: str
    type: str
    description: str
    unit: Optional[str] = None


@dataclass
class ProtocolDetails:
    """协议底层细节"""
    endian: str = "little"
    header: Optional[str] = None
    checksum: Optional[str] = None
    baudrate: Optional[int] = None
    frequency: Optional[float] = None


class SDKAnalyzer:
    """
    SDK 协议分析器

    支持分析：
    - PDF 协议文档
    - C++/Python 源码
    - ROS 接口定义
    """

    def __init__(self):
        self.analysis_result: Dict[str, Any] = {}

    async def analyze(
        self,
        source_path: str,
        hardware_type: Literal['serial', 'dds', 'ros', 'http']
    ) -> Dict[str, Any]:
        """
        分析硬件接口文档/SDK。

        Args:
            source_path: SDK 文件夹路径或 PDF 文件路径
            hardware_type: 硬件通信类型

        Returns:
            结构化的协议描述 JSON
        """
        source_path = Path(source_path)

        if not source_path.exists():
            raise FileNotFoundError(f"源路径不存在: {source_path}")

        # 读取源文件内容
        if source_path.is_file() and source_path.suffix.lower() == '.pdf':
            raw_content = await self._read_pdf(source_path)
        elif source_path.is_dir():
            raw_content = await self._read_code_directory(source_path, hardware_type)
        else:
            raise ValueError(f"不支持的源文件类型: {source_path}")

        # 使用 LLM 提取结构化信息
        protocol_schema = await self._extract_protocol_schema(
            raw_content, hardware_type
        )

        return protocol_schema

    async def _read_pdf(self, pdf_path: Path) -> str:
        """读取 PDF 文档内容"""
        print(f"   📄 读取 PDF: {pdf_path.name}")

        try:
            reader = PdfReader(str(pdf_path))
            text_content = []

            for i, page in enumerate(reader.pages):
                try:
                    text = page.extract_text()
                    if text:
                        text_content.append(f"--- Page {i+1} ---\n{text}")
                except Exception as e:
                    print(f"      ⚠ 第 {i+1} 页读取失败: {e}")

            content = "\n\n".join(text_content)

            if not content.strip():
                raise ValueError("PDF 内容为空或无法提取文本")

            return content

        except Exception as e:
            raise RuntimeError(f"PDF 读取错误: {e}")

    async def _read_code_directory(
        self,
        code_dir: Path,
        hardware_type: Literal['serial', 'dds', 'ros', 'http']
    ) -> str:
        """读取代码目录内容"""
        print(f"   📁 扫描代码目录: {code_dir}")

        content_parts = []

        # 读取关键文件
        key_extensions = {'.py', '.cpp', '.h', '.hpp', '.c'}
        ignore_patterns = ['__pycache__', '.git', 'node_modules', 'build', 'dist']

        for root, dirs, files in os.walk(code_dir):
            # 过滤忽略目录
            dirs[:] = [d for d in dirs if d not in ignore_patterns]

            for file in files:
                if any(file.endswith(ext) for ext in key_extensions):
                    file_path = Path(root) / file
                    try:
                        text = file_path.read_text(encoding='utf-8', errors='ignore')
                        # 限制每个文件的大小
                        if len(text) > 10000:
                            text = text[:10000] + "\n... [内容截断]"
                        content_parts.append(f"=== {file_path.relative_to(code_dir)} ===\n{text}")
                    except Exception as e:
                        print(f"      ⚠ 读取文件失败 {file}: {e}")

        if not content_parts:
            raise ValueError("未找到可读取的代码文件")

        return "\n\n".join(content_parts)

    async def _extract_protocol_schema(
        self,
        raw_content: str,
        hardware_type: Literal['serial', 'dds', 'ros', 'http']
    ) -> Dict[str, Any]:
        """
        使用 LLM 提取结构化的协议模式。

        如果 litellm 不可用，使用基于规则的分析。
        """
        if LITELLM_AVAILABLE:
            return await self._llm_extract_schema(raw_content, hardware_type)
        else:
            return await self._rule_based_extract_schema(raw_content, hardware_type)

    async def _llm_extract_schema(
        self,
        raw_content: str,
        hardware_type: Literal['serial', 'dds', 'ros', 'http']
    ) -> Dict[str, Any]:
        """使用 LLM 提取协议模式"""

        # 构建提示词
        prompt = self._build_analysis_prompt(raw_content, hardware_type)

        try:
            response = await litellm.acompletion(
                model="claude-3-5-sonnet-20241022",
                messages=[
                    {
                        "role": "system",
                        "content": (
                            "你是一个硬件协议分析专家。你的任务是分析硬件接口文档，"
                            "提取出结构化的动作命令和状态反馈定义。"
                            "必须返回有效的 JSON 格式，不要包含任何其他文本。"
                        )
                    },
                    {"role": "user", "content": prompt}
                ],
                temperature=0.1,
                max_tokens=4000,
                response_format={"type": "json_object"}
            )

            result_text = response.choices[0].message.content
            schema = json.loads(result_text)
            return self._normalize_schema(schema)

        except Exception as e:
            print(f"      ⚠ LLM 分析失败，回退到规则分析: {e}")
            return await self._rule_based_extract_schema(raw_content, hardware_type)

    def _build_analysis_prompt(
        self,
        raw_content: str,
        hardware_type: Literal['serial', 'dds', 'ros', 'http']
    ) -> str:
        """构建分析提示词"""

        prompt = f"""分析以下硬件接口文档，提取结构化的协议定义。

硬件通信类型: {hardware_type}

文档内容:
```
{raw_content[:15000]}  # 限制输入长度
```

请返回以下 JSON 格式的协议描述:

```json
{{
  "actions": [
    {{
      "name": "动作名称（英文，snake_case）",
      "params": {{
        "参数名": "参数类型（int/float/bool/str/list）",
        "...": "..."
      }},
      "description": "动作的中文描述",
      "command_code": "十六进制命令码（如 0x01），可选"
    }}
  ],
  "states": [
    {{
      "name": "状态名称（英文，snake_case）",
      "type": "数据类型（int/float/bool/str）",
      "description": "状态的中文描述",
      "unit": "单位（如 degrees, meters, %），可选"
    }}
  ],
  "protocol_details": {{
    "endian": "little 或 big",
    "header": "帧头十六进制（如 0xA8 0xE5），可选",
    "checksum": "校验算法（如 crc16, crc32, sum），可选",
    "baudrate": 波特率数字（串口），可选,
    "frequency": 通信频率Hz（如 100），可选
  }},
  "hardware_info": {{
    "name": "硬件名称",
    "description": "硬件描述",
    "safety_limits": {{
      "max_velocity": "最大速度限制",
      "max_angle": "最大角度限制",
      "...": "..."
    }}
  }}
}}
```

要求:
1. actions 必须包含所有可执行的控制命令
2. states 必须包含所有可读取的传感器/状态数据
3. protocol_details 必须准确反映文档中的通信参数
4. 所有字符串使用中文描述
5. 必须返回严格的 JSON 格式，不要包含 markdown 代码块标记
"""
        return prompt

    async def _rule_based_extract_schema(
        self,
        raw_content: str,
        hardware_type: Literal['serial', 'dds', 'ros', 'http']
    ) -> Dict[str, Any]:
        """
        基于规则的协议提取（LLM 不可用时的回退方案）
        """
        print("   🔧 使用规则分析模式...")

        schema = {
            "actions": [],
            "states": [],
            "protocol_details": {},
            "hardware_info": {"name": "Unknown", "description": ""}
        }

        # 提取协议细节（串口）
        if hardware_type == 'serial':
            # 尝试匹配波特率
            baud_match = re.search(r'(\d{4,6})\s*(bps|baud)', raw_content, re.IGNORECASE)
            if baud_match:
                schema["protocol_details"]["baudrate"] = int(baud_match.group(1))

            # 尝试匹配帧头
            header_match = re.search(r'(0x[0-9A-Fa-f]{2})\s*(0x[0-9A-Fa-f]{2})\s*帧头', raw_content)
            if header_match:
                schema["protocol_details"]["header"] = f"{header_match.group(1)} {header_match.group(2)}"

            # 尝试匹配校验算法
            if re.search(r'crc16|CRC16', raw_content):
                schema["protocol_details"]["checksum"] = "crc16"
            elif re.search(r'crc32|CRC32', raw_content):
                schema["protocol_details"]["checksum"] = "crc32"
            elif re.search(r'校验和|checksum', raw_content, re.IGNORECASE):
                schema["protocol_details"]["checksum"] = "sum"

        # 提取动作（从函数定义）
        # Python 函数
        py_funcs = re.findall(r'def\s+(\w+)\s*\(([^)]*)\)', raw_content)
        for func_name, params in py_funcs:
            if not func_name.startswith('_'):
                schema["actions"].append({
                    "name": func_name,
                    "params": {"args": "str"},
                    "description": f"Python 函数 {func_name}"
                })

        # C++ 函数
        cpp_funcs = re.findall(r'(void|int|float|bool)\s+(\w+)\s*\(([^)]*)\)', raw_content)
        for ret_type, func_name, params in cpp_funcs:
            if not func_name.startswith('_'):
                schema["actions"].append({
                    "name": func_name,
                    "params": {"args": "str"},
                    "description": f"C++ 函数 {func_name}"
                })

        return schema

    def _normalize_schema(self, schema: Dict[str, Any]) -> Dict[str, Any]:
        """规范化模式，确保所有必需字段存在"""
        normalized = {
            "actions": schema.get("actions", []),
            "states": schema.get("states", []),
            "protocol_details": schema.get("protocol_details", {}),
            "hardware_info": schema.get("hardware_info", {"name": "Unknown", "description": ""})
        }

        # 确保 protocol_details 有默认值
        proto = normalized["protocol_details"]
        proto.setdefault("endian", "little")
        proto.setdefault("header", None)
        proto.setdefault("checksum", None)

        return normalized
