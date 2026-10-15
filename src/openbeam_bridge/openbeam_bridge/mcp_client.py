"""A small, dependency-free MCP (Streamable HTTP, JSON responses) client.

It talks to either
  * a local OpenBeam game started with ROR_AGENT_PORT (its endpoint file gives
    host, port, token and mcpPath), or
  * a hosted session: https://cloud.openbeam.world/v1/sessions/<id>/mcp with a
    device-code access token.

Only the standard library is used so the same module runs inside the ROS 2
image, on a student's laptop and in the unit tests.
"""

from __future__ import annotations

import base64
import http.client
import itertools
import json
import os
import ssl
import threading
import time
import urllib.parse
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

PROTOCOL_VERSION = "2025-06-18"


class McpError(RuntimeError):
    """A transport or JSON-RPC failure (not a tool's own refusal)."""


class ToolError(RuntimeError):
    """The tool ran and answered isError: true."""

    def __init__(self, tool: str, message: str, payload: Any = None):
        super().__init__(f"{tool}: {message}")
        self.tool = tool
        self.payload = payload


@dataclass
class Endpoint:
    url: str
    token: str

    def redacted(self) -> str:
        return f"{self.url} (token {'set' if self.token else 'missing'})"


def endpoint_from_file(path: str, host_override: Optional[str] = None) -> Endpoint:
    """Read a local game's agent-endpoint.json (written by ROR_AGENT_ENDPOINT_FILE)."""
    with open(path, "r", encoding="utf-8") as handle:
        data = json.load(handle)
    host = host_override or data.get("host") or "127.0.0.1"
    port = int(data["port"])
    mcp_path = data.get("mcpPath") or "/mcp"
    return Endpoint(url=f"http://{host}:{port}{mcp_path}", token=str(data.get("token", "")))


def endpoint_from_env(env: Optional[Dict[str, str]] = None) -> Endpoint:
    """Resolve the endpoint from the environment.

    OPENBEAM_MCP_URL + OPENBEAM_MCP_TOKEN (or OPENBEAM_MCP_TOKEN_FILE) win;
    otherwise OPENBEAM_ENDPOINT_FILE (+ optional OPENBEAM_ENDPOINT_HOST, e.g.
    host.docker.internal when the bridge runs in Docker beside a local game).
    OPENBEAM_SESSION_ID alone builds the hosted session URL.
    """
    env = dict(os.environ if env is None else env)
    token = env.get("OPENBEAM_MCP_TOKEN", "")
    token_file = env.get("OPENBEAM_MCP_TOKEN_FILE", "")
    if not token and token_file:
        with open(token_file, "r", encoding="utf-8") as handle:
            token = handle.read().strip()
    url = env.get("OPENBEAM_MCP_URL", "")
    session = env.get("OPENBEAM_SESSION_ID", "")
    if not url and session:
        api = env.get("OPENBEAM_CLOUD_API_URL", "https://cloud.openbeam.world").rstrip("/")
        url = f"{api}/v1/sessions/{urllib.parse.quote(session, safe='')}/mcp"
    if url:
        return Endpoint(url=url, token=token)
    endpoint_file = env.get("OPENBEAM_ENDPOINT_FILE", "")
    if endpoint_file:
        endpoint = endpoint_from_file(endpoint_file, env.get("OPENBEAM_ENDPOINT_HOST") or None)
        if token:
            endpoint.token = token
        return endpoint
    raise McpError(
        "no OpenBeam endpoint: set OPENBEAM_MCP_URL (+ OPENBEAM_MCP_TOKEN), "
        "OPENBEAM_SESSION_ID (+ token) or OPENBEAM_ENDPOINT_FILE")


class McpClient:
    """Thread-safe JSON-RPC over HTTP POST.

    A connection is kept per thread while the server allows it: the cloud
    and games with MCP keep-alive reuse it (no TCP or TLS setup per tick);
    older games answer Connection: close and get a fresh one per call.
    """

    def __init__(self, endpoint: Endpoint, timeout: float = 10.0, user_agent: str = "openbeam-ros2-bridge/0.1"):
        self.endpoint = endpoint
        self.timeout = timeout
        self.user_agent = user_agent
        parsed = urllib.parse.urlsplit(endpoint.url)
        if parsed.scheme not in ("http", "https"):
            raise McpError(f"unsupported URL scheme in {endpoint.url!r}")
        self._https = parsed.scheme == "https"
        self._host = parsed.hostname or "127.0.0.1"
        self._port = parsed.port or (443 if self._https else 80)
        self._path = (parsed.path or "/mcp") + (f"?{parsed.query}" if parsed.query else "")
        self._ids = itertools.count(1)
        self._local = threading.local()
        self._ssl = ssl.create_default_context() if self._https else None
        self.protocol_version: Optional[str] = None
        self.server_info: Dict[str, Any] = {}
        self._tools: Optional[Dict[str, Dict[str, Any]]] = None

    # -- transport -------------------------------------------------------
    def _connection(self) -> http.client.HTTPConnection:
        conn = getattr(self._local, "conn", None)
        if conn is None:
            if self._https:
                conn = http.client.HTTPSConnection(self._host, self._port, timeout=self.timeout, context=self._ssl)
            else:
                conn = http.client.HTTPConnection(self._host, self._port, timeout=self.timeout)
            self._local.conn = conn
        return conn

    def _drop(self) -> None:
        conn = getattr(self._local, "conn", None)
        if conn is not None:
            try:
                conn.close()
            except Exception:  # pragma: no cover - best effort
                pass
        self._local.conn = None

    def post(self, payload: Any) -> Any:
        body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        headers = {
            "Content-Type": "application/json",
            "Accept": "application/json, text/event-stream",
            "User-Agent": self.user_agent,
        }
        if self.endpoint.token:
            headers["Authorization"] = f"Bearer {self.endpoint.token}"
        if self.protocol_version:
            headers["MCP-Protocol-Version"] = self.protocol_version
        last_error: Optional[Exception] = None
        for attempt in range(2):
            conn = self._connection()
            try:
                conn.request("POST", self._path, body=body, headers=headers)
                response = conn.getresponse()
                data = response.read()
                # Keep the connection when the server allows it (the cloud, and
                # games with MCP keep-alive); older games answer Connection: close.
                closing = (response.getheader("Connection", "") or "").lower() == "close" or response.will_close
                if closing:
                    self._drop()
            except (http.client.HTTPException, OSError) as error:
                self._drop()
                last_error = error
                # A kept-alive connection the server already closed: retry once.
                if attempt == 0:
                    continue
                raise McpError(f"POST {self.endpoint.url}: {error}") from error
            if response.status == 202:
                return None
            if response.status == 429:
                retry = response.getheader("Retry-After") or "1"
                raise McpError(f"rate limited (429), retry after {retry}s")
            if response.status != 200:
                text = data[:400].decode("utf-8", "replace")
                raise McpError(f"HTTP {response.status} from {self.endpoint.url}: {text}")
            ctype = (response.getheader("Content-Type", "") or "").lower()
            if "text/event-stream" in ctype:
                return _parse_sse(data)
            try:
                return json.loads(data)
            except ValueError as error:
                raise McpError(f"non-JSON answer from {self.endpoint.url}") from error
        raise McpError(f"POST {self.endpoint.url}: {last_error}")

    def rpc(self, method: str, params: Optional[Dict[str, Any]] = None) -> Any:
        message = {"jsonrpc": "2.0", "id": next(self._ids), "method": method, "params": params or {}}
        answer = self.post(message)
        if not isinstance(answer, dict):
            raise McpError(f"{method}: unexpected answer {answer!r}")
        if "error" in answer:
            error = answer["error"] or {}
            raise McpError(f"{method}: {error.get('message', error)}")
        return answer.get("result")

    # -- MCP --------------------------------------------------------------
    def initialize(self) -> Dict[str, Any]:
        result = self.rpc("initialize", {
            "protocolVersion": PROTOCOL_VERSION,
            "capabilities": {},
            "clientInfo": {"name": "openbeam-ros2-bridge", "version": "0.1.0"},
        }) or {}
        self.protocol_version = result.get("protocolVersion", PROTOCOL_VERSION)
        self.server_info = result.get("serverInfo", {})
        try:
            self.post({"jsonrpc": "2.0", "method": "notifications/initialized"})
        except McpError:
            pass
        return result

    def tools(self, refresh: bool = False) -> Dict[str, Dict[str, Any]]:
        if self._tools is None or refresh:
            result = self.rpc("tools/list") or {}
            self._tools = {tool["name"]: tool for tool in result.get("tools", [])}
        return self._tools

    def has_tool(self, name: str) -> bool:
        return name in self.tools()

    def call_raw(self, name: str, arguments: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        return self.rpc("tools/call", {"name": name, "arguments": arguments or {}}) or {}

    def call(self, name: str, arguments: Optional[Dict[str, Any]] = None) -> Any:
        """Call a tool and return its structured (JSON) answer."""
        result = self.call_raw(name, arguments)
        payload = tool_payload(result)
        if result.get("isError"):
            message = payload.get("message") if isinstance(payload, dict) else None
            raise ToolError(name, message or json.dumps(payload)[:400], payload)
        return payload

    def call_batch(self, calls: List[tuple]) -> List[Any]:
        """Several tools/call in one HTTP request (the game accepts batches).

        Note the game still runs each call on its own frame, so a batch saves
        round trips, not frames.
        """
        messages = []
        for name, arguments in calls:
            messages.append({"jsonrpc": "2.0", "id": next(self._ids), "method": "tools/call",
                             "params": {"name": name, "arguments": arguments or {}}})
        answer = self.post(messages)
        if not isinstance(answer, list):
            raise McpError("batch: unexpected answer")
        by_id = {item.get("id"): item for item in answer if isinstance(item, dict)}
        out = []
        for message in messages:
            item = by_id.get(message["id"], {})
            if "error" in item:
                out.append(McpError(str(item["error"])))
                continue
            result = item.get("result") or {}
            payload = tool_payload(result)
            out.append(ToolError(message["params"]["name"], str(payload)[:400], payload)
                       if result.get("isError") else payload)
        return out

    def image(self, name: str = "screenshot", arguments: Optional[Dict[str, Any]] = None, with_meta: bool = False):
        """Call an image-returning tool; answer (mime, bytes[, structured]) or raise."""
        result = self.call_raw(name, arguments)
        if result.get("isError"):
            raise ToolError(name, json.dumps(tool_payload(result))[:400])
        for item in result.get("content", []):
            if item.get("type") == "image" and item.get("data"):
                data = base64.b64decode(item["data"])
                mime = item.get("mimeType", "image/png")
                return (mime, data, tool_payload(result)) if with_meta else (mime, data)
        raise ToolError(name, "no inline image in the answer (too large? the game returned a path)",
                        tool_payload(result))


def tool_payload(result: Dict[str, Any]) -> Any:
    if "structuredContent" in result and result["structuredContent"] is not None:
        return result["structuredContent"]
    for item in result.get("content", []) or []:
        if item.get("type") == "text":
            text = item.get("text", "")
            try:
                return json.loads(text)
            except ValueError:
                return {"text": text}
    return {}


def _parse_sse(data: bytes) -> Any:
    """Return the last JSON-RPC message carried in an SSE body."""
    last = None
    for block in data.decode("utf-8", "replace").split("\n\n"):
        lines = [line[5:].lstrip() for line in block.splitlines() if line.startswith("data:")]
        if lines:
            try:
                last = json.loads("\n".join(lines))
            except ValueError:
                continue
    if last is None:
        raise McpError("empty event stream")
    return last


def wait_for(predicate, timeout: float, interval: float = 0.25) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return predicate()
