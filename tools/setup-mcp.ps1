# setup-mcp.ps1
# Configures MCP for Claude Code and VS Code to connect to the roboRIO MCP server.
# Run this script once on each laptop after cloning the repo.
#
# Usage:
#   .\tools\setup-mcp.ps1              # simulation (localhost)
#   .\tools\setup-mcp.ps1 -Robot       # real robot (10.105.98.2)
#   .\tools\setup-mcp.ps1 -Ip 10.0.0.5 # custom IP

param(
    [switch]$Robot,
    [string]$Ip
)

$RoboRioIp = if ($Ip) { $Ip } elseif ($Robot) { "10.105.98.2" } else { "localhost" }
$McpUrl    = "http://${RoboRioIp}:8765/mcp"
$ProjectRoot = Split-Path $PSScriptRoot -Parent

Write-Host "Setting up MCP servers pointing to: $McpUrl" -ForegroundColor Cyan

# ── 1. Claude Code — project .mcp.json ───────────────────────────────────────
$ClaudeMcpPath = Join-Path $ProjectRoot ".mcp.json"
$ClaudeMcp = @{
    mcpServers = @{
        roborio = @{
            type = "http"
            url  = $McpUrl
        }
    }
}
$ClaudeMcp | ConvertTo-Json -Depth 4 | Set-Content $ClaudeMcpPath -Encoding UTF8
Write-Host "  [OK] Claude Code: $ClaudeMcpPath" -ForegroundColor Green

# ── 2. VS Code — .vscode/mcp.json ────────────────────────────────────────────
$VscodeMcpPath = Join-Path $ProjectRoot ".vscode\mcp.json"
$VscodeMcp = @{
    servers = @{
        roborio = @{
            type = "http"
            url  = $McpUrl
        }
    }
}
$VscodeMcp | ConvertTo-Json -Depth 4 | Set-Content $VscodeMcpPath -Encoding UTF8
Write-Host "  [OK] VS Code:      $VscodeMcpPath" -ForegroundColor Green

# ── 3. Claude Desktop (optional, only if installed) ──────────────────────────
$DesktopConfig = "$env:APPDATA\Claude\claude_desktop_config.json"
if (Test-Path (Split-Path $DesktopConfig)) {
    $Config = if (Test-Path $DesktopConfig) {
        Get-Content $DesktopConfig -Raw | ConvertFrom-Json -AsHashtable
    } else {
        @{}
    }

    if (-not $Config.ContainsKey("mcpServers")) { $Config["mcpServers"] = @{} }
    $Config["mcpServers"]["roborio"] = @{ url = $McpUrl }

    $Config | ConvertTo-Json -Depth 6 | Set-Content $DesktopConfig -Encoding UTF8
    Write-Host "  [OK] Claude Desktop: $DesktopConfig" -ForegroundColor Green
} else {
    Write-Host "  [--] Claude Desktop not found, skipping." -ForegroundColor DarkGray
}

Write-Host ""
Write-Host "Done. Start simulation (or connect to robot) then open Claude Code or VS Code Copilot." -ForegroundColor Cyan
if (-not $Robot -and -not $Ip) {
    Write-Host "Tip: run with -Robot to point at the real roboRIO (10.105.98.2)." -ForegroundColor Yellow
}
