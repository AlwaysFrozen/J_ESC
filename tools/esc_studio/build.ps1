param(
    [string]$QtRoot,
    [string]$CompilerBin,
    [string]$BuildDirectory,
    [switch]$Test,
    [switch]$Deploy
)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
$projectRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
if (-not $QtRoot) { $QtRoot = Join-Path $projectRoot 'build/qt-sdk/6.8.3/mingw_64' }
if (-not $CompilerBin) { $CompilerBin = Join-Path $projectRoot 'build/qt-sdk/Tools/mingw1310_64/bin' }
if (-not $BuildDirectory) { $BuildDirectory = Join-Path $projectRoot 'build/esc-studio' }
$QtRoot = (Resolve-Path -LiteralPath $QtRoot).Path
$CompilerBin = (Resolve-Path -LiteralPath $CompilerBin).Path
$BuildDirectory = [IO.Path]::GetFullPath($BuildDirectory)
foreach ($required in @((Join-Path $QtRoot 'bin/qmake.exe'), (Join-Path $CompilerBin 'g++.exe'))) {
    if (-not (Test-Path -LiteralPath $required)) { throw "Missing build tool: $required" }
}
$originalEnvironment = @{}
foreach ($name in @('PATH', 'TMP', 'TEMP', 'J_ESC_TEST_BRIDGE')) {
    $originalEnvironment[$name] = [Environment]::GetEnvironmentVariable($name, 'Process')
}
function Assert-Exit([string]$stage) { if ($LASTEXITCODE -ne 0) { throw "$stage failed (exit $LASTEXITCODE)" } }
try {
    $env:PATH = "$CompilerBin;$(Join-Path $QtRoot 'bin');$env:PATH"
    & cmake -S $PSScriptRoot -B $BuildDirectory -G Ninja -DCMAKE_BUILD_TYPE=Release `
        "-DCMAKE_PREFIX_PATH=$QtRoot" "-DCMAKE_C_COMPILER=$(Join-Path $CompilerBin 'gcc.exe')" `
        "-DCMAKE_CXX_COMPILER=$(Join-Path $CompilerBin 'g++.exe')"
    Assert-Exit 'Configure'
    & cmake --build $BuildDirectory --parallel 4
    Assert-Exit 'Build'
    if ($Test) {
        $temporary = Join-Path $BuildDirectory 'tmp'
        New-Item -ItemType Directory -Path $temporary -Force | Out-Null
        $env:TMP = $temporary; $env:TEMP = $temporary
        $bridge = Join-Path $projectRoot 'build/od-tests/scope_bridge.dll'
        if (Test-Path -LiteralPath $bridge) { $env:J_ESC_TEST_BRIDGE = $bridge }
        else { Write-Warning 'Native firmware bridge not built; interoperability test will be skipped.' }
        & ctest --test-dir $BuildDirectory --output-on-failure
        Assert-Exit 'Tests'
        Get-Content -LiteralPath (Join-Path $BuildDirectory 'ui-tests.txt')
        Get-Content -LiteralPath (Join-Path $BuildDirectory 'scope-tests.txt')
        $renderMetrics = Join-Path $BuildDirectory 'render-metrics.json'
        if (Test-Path -LiteralPath $renderMetrics) { Get-Content -LiteralPath $renderMetrics }
    }
    if ($Deploy) {
        $package = Join-Path $BuildDirectory 'package'
        New-Item -ItemType Directory -Path $package -Force | Out-Null
        $binary = Join-Path $BuildDirectory 'esc_studio.exe'
        & (Join-Path $QtRoot 'bin/windeployqt.exe') --release --no-translations --include-plugins qoffscreen `
            --no-opengl-sw --no-system-d3d-compiler --compiler-runtime `
            --qmldir (Join-Path $PSScriptRoot 'qml') --dir $package $binary
        Assert-Exit 'Qt deployment'
        Copy-Item -LiteralPath $binary -Destination (Join-Path $package 'esc_studio.exe') -Force
        Write-Host "Standalone application: $(Join-Path $package 'esc_studio.exe')"
    }
} finally {
    foreach ($name in $originalEnvironment.Keys) {
        [Environment]::SetEnvironmentVariable($name, $originalEnvironment[$name], 'Process')
    }
}
