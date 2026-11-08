param([switch]$Demo,[switch]$Scope)
$ErrorActionPreference = 'Stop'
$projectRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$build = Join-Path $projectRoot 'build/esc-studio'
$binary = Join-Path $build 'package/esc_studio.exe'
if (-not (Test-Path -LiteralPath $binary)) {
    $binary = Join-Path $build 'esc_studio.exe'
    $env:PATH = "$(Join-Path $projectRoot 'build/qt-sdk/Tools/mingw1310_64/bin');$(Join-Path $projectRoot 'build/qt-sdk/6.8.3/mingw_64/bin');$env:PATH"
}
if (-not (Test-Path -LiteralPath $binary)) { throw 'Run tools/esc_studio/build.ps1 -Deploy first.' }
$launchArguments = @()
if ($Demo) { $launchArguments += '--demo' }
if ($Demo -and $Scope) { $launchArguments += '--scope-demo' }
& $binary @launchArguments
